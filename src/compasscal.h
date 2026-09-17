#pragma once
#ifndef COMPASSCAL_H
#define COMPASSCAL_H

// Continuous, autonomous magnetometer hard-iron calibration. Chip agnostic: MotionManager feeds it calibrated mag
// samples (uT, mag package frame) and implements MagBiasPort, which reads/writes the hard-iron offset wherever the chip
// keeps it. Runs on core1 with the sensors, always on from boot.
//
// Method: samples are accumulated into a window and least-squares fit to a sphere (Kasa fit: |m|^2 = 2 m.c + k is linear
// in the center c, so the window reduces to a 4x4 normal-equation system solved once per evaluation). The fitted center is
// the residual hard iron; it is applied in one step and a fresh window begins. A window is only evaluated once every axis
// spans enough of the fitted diameter, and the correction is only applied if it passes the quality gates:
//   - the fitted radius is a plausible geomagnetic field,
//   - the radial RMS residual is small (a stray or moving field scatters the samples off the sphere),
//   - the fitted radius agrees with the last accepted radius (a static stray field tumbled along with the device makes a
//     clean sphere of the wrong size), and
//   - a trusted offset moves by at most a bounded amount per correction.
// Windows that fail a gate are discarded without touching the offset. Windows that never reach coverage expire so old
// extremes can't linger. There is no failure state, only more collecting.
//
// Trust ("calibrated") means the offset in effect came from an accepted fit, a stored offset restored at boot, or the
// per-revision seed. It says nothing about the field the device is in right now; that is judged on core0 (compass.h).
// COMPASSCAL discards the offset and restarts from zero.
//
// Sample intake is rate-limited to kSampleIntervalMS so coverage and timing mean the same thing regardless of chip rate.
// core0 persists the offset whenever biasGeneration changes while calibrated and restores it at boot via noteRestoredBias.

#include <Arduino.h>
#include <math.h>

enum class CalState : uint8_t {
  Off,        // no magnetometer
  Collecting, // accumulating a window
  Applying,   // writing an offset correction (transient)
};

struct CompassCalStatus {
  CalState state = CalState::Off;
  bool calibrated = false;       // the offset in effect is trusted: an accepted fit, a stored record or the seed (vs. zero)
  uint8_t corrections = 0;       // offset corrections applied since boot (wraps)
  uint8_t rejected = 0;          // windows discarded by the quality gates since boot (wraps)
  uint32_t windows = 0;          // windows evaluated since boot
  uint32_t sampleCount = 0;      // samples in the current window
  float axisMin[3] = {0};        // current window, uT
  float axisMax[3] = {0};
  float magNormMin = 0;          // |B| over the current window, uT
  float magNormMax = 0;
  float lastResidual = 0;        // |fitted center| at the last evaluation, uT
  float lastRadius = 0;          // fitted |B| at the last evaluation, uT
  float lastRMS = 0;             // radial RMS of the last evaluation, uT
  float lastSpread = 0;          // |B| max-min of the last evaluated window, uT
  float bias[3] = {0};           // offset in effect, uT, mag package frame, as of the last generation bump or restore
  float acceptedRadius = 0;      // fitted |B| of the last accepted window (or as restored), uT; 0 = none yet
  uint32_t biasGeneration = 0;   // bumps when `bias` changes while trusted; core0 persists on change. 0 = as restored.
};

// Where the chip keeps its hard-iron offset. calibrated = raw - bias, uT, mag package frame.
struct MagBiasPort {
  virtual bool readMagBias(float bias[3]) = 0;
  virtual bool writeMagBias(const float bias[3]) = 0;
};

class CompassCalibrator {
public:
  // A stored (or seeded) offset was written into the chip at init: trust it until a fit says otherwise. core1.
  void noteRestoredBias(const float b[3], float acceptedRadius) {
    for (int i = 0; i < 3; ++i) _status.bias[i] = b[i];
    _status.acceptedRadius = acceptedRadius;
    _status.calibrated = true;
  }

  // COMPASSCAL: discard the offset in effect and start over from zero. core1.
  void restart(MagBiasPort &port) {
    float zero[3] = {0, 0, 0};
    port.writeMagBias(zero);
    for (int i = 0; i < 3; ++i) _status.bias[i] = 0;
    _status.acceptedRadius = 0;
    _status.calibrated = false;
    _status.corrections = 0;
    resetWindow();
  }

  // Feed a calibrated (current offset already applied) mag sample, uT, mag package frame.
  void onMagSample(const float m[3]) {
    if (_status.state == CalState::Off) return;
    unsigned long now = millis();
    if (_lastSampleMillis != 0 && now - _lastSampleMillis < kSampleIntervalMS) return;
    _lastSampleMillis = now;

    float norm = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
    if (_status.sampleCount == 0) { // window start
      _windowStartMillis = now;
      for (int i = 0; i < 3; ++i) { _status.axisMin[i] = _status.axisMax[i] = m[i]; _ref[i] = m[i]; }
      _status.magNormMin = _status.magNormMax = norm;
    } else {
      for (int i = 0; i < 3; ++i) {
        _status.axisMin[i] = min(_status.axisMin[i], m[i]);
        _status.axisMax[i] = max(_status.axisMax[i], m[i]);
      }
      _status.magNormMin = min(_status.magNormMin, norm);
      _status.magNormMax = max(_status.magNormMax, norm);
    }
    accumulate(m);
    _status.sampleCount++;
    _dirty = true;
  }

  // Run the state machine. Called once per MotionManager::loop() on core1.
  void loop(MagBiasPort &port, bool haveMagnetometer) {
    if (!haveMagnetometer) { _status.state = CalState::Off; return; }
    if (_status.state == CalState::Off) { _status.state = CalState::Collecting; resetWindow(); }

    if (!_dirty) return;
    _dirty = false;
    if (_status.sampleCount < kMinSamples || !prefilterCoverage()) {
      // Stale window: samples from wherever the device was minutes ago shouldn't join a tumble happening now.
      if (_status.sampleCount > 0 && millis() - _windowStartMillis > kWindowMaxMillis) resetWindow();
      return;
    }
    evaluateWindow(port);
  }

  const CompassCalStatus &status() const { return _status; }
  bool calibrated() const { return _status.calibrated; }

private:
  static constexpr unsigned long kSampleIntervalMS = 100; // 10Hz sample intake regardless of chip rate
  static constexpr uint32_t kMinSamples = 50;             // 5s of handling before a fit is attempted
  static constexpr float kPrefilterAxisRange = 40.0f;     // uT: don't bother fitting until every axis has moved this much
  static constexpr float kCoverageFrac = 0.6f;            // each axis must span this fraction of the fitted diameter
  // Discard a window that never reaches coverage: a deliberate tumble takes 20-60s, and a shorter window mixes in less of
  // wherever the device was before and less thermal drift.
  static constexpr unsigned long kWindowMaxMillis = 90UL * 1000UL;
  // Quality gates on the fit. Geomagnetic |B| is 25-65uT worldwide. A clean window fits to ~1-2uT radial RMS (sensor
  // noise, a little soft iron, a little thermal drift within the window); a stray field in the window is tens to hundreds.
  static constexpr float kFitRadiusMin = 20.0f;   // uT
  static constexpr float kFitRadiusMax = 70.0f;   // uT: earth never exceeds ~65
  static constexpr float kFitMaxRMS = 6.0f;       // uT
  // The local field's magnitude does not change between tumbles: a fit whose radius disagrees with the last accepted one
  // by more than this is earth plus something else (a phone under the device fits a clean sphere with a bigger R).
  static constexpr float kRadiusAgreeUT = 10.0f;
  // A trusted offset only drifts (thermal: ~12uT hot vs cold on v7). A larger single correction is a contaminated window.
  static constexpr float kMaxTrustedCorrectionUT = 30.0f;
  // A residual under this is noise: trust the offset as is. The ICM's DMP runtime cal moves its registers a few uT.
  static constexpr float kConvergedThresh = 5.0f; // uT
  // Runaway guard: hard iron is ~140uT on v7 and ~44uT on v5/v6. An offset beyond this can only be a compounding bug or
  // hopeless data; restart from zero rather than chase it.
  static constexpr float kBiasMax = 400.0f;       // uT
  static constexpr float kGenerationMinDelta = 0.5f; // uT: don't bump the generation (and a flash write) for noise

  void resetWindow() {
    _status.sampleCount = 0;
    for (int i = 0; i < 3; ++i) { _status.axisMin[i] = _status.axisMax[i] = 0; }
    _status.magNormMin = _status.magNormMax = 0;
    for (int i = 0; i < 4; ++i) { _sumAb[i] = 0; for (int j = 0; j < 4; ++j) _sumAA[i][j] = 0; }
    _sumBB = 0;
    _dirty = false;
  }

  bool prefilterCoverage() const {
    for (int i = 0; i < 3; ++i) {
      if ((_status.axisMax[i] - _status.axisMin[i]) < kPrefilterAxisRange) return false;
    }
    return true;
  }

  // Normal equations for |p|^2 = 2 p.c' + k, p = m - ref (the window's first sample, so the sums stay well conditioned).
  // Doubles: the sums of fourth-power terms outgrow a float mantissa within a window. 14 soft-double MACs at 10Hz is nothing.
  void accumulate(const float m[3]) {
    double p[3] = {m[0] - _ref[0], m[1] - _ref[1], m[2] - _ref[2]};
    double a[4] = {2 * p[0], 2 * p[1], 2 * p[2], 1};
    double b = p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j <= i; ++j) _sumAA[i][j] += a[i] * a[j];
      _sumAb[i] += a[i] * b;
    }
    _sumBB += b * b;
  }

  // Solve the fit. Returns false if the system is singular (samples coplanar, or too few).
  bool fit(double center[3], double *radius, double *rms) const {
    double A[4][5];
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) A[i][j] = (j <= i) ? _sumAA[i][j] : _sumAA[j][i];
      A[i][4] = _sumAb[i];
    }
    for (int col = 0; col < 4; ++col) { // Gaussian elimination, partial pivoting
      int piv = col;
      for (int r = col + 1; r < 4; ++r) if (fabs(A[r][col]) > fabs(A[piv][col])) piv = r;
      if (fabs(A[piv][col]) < 1e-9) return false;
      if (piv != col) for (int j = 0; j < 5; ++j) { double tmp = A[col][j]; A[col][j] = A[piv][j]; A[piv][j] = tmp; }
      for (int r = 0; r < 4; ++r) {
        if (r == col) continue;
        double f = A[r][col] / A[col][col];
        for (int j = col; j < 5; ++j) A[r][j] -= f * A[col][j];
      }
    }
    double x[4];
    for (int i = 0; i < 4; ++i) x[i] = A[i][4] / A[i][i];
    double r2 = x[3] + x[0] * x[0] + x[1] * x[1] + x[2] * x[2];
    if (!(r2 > 0)) return false;
    double R = sqrt(r2);
    // Algebraic residual sum (b - a.x)^2 = bb - 2 x.t + x'Sx, which is ~2R times the radial residual per sample.
    double xt = 0, xSx = 0;
    for (int i = 0; i < 4; ++i) {
      xt += x[i] * _sumAb[i];
      for (int j = 0; j < 4; ++j) xSx += x[i] * ((j <= i) ? _sumAA[i][j] : _sumAA[j][i]) * x[j];
    }
    double ssr = _sumBB - 2 * xt + xSx;
    if (ssr < 0) ssr = 0;
    for (int i = 0; i < 3; ++i) center[i] = _ref[i] + x[i];
    *radius = R;
    *rms = sqrt(ssr / _status.sampleCount) / (2 * R);
    return true;
  }

  // The offset in effect is trusted: record it (and the radius that vouched for it) for core0 to persist if it moved. The
  // generation follows the offset only; the radius jitters ~1uT between confirming windows and nobody needs to react to that.
  void promote(MagBiasPort &port, float radius) {
    float b[3] = {0, 0, 0};
    _status.acceptedRadius = radius;
    if (port.readMagBias(b)) {
      bool moved = false;
      for (int i = 0; i < 3; ++i) moved |= fabsf(b[i] - _status.bias[i]) > kGenerationMinDelta;
      for (int i = 0; i < 3; ++i) _status.bias[i] = b[i];
      if (moved) _status.biasGeneration++;
    }
    _status.calibrated = true;
  }

  void evaluateWindow(MagBiasPort &port) {
    double c[3], R, rms;
    if (!fit(c, &R, &rms)) return; // degenerate so far; keep collecting
    // Coverage against the fitted sphere: every axis must span most of the diameter, else keep collecting. R is clamped to
    // the plausible range so a contaminated window (absurd R) still reaches evaluation and gets rejected instead of
    // sitting there until it ages out.
    double coverageR = min(max(R, (double)kFitRadiusMin), (double)kFitRadiusMax);
    for (int i = 0; i < 3; ++i) {
      if ((_status.axisMax[i] - _status.axisMin[i]) < kCoverageFrac * 2 * coverageR) return;
    }

    float residual = sqrtf(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
    _status.windows++;
    _status.lastResidual = residual;
    _status.lastRadius = R;
    _status.lastRMS = rms;
    _status.lastSpread = _status.magNormMax - _status.magNormMin;

    bool radiusDisagrees = _status.acceptedRadius > 0 && fabs(R - _status.acceptedRadius) > kRadiusAgreeUT;
    bool tooBigForTrusted = _status.calibrated && residual > kMaxTrustedCorrectionUT;
    if (R < kFitRadiusMin || R > kFitRadiusMax || rms > kFitMaxRMS || radiusDisagrees || tooBigForTrusted) {
      _status.rejected++; // contaminated window: leave the offset alone
      resetWindow();
      return;
    }
    if (residual < kConvergedThresh) {
      promote(port, R);
      resetWindow();
      return;
    }

    // Apply the fitted center. Read the current offset first (the ICM DMP's runtime cal may have moved it), then compound.
    // calibrated = raw - bias, so a positive center means the bias is too small.
    _status.state = CalState::Applying;
    float cur[3] = {0, 0, 0};
    if (port.readMagBias(cur)) {
      float nb[3];
      float nbMag = 0;
      for (int i = 0; i < 3; ++i) { nb[i] = cur[i] + (float)c[i]; nbMag += nb[i] * nb[i]; }
      if (sqrtf(nbMag) > kBiasMax) {
        restart(port);
      } else if (port.writeMagBias(nb)) {
        _status.corrections++;
        promote(port, R); // a fit that passed the gates is as good as a confirming window; don't make the user tumble twice
      }
    }
    _status.state = CalState::Collecting;
    resetWindow();
  }

  CompassCalStatus _status;
  unsigned long _lastSampleMillis = 0;
  unsigned long _windowStartMillis = 0;
  bool _dirty = false;
  float _ref[3] = {0, 0, 0};
  double _sumAA[4][4] = {{0}}, _sumAb[4] = {0}, _sumBB = 0;
};

#endif // COMPASSCAL_H
