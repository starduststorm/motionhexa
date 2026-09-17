#pragma once
#ifndef COMPASS_H
#define COMPASS_H

static bool compassLogging = false; // COMPASS serial command toggles a periodic heading diagnostic line

// Magnetic-north heading API. Runs on core0; call Compass::update() once per loop after getAsyncData(). Always compiled in.
//
// Method: the orientation quaternion's yaw is smooth and responsive but has an arbitrary boot-relative zero and, on
// 6-axis fusion, drifts. The tilt-compensated eCompass (accel + calibrated mag) gives an absolute north bearing but is
// noisy and only valid near-stationary. We track a slowly-filtered offset = (eCompass bearing + panelYaw), updated only
// when quasi-stationary, and output north = offset - panelYaw: the quaternion's smoothness with the eCompass's absolute
// reference, self-correcting after fast motion and immune to the boot zero.
//
// Stray fields: the offset only takes fixes from samples that agree with a learned baseline of the local field (|B| and
// dip, both independent of yaw, so a field bent by a nearby phone or speaker is recognised even when |B| stays plausible).
// Disagreeing samples are ignored and the heading holds on the quaternion's yaw alone until the last good fix is
// kHoldMillis old; then it drops to the "tumble me" dots and a fresh baseline is acquired from a few seconds of mutually
// consistent samples. A static stray field is indistinguishable from earth by steadiness alone, so the calibrator's
// fitted radius is the tiebreaker: a baseline whose |B| matches the accepted radius is vouched, one that doesn't is
// unvouched, and while unvouched the first steady stationary fix that does match re-anchors the heading on the spot. A
// rejected calibrator window while unvouched drops the heading; against a vouched baseline it is just the disturbance
// showing up in the calibrator too. The baseline tracks slowly enough to follow thermal drift without flagging it.
//
// All inputs are the hexa motion frame (MotionManager::loop() has already localized acc/gyr/mag/quat). Angles are CCW
// about +z_motion (out of the LED face) with 0 = +x_motion. The pixel geometry frame is this rotated 180deg about z;
// see MotionManager.h and CompassPattern in patterns.h.

#include <math.h>
#include "MotionManager.h"

namespace Compass {

// Filter state (sin/cos accumulators avoid the +/-pi wrap seam).
static float _offsetSin = 0, _offsetCos = 0;
static bool _offsetSeeded = false;    // offset and field baseline are valid; the arrow may be shown
static float _panelYaw = 0; // rad, latest
static uint32_t _lastMagCount = 0;
static unsigned long _lastMagMillis = 0;
static bool _magCalibrated = false;
static uint32_t _lastBiasGeneration = 0;
static float _lastBias[3] = {0, 0, 0}; // calibrator offset as of the previous loop, to size a correction
static bool _reseedPending = false;    // the offset moved a little: the next anchor replaces the filter state instead of nudging it
// Anchor dwell: a re-anchor (unvouched baseline meeting the calibrated field, or a small offset correction) seeds from a
// field that has been steady for kAnchorDwellMillis, not from one sample, since a single mid-motion sample can carry the
// right |B| with a wrong dip and anchor the heading tens of degrees off.
static unsigned long _anchorStart = 0;
static float _anchorBNorm = 0, _anchorDip = 0;
static uint8_t _lastRejected = 0;
static uint32_t _lastWindows = 0;
static bool _baselineVouched = false; // this baseline's |B| matches the calibrator's accepted radius (see header comment)
// Field baseline (learned from accepted fixes): what the calibrated earth field looks like here.
static float _refBNorm = 0, _refDip = 0; // uT, rad
static unsigned long _lastFixMillis = 0; // last time the offset took a fix from a clean stationary sample
static unsigned long _disturbedSince = 0; // 0 = field agrees with the baseline; else when it stopped agreeing
static unsigned long _cleanSince = 0;     // during a hold: when the field started agreeing again
// Acquisition (before the first fix, and again after a hold expires): a provisional baseline that consecutive samples must
// agree with for _acquireNeedMillis before it is adopted. Zero after a calibrator fit, since the fit already vouched for
// the field; kReacquireMillis after a hold expired, so a settled stray field has to at least be steady before it is
// believed (that is the fundamental limit: a static field is a static field).
static unsigned long _acquireStart = 0, _acquireNeedMillis = 0;
static float _acqBNorm = 0, _acqDip = 0;
// diagnostics, for the COMPASS serial log
static float _lastNorthInst = 0, _lastTarget = 0, _lastDip = 0, _lastBNorm = 0; // last stationary sample (fix or not)
static uint32_t _acceptedCount = 0; // fixes folded into the offset
static uint16_t _disturbances = 0, _holdExpiries = 0;

// Offset filter time constant.
static constexpr float kOffsetTauSeconds = 4.0f;
static constexpr float kOffsetMaxAlpha = 0.25f; // per sample, so a long gap can't snap to one noisy sample
// Field baseline time constant: slow enough to average a phone being waved past, fast enough to follow thermal drift
// (~0.15uT/s worst case measured on v7, i.e. a ~3uT lag at this tau).
static constexpr float kRefTauSeconds = 20.0f;
// A stationary sample disagrees with the baseline (and is ignored) beyond these. Per-sample noise is ~1uT / ~2deg. |B| is
// judged against the calibrator's fitted radius once there is one (a fixed anchor: the learned baseline can be dragged
// toward a disturbance that sits just inside tolerance); dip against the learned baseline, since the fit carries no
// orientation. Clean air at rest reads 2-3uT under the fitted
// radius on this unit, hand-held next to a phone 4-7uT under, the desk anomaly 12-15.
static constexpr float kDisturbBNormUT = 8.0f;
static constexpr float kDisturbDipRad = 10.0f * (float)M_PI / 180.0f;
// Absolute |B| plausibility for any fix or baseline: geomagnetic 25-65uT plus room for uncorrected drift.
static constexpr float kBNormMinUT = 15.0f;
static constexpr float kBNormMaxUT = 100.0f;
// How long the heading rides on yaw alone (stray field, or simply moving) before it is dropped. v7 6-axis yaw drifts
// ~3deg/min at rest cool and ~8deg/min hot, so this bounds the accumulated error to ~16deg.
static constexpr unsigned long kHoldMillis = 120UL * 1000UL;
// A held heading resumes taking fixes once the field has agreed with the baseline for this long.
static constexpr unsigned long kRecoverMillis = 2000;
// Consistent samples needed to adopt a baseline after a hold expired or a window was rejected.
static constexpr unsigned long kReacquireMillis = 3000;
// ... and to re-anchor an existing heading (half the disturbance tolerances must hold throughout).
static constexpr unsigned long kAnchorDwellMillis = 1500;
// A stationary |B| within this of the calibrator's accepted radius is the field the offset was fitted in. At rest in clean
// air this unit reads 2.5-3uT under its fitted radius (soft iron, thermal model residual); the desk anomaly was 15uT off.
static constexpr float kRadiusMatchUT = 6.0f;
// An offset correction smaller than this leaves the held heading roughly right (a few degrees against a ~22uT horizontal
// field), so the arrow stays up and the next fix re-anchors it. Larger, and the old heading is garbage: drop to dots.
static constexpr float kReseedDropUT = 15.0f;
// Quasi-stationary gate: gyro LSB per axis (16.4 LSB/dps => ~3.7 dps).
static constexpr int kStationaryGyroLSB = 60;
// Accel sanity for eCompass (8192 LSB/g): accept 0.9g .. 1.1g. Hand motion slow enough to pass the gyro gate still tilts
// the measured "up" by tens of degrees at 0.7-1.3g and the tilt compensation with it.
static constexpr float kAccelMinLSB = 0.9f * MotionManager::accelToGScale;
static constexpr float kAccelMaxLSB = 1.1f * MotionManager::accelToGScale;
// mag is considered stale if no new sample within this window.
static constexpr unsigned long kMagFreshMillis = 2000;

void compassCalLoop(const CompassCalStatus &cal);

static inline float wrapRad(float a) {
  while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

// Rotation of the panel about its own +z axis, as seen in the world frame, CCW positive looking at the LED face. Measured
// about whichever world vertical (+z or -z) the panel's +z is nearer, so it is independent of which way the backend's
// world frame points (ICM DMP world comes out z-down after localization; the v7 Madgwick world is z-up) and stays
// consistent with the eCompass bearing when the panel is face down. Its zero is arbitrary; only changes matter.
static inline float panelYawFromQuat(const Quaternion &q) {
  // world direction of the panel's +x is the first column of R(q); +z's z-component is R[2][2]
  float up = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
  float s = (up >= 0) ? 1.0f : -1.0f;
  return atan2f(s * 2.0f * (q.x * q.y + q.w * q.z), 1.0f - 2.0f * (q.y * q.y + q.z * q.z));
}

static inline bool bNormPlausible(float b) { return b >= kBNormMinUT && b <= kBNormMaxUT; }
// Does a stationary |B| match the field the calibrator fitted? With no accepted fit yet (stored offset, no radius) there is
// nothing to compare against and the caller's fallback applies.
static inline bool haveRadius(const CompassCalStatus &cal) { return cal.acceptedRadius > 0; }
static inline bool matchesRadius(float b, const CompassCalStatus &cal) {
  return haveRadius(cal) && fabsf(b - cal.acceptedRadius) <= kRadiusMatchUT;
}

// Drop the heading and start acquiring a baseline again.
static inline void dropHeading(unsigned long needMillis, const char *why) {
  if (_offsetSeeded) logf("compass: heading dropped (%s); dots until the field is steady", why);
  _offsetSeeded = false;
  _reseedPending = false;
  _anchorStart = 0;
  _acquireStart = 0;
  _acquireNeedMillis = needMillis;
  _disturbedSince = _cleanSince = 0;
}

inline void update(const MotionFrame &frame) {
  compassCalLoop(frame.compassCal); // core0 housekeeping for the core1 calibrator: persist, event log. Every loop.

  unsigned long now = millis();
  if (frame.compassCal.biasGeneration != _lastBiasGeneration) {
    _lastBiasGeneration = frame.compassCal.biasGeneration;
    // The offset moved on the strength of a fit that passed the quality gates, so the field is vouched for. A small move
    // keeps the arrow up (held on yaw) and re-anchors on the next fix; a large one means the old heading is garbage.
    float moved = 0;
    for (int i = 0; i < 3; ++i) moved = max(moved, fabsf(frame.compassCal.bias[i] - _lastBias[i]));
    if (!_offsetSeeded || moved > kReseedDropUT) {
      dropHeading(0, "offset corrected");
    } else {
      _reseedPending = true;
      logf("compass: offset moved %.1fuT; heading re-anchors on the next fix", moved);
    }
  }
  for (int i = 0; i < 3; ++i) _lastBias[i] = frame.compassCal.bias[i];
  if (frame.compassCal.windows != _lastWindows) {
    _lastWindows = frame.compassCal.windows;
    if (frame.compassCal.rejected != _lastRejected) {
      _lastRejected = frame.compassCal.rejected;
      // The device was just handled in a field that does not fit a sphere. If the baseline does not match the calibrated
      // field either, it is probably that field's: drop it. A vouched baseline stands; the per-sample hold covers the field.
      if (_offsetSeeded && !_baselineVouched) dropHeading(kReacquireMillis, "calibrator rejected a window against an unvouched baseline");
    }
  }
  _magCalibrated = frame.magCalibrated;
  if (!_magCalibrated) dropHeading(0, "offset discarded");
  if (frame.hasOrientation) {
    _panelYaw = panelYawFromQuat(frame.quat);
  }
  if (_offsetSeeded && now - _lastFixMillis > kHoldMillis) {
    _holdExpiries++;
    dropHeading(kReacquireMillis, "no fix for too long, yaw drift unbounded");
  }

  // Only fold in the eCompass on a fresh mag sample.
  if (frame.magCount == _lastMagCount) return;
  _lastMagCount = frame.magCount;
  float dt = (_lastMagMillis == 0) ? 0 : (now - _lastMagMillis) * 1e-3f;
  _lastMagMillis = now;
  if (!_magCalibrated) return;

  // Require quasi-stationary (accel ~= gravity) for a trustworthy eCompass bearing.
  int gx = frame.gyr.x, gy = frame.gyr.y, gz = frame.gyr.z;
  if (abs(gx) > kStationaryGyroLSB || abs(gy) > kStationaryGyroLSB || abs(gz) > kStationaryGyroLSB) return;

  float ax = frame.acc.x, ay = frame.acc.y, az = frame.acc.z;
  float alen = sqrtf(ax * ax + ay * ay + az * az);
  if (alen < kAccelMinLSB || alen > kAccelMaxLSB) return;

  // Tilt-compensated north bearing in the panel plane. "Up" comes from the fused orientation when there is one: the
  // quaternion's gravity direction is filtered, so hand jitter that passes the stationary gates barely moves it, where the
  // raw accelerometer tilted it several degrees (with a weak horizontal field, 10-15deg of heading scatter). The
  // accelerometer resolves which way the backend's world z points (DMP world is z-down after localization, Madgwick z-up)
  // and stays the stationary gate above.
  float ux = ax / alen, uy = ay / alen, uz = az / alen; // up (motion frame), from the accelerometer
  if (frame.hasOrientation) {
    const Quaternion &q = frame.quat;
    // third row of R(q): world z expressed in the body frame
    float gx = 2.0f * (q.x * q.z - q.w * q.y), gy = 2.0f * (q.y * q.z + q.w * q.x), gz = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    float glen = sqrtf(gx * gx + gy * gy + gz * gz);
    if (glen > 0.5f) {
      float s = (gx * ux + gy * uy + gz * uz) >= 0 ? 1.0f / glen : -1.0f / glen; // agree with the accelerometer's up
      ux = gx * s; uy = gy * s; uz = gz * s;
    }
  }
  float mx = frame.mag.x / MotionManager::magToUTScale, my = frame.mag.y / MotionManager::magToUTScale, mz = frame.mag.z / MotionManager::magToUTScale;
  float blen = sqrtf(mx * mx + my * my + mz * mz);
  if (blen < 1.0f) return;
  // east = mag x up ; north = up x east
  float ex = my * uz - mz * uy, ey = mz * ux - mx * uz, ez = mx * uy - my * ux;
  float nx = uy * ez - uz * ey, ny = uz * ex - ux * ez;
  float northInst = atan2f(ny, nx); // bearing of magnetic north in the panel plane
  float dip = asinf(constrain(-(mx * ux + my * uy + mz * uz) / blen, -1.0f, 1.0f)); // +ve = field points down (northern hemisphere)
  float target = wrapRad(northInst + _panelYaw);
  _lastNorthInst = northInst;
  _lastTarget = target;
  _lastBNorm = blen;
  _lastDip = dip;

  if (!_offsetSeeded) {
    // Acquiring: consecutive plausible samples must agree with each other for _acquireNeedMillis.
    bool agrees = _acquireStart != 0 && fabsf(blen - _acqBNorm) <= kDisturbBNormUT && fabsf(dip - _acqDip) <= kDisturbDipRad;
    if (!bNormPlausible(blen)) { _acquireStart = 0; return; }
    if (!agrees) { _acquireStart = now; _acqBNorm = blen; _acqDip = dip; }
    if (now - _acquireStart < _acquireNeedMillis) return;
    _offsetSin = sinf(target);
    _offsetCos = cosf(target);
    _refBNorm = blen;
    _refDip = dip;
    _offsetSeeded = true;
    // Vouched if this field is the one the offset was fitted in; with no fitted radius yet, trust an immediate seed.
    _baselineVouched = haveRadius(frame.compassCal) ? matchesRadius(blen, frame.compassCal) : (_acquireNeedMillis == 0);
    _lastFixMillis = now;
    _disturbedSince = _cleanSince = 0;
    _acceptedCount++;
    logf("compass: heading available, |B| %.1fuT dip %.0fdeg%s", blen, dip * 180.0f / (float)M_PI,
         _baselineVouched ? "" : " (unvouched: |B| does not match the calibrated field; re-anchors on one that does)");
    return;
  }

  // Re-anchor: after a small offset correction (the fit vouched for the field, so a plausible steady field is the new
  // truth), or when an unvouched baseline meets the field the offset was actually fitted in. Either way the candidate field
  // must hold steady for kAnchorDwellMillis first; the arrow stays up (held) meanwhile.
  bool matches = matchesRadius(blen, frame.compassCal);
  if ((_reseedPending && bNormPlausible(blen)) || (!_baselineVouched && matches)) {
    bool agrees = _anchorStart != 0 && fabsf(blen - _anchorBNorm) <= kDisturbBNormUT / 2 && fabsf(dip - _anchorDip) <= kDisturbDipRad / 2;
    if (!agrees) { _anchorStart = now; _anchorBNorm = blen; _anchorDip = dip; }
    if (now - _anchorStart < kAnchorDwellMillis) return;
    if (!_baselineVouched && matches) logf("compass: re-anchored to the calibrated field (|B| %.1f vs baseline %.1fuT, dip %.0fdeg)", blen, _refBNorm, dip * 180.0f / (float)M_PI);
    _anchorStart = 0;
    _reseedPending = false;
    _baselineVouched = haveRadius(frame.compassCal) ? matches : true;
    _offsetSin = sinf(target);
    _offsetCos = cosf(target);
    _refBNorm = blen;
    _refDip = dip;
    _lastFixMillis = now;
    _disturbedSince = _cleanSince = 0;
    _acceptedCount++;
    return;
  }
  _anchorStart = 0;

  // Tracking: does this sample look like the field we calibrated in? If not, hold the offset and ride on yaw.
  // A vouched baseline is judged against the fitted radius so a disagreeing field can't erode it; an unvouched one against
  // itself, since we have admitted we don't know the true field here and a steady (if biased) arrow beats a hold that
  // drifts on yaw and then blinks to dots every kHoldMillis. The radius still vouches and re-anchors it above.
  float bRef = (_baselineVouched && haveRadius(frame.compassCal)) ? frame.compassCal.acceptedRadius : _refBNorm;
  bool disturbed = !bNormPlausible(blen) || fabsf(blen - bRef) > kDisturbBNormUT || fabsf(dip - _refDip) > kDisturbDipRad;
  if (disturbed) {
    if (_disturbedSince == 0) {
      _disturbedSince = now;
      _disturbances++;
      logf("compass: field disturbed (|B| %.1f vs %.1fuT, dip %.0f vs %.0fdeg); holding heading on yaw",
           blen, bRef, dip * 180.0f / (float)M_PI, _refDip * 180.0f / (float)M_PI);
    }
    _cleanSince = 0;
    return;
  }
  if (_disturbedSince != 0) {
    if (_cleanSince == 0) _cleanSince = now;
    if (now - _cleanSince < kRecoverMillis) return;
    logf("compass: field clean again after %lus; taking fixes", (now - _disturbedSince) / 1000UL);
    _disturbedSince = _cleanSince = 0;
  }

  float alpha = min(kOffsetMaxAlpha, dt / kOffsetTauSeconds);
  _offsetSin += alpha * (sinf(target) - _offsetSin);
  _offsetCos += alpha * (cosf(target) - _offsetCos);
  float ralpha = min(kOffsetMaxAlpha, dt / kRefTauSeconds);
  _refBNorm += ralpha * (blen - _refBNorm);
  _refDip += ralpha * (dip - _refDip);
  _lastFixMillis = now;
  _acceptedCount++;
}

// Direction of magnetic north in the panel plane, radians, motion frame: 0 = +x_motion, CCW positive about +z_motion.
inline float northAngleRad() {
  if (!_offsetSeeded) return 0;
  float offset = atan2f(_offsetSin, _offsetCos);
  return wrapRad(offset - _panelYaw);
}

// Heading is trustworthy: hard iron calibrated, a baseline acquired and a fix taken within kHoldMillis, mag data fresh.
inline bool headingValid() {
  return _magCalibrated && _offsetSeeded && (millis() - _lastMagMillis) < kMagFreshMillis;
}

// The heading is being held on yaw because the field disagrees with the baseline (the arrow is up but not being corrected).
inline bool headingHeld() { return _offsetSeeded && _disturbedSince != 0; }

// Serial diagnostics, core0. `target` should hold still while the device is rotated slowly if the mag and accel axis maps
// agree with each other and with the quaternion; `dip` and |B| should hold still across any orientation once calibrated.
// `hold` is seconds the field has disagreed with the baseline (0 = taking fixes), `fix` seconds since the last fix.
inline void logDiagnostics(const MotionFrame &frame) {
  const float deg = 180.0f / (float)M_PI;
  unsigned long now = millis();
  logf("COMPASS valid=%i cal=%i mag=[%.1f %.1f %.1f]uT |B|=%.1f dip=%.0f ref=[%.1f %.0f]%s hold=%lu fix=%lu dist=%u exp=%u T=%.1f acc=[%i %i %i] gyr=[%i %i %i] yaw=%.0f northInst=%.0f target=%.0f north=%.0f n=%lu accepted=%lu calstate=%u magretry=%u",
       headingValid(), _magCalibrated,
       frame.mag.x / MotionManager::magToUTScale, frame.mag.y / MotionManager::magToUTScale, frame.mag.z / MotionManager::magToUTScale,
       _lastBNorm, _lastDip * deg, _refBNorm, _refDip * deg, _baselineVouched ? "" : "?",
       _disturbedSince ? (now - _disturbedSince) / 1000UL : 0UL, _lastFixMillis ? (now - _lastFixMillis) / 1000UL : 0UL,
       _disturbances, _holdExpiries, frame.tempC,
       frame.acc.x, frame.acc.y, frame.acc.z, frame.gyr.x, frame.gyr.y, frame.gyr.z,
       _panelYaw * deg, _lastNorthInst * deg, _lastTarget * deg, northAngleRad() * deg,
       (unsigned long)frame.magCount, (unsigned long)_acceptedCount, (unsigned)frame.compassCal.state, (unsigned)frame.magInitRetries);
}

// Core0 side of the always-on hard-iron calibration (compasscal.h, core1), called from update() every loop. Persists the
// offset in effect whenever the calibrator reports a new trusted one (EEPROM.commit parks core1 during the erase, so flash
// writes must be core0) and logs the calibrator's rare events. Nothing is drawn from here: CompassPattern reflects the
// heading state when it is the running pattern; the calibrator runs regardless.
void compassCalLoop(const CompassCalStatus &cal) {
  static constexpr float kPersistMinDeltaUT = 1.0f; // don't rewrite flash for an offset that only moved by noise
  static constexpr unsigned long kPersistMinIntervalMS = 30000; // and not more often than this; the latest wins
  static bool lastCalibrated = false;
  static uint8_t lastCorrections = 0, lastRejected = 0;
  static uint32_t persistedGeneration = 0; // only advances when a persist happens, so a change inside the interval retries
  static unsigned long lastLog = 0, lastPersist = 0;

  if (cal.corrections != lastCorrections) {
    lastCorrections = cal.corrections;
    logf("compass cal: correction #%u applied: center %.1fuT, |B| %.1fuT, fit rms %.2fuT, bias now [%.1f %.1f %.1f]uT",
         cal.corrections, cal.lastResidual, cal.lastRadius, cal.lastRMS, cal.bias[0], cal.bias[1], cal.bias[2]);
  }
  if (cal.rejected != lastRejected) {
    lastRejected = cal.rejected;
    logf("compass cal: window #%lu rejected (center %.1fuT, |B| %.1fuT vs accepted %.1f, fit rms %.2fuT, spread %.1fuT): stray field? offset left alone",
         (unsigned long)cal.windows, cal.lastResidual, cal.lastRadius, cal.acceptedRadius, cal.lastRMS, cal.lastSpread);
  }
  if (cal.calibrated != lastCalibrated) {
    lastCalibrated = cal.calibrated;
    if (cal.calibrated) {
      logf("compass cal: offset trusted, bias [%.1f %.1f %.1f]uT (last fit center %.1fuT, gen %lu)",
           cal.bias[0], cal.bias[1], cal.bias[2], cal.lastResidual, (unsigned long)cal.biasGeneration);
    } else {
      logf("compass: calibration needed; tumble the device through every orientation");
    }
  }

  if (cal.calibrated && cal.biasGeneration != persistedGeneration && millis() - lastPersist >= kPersistMinIntervalMS) {
    persistedGeneration = cal.biasGeneration;
    lastPersist = millis();
    CompassBiasRecord prev;
    bool haveStored = compassStoreRead(&prev);
    float delta = 0;
    if (haveStored) {
      for (int i = 0; i < 3; ++i) delta = max(delta, fabsf(cal.bias[i] - prev.biasCentiUT[i] / 100.0f));
    }
    if (haveStored && delta < kPersistMinDeltaUT) {
      logf("compass cal: bias within %.2fuT of stored, not persisting", delta);
    } else {
      CompassBiasRecord rec = {0};
      for (int i = 0; i < 3; ++i) rec.biasCentiUT[i] = (int32_t)lroundf(cal.bias[i] * 100.0f);
      rec.calCount = haveStored ? prev.calCount + 1 : 1;
      rec.magSpreadCentiUT = (int32_t)(cal.lastSpread * 100.0f);
      rec.fitRadiusCentiUT = (int32_t)lroundf(cal.acceptedRadius * 100.0f);
      bool ok = compassStoreWrite(rec);
      logf("compass cal persist bias=[%.2f %.2f %.2f]uT R=%.1fuT |B|spread=%.1fuT calCount=%lu: %s",
           cal.bias[0], cal.bias[1], cal.bias[2], cal.acceptedRadius, cal.lastSpread, (unsigned long)rec.calCount, ok ? "ok" : "FAILED");
    }
  }

  if (compassLogging && millis() - lastLog >= 1000) {
    lastLog = millis();
    logf("CCAL cal=%i n=%lu range=[%.0f %.0f %.0f] |B|=[%.1f %.1f] last: center=%.1f R=%.1f rms=%.2f accR=%.1f bias=[%.1f %.1f %.1f] corr=%u rej=%u win=%lu",
         cal.calibrated, (unsigned long)cal.sampleCount,
         cal.axisMax[0] - cal.axisMin[0], cal.axisMax[1] - cal.axisMin[1], cal.axisMax[2] - cal.axisMin[2],
         cal.magNormMin, cal.magNormMax, cal.lastResidual, cal.lastRadius, cal.lastRMS, cal.acceptedRadius,
         cal.bias[0], cal.bias[1], cal.bias[2],
         cal.corrections, cal.rejected, (unsigned long)cal.windows);
  }
}

} // namespace Compass

#endif // COMPASS_H
