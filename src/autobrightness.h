#ifndef AUTOBRIGHTNESS_H
#define AUTOBRIGHTNESS_H

// Ambient-light brightness from the phototransistors, plus a thermal ceiling.
//
// The sensors sit right next to pixels behind the same diffuser, so with the panel running they
// mostly see the panel. There is no per-unit calibration of that coupling and there will not be one
// (doc/autobrightness-challenge-review.md: it differs up to ~3x between units, structured by die and
// placement geometry, so a shared table cannot subtract it either). So this never subtracts. Instead:
//
//   - At boot the pixels are off (the power-on animation lights only pixels far from the sensors),
//     so every sensor reads pure ambient. That baseline sets the brightness for the session.
//   - While running, a sensor updates its estimate only on "clean" frames: the panel's predicted
//     light at it, from a coarse per-revision model of which pixels sit near it, times a margin for
//     how far any unit may differ from that model, is small next to the ambient level being
//     reported. The model is a gate, never a subtrahend, so its accuracy is never load-bearing.
//     How often that happens depends entirely on the pattern, so tracking is best-effort and slow
//     by design -- transient changes in the room should not move the panel.
//   - Ambient is the max across sensors: a sensor the panel is swamping (or a finger is covering)
//     holds its last clean reading and can only ever under-report, so the max is the tightest
//     claim available. A sensor blind for long enough drifts toward one that can still see.
//
// Everything is in raw ADC counts (12-bit, panel off, summed over 16 reads so the fixed-point unit is
// counts*16). Per-unit spread in sensor gain and dark floor (measured ~2-3x and 0-7 counts) shifts
// the ambient anchor by a few brightness steps on the log curve; that is the accepted cost of not
// calibrating. It cannot oscillate or self-brighten in the dark, because no subtraction happens.

#include <Arduino.h>
#include <FastLED.h>
#include "fl/five_bit_hd_gamma.h"
#include "pinout.h"

// ---- per hardware revision: sensor geometry and analog front end ----
//
// Pixels near each sensor, in rings of increasing distance, each with a coupling weight relative to
// the nearest pixel (out of 256). Measured on one v7 unit and one v6 unit with the front case on,
// the two adjacent pixels are ~80% of a sensor's total coupling and these four rings ~98%; the
// weights are rounded up to the strongest pixel in each ring, since over-predicting only makes the
// gate stricter. Other units differ by up to ~3x per pixel, which is what gateMarginX is for.
struct NearPixel {
  uint16_t pixel;
  uint8_t weight; // coupling relative to the nearest pixel, /256
};
#define AB_RING0 255
#define AB_RING1 32
#define AB_RING2 8
#define AB_RING3 3

#if HARDWARE_VERSION >= 7
#define AB_SENSOR_TAU_MS 15    // the phototransistor's own lag, measured with PB STEP
#define AB_COUNTS_PER_DRIVE 3  // ADC counts per unit of FastLED drive (pwm*current) at the nearest pixel
#define AB_DARK_COUNTS 10      // at or below this the room is dark (dark floor 4-7 counts across units)
#define AB_BRIGHT_COUNTS 158   // the level mapped to maxBrightness
#define AB_CURVE_OFFSET_COUNTS 8 // the log curve runs on counts above this; see curveOffsetCounts
// Sensor 0 / GPIO26 / Q3: logical top-left corner, next to px 0 and 10.
static const NearPixel kNearPixels0[] = {
  {10, AB_RING0}, {0, AB_RING0},
  {11, AB_RING1}, {21, AB_RING1}, {22, AB_RING1}, {1, AB_RING1},
  {23, AB_RING2}, {34, AB_RING2}, {12, AB_RING2}, {33, AB_RING2}, {35, AB_RING2}, {2, AB_RING2},
  {24, AB_RING3}, {47, AB_RING3}, {36, AB_RING3}, {48, AB_RING3}, {13, AB_RING3}, {46, AB_RING3}, {49, AB_RING3}, {3, AB_RING3},
};
// Sensor 1 / GPIO27 / Q2: logical bottom-left corner, next to px 250 and 261.
static const NearPixel kNearPixels1[] = {
  {250, AB_RING0}, {261, AB_RING0},
  {251, AB_RING1}, {238, AB_RING1}, {239, AB_RING1}, {262, AB_RING1},
  {240, AB_RING2}, {252, AB_RING2}, {226, AB_RING2}, {225, AB_RING2}, {227, AB_RING2}, {263, AB_RING2},
  {241, AB_RING3}, {212, AB_RING3}, {228, AB_RING3}, {213, AB_RING3}, {253, AB_RING3}, {211, AB_RING3}, {214, AB_RING3}, {264, AB_RING3},
};
// Sensor 2 / GPIO28 / Q1: logical right corner, next to px 144 (end of the center row) and 125.
static const NearPixel kNearPixels2[] = {
  {144, AB_RING0}, {125, AB_RING0},
  {143, AB_RING1}, {162, AB_RING1}, {107, AB_RING1}, {124, AB_RING1},
  {161, AB_RING2}, {142, AB_RING2}, {106, AB_RING2}, {179, AB_RING2}, {90, AB_RING2}, {123, AB_RING2},
  {160, AB_RING3}, {178, AB_RING3}, {141, AB_RING3}, {89, AB_RING3}, {105, AB_RING3}, {195, AB_RING3}, {74, AB_RING3}, {122, AB_RING3},
};
static const NearPixel *const kNearPixels[] = {kNearPixels0, kNearPixels1, kNearPixels2};
static const uint8_t kNearPixelCount[] = {
  sizeof(kNearPixels0) / sizeof(NearPixel), sizeof(kNearPixels1) / sizeof(NearPixel), sizeof(kNearPixels2) / sizeof(NearPixel)
};
#else
// v6's front end is more sensitive (~1.5x the counts per unit drive) and much faster than v7's.
#define AB_SENSOR_TAU_MS 3
#define AB_COUNTS_PER_DRIVE 4
#define AB_DARK_COUNTS 16      // dark floor measured 13.2 counts; v7 anchors scaled by the front-end sensitivity, unverified
#define AB_BRIGHT_COUNTS 238
#define AB_CURVE_OFFSET_COUNTS 13
// single phototransistor near px 9/10 (logical top-right corner)
static const NearPixel kNearPixels0[] = {
  {10, AB_RING0}, {9, AB_RING0},
  {32, AB_RING1}, {8, AB_RING1}, {33, AB_RING1}, {11, AB_RING1}, {31, AB_RING1}, {34, AB_RING1},
  {7, AB_RING2}, {12, AB_RING2}, {30, AB_RING2}, {35, AB_RING2}, {57, AB_RING2}, {58, AB_RING2}, {59, AB_RING2},
  {60, AB_RING3}, {61, AB_RING3}, {62, AB_RING3}, {63, AB_RING3}, {64, AB_RING3}, {56, AB_RING3}, {36, AB_RING3}, {29, AB_RING3}, {13, AB_RING3}, {6, AB_RING3},
};
static const NearPixel *const kNearPixels[] = {kNearPixels0};
static const uint8_t kNearPixelCount[] = {sizeof(kNearPixels0) / sizeof(NearPixel)};
#endif

class HexaAutoBrightness {
public:
  static const int kMaxSensors = sizeof(kNearPixels) / sizeof(kNearPixels[0]);

  // ---- brightness envelope ----
  uint8_t minBrightness = 2;     // floor for a properly dark room
  uint8_t maxBrightness = 30;    // ceiling in bright ambient; deliberately far below what the panel can do
  uint8_t sustainBrightness = 15; // the level the panel can hold indefinitely; the rolloff plateaus
                                  // here rather than ramping straight past it. Measured: brightness 15
                                  // in open air settles around 50C gauge / 55C die, which prior
                                  // full-case revisions ran continuously without trouble.
  uint8_t thermalFloor = 5;      // brightness the thermal rolloff bottoms out at
  uint8_t criticalBrightness = 1; // and past the critical temperature, below even that

  // ---- ambient -> brightness curve ----
  // Ambient is in ADC counts with the panel off. At or below darkCounts the room counts as dark and
  // we sit at minBrightness; at brightCounts we reach maxBrightness. In between it is logarithmic
  // in the counts above curveOffsetCounts. A plain log of the raw count is too flat at the dark
  // end: the sensors' dark floor eats most of the few counts that separate rooms the eye sees as
  // very different. Measured on one v7 in an office (floor ~4 counts): lights off 10.3, lit but
  // dim 14.5, desk lamp 38.8; the plain log put the dim room at 6 of 30, one step above dark.
  // With the offset those land at 2-3 / 10 / 19, which is what those rooms want. All three are
  // population anchors: any given unit's sensors read up to ~2-3x off them in gain and a few
  // counts off in floor, accepted as a few steps of per-unit bias.
  uint16_t darkCounts = AB_DARK_COUNTS;
  uint16_t brightCounts = AB_BRIGHT_COUNTS;
  uint16_t curveOffsetCounts = AB_CURVE_OFFSET_COUNTS;
  // Deadband around each step, as a percentage of one step's width on the curve. The output is
  // quantised to whole brightness levels, so without this the estimate sitting near a boundary
  // would toggle across it. (As a percentage of counts it was worth 1.5 steps near the dark end
  // and a fraction of one near the bright end.)
  uint8_t hysteresisStepPct = 50;

  // ---- thermal limiting ----
  // Scale brightness down to manage heat. We'll use the fuel gauge sensor and the RP2350's own die sensor,
  // which is far faster and finer-grained (the gauge only moves in whole kelvin and lags by tens of
  // seconds). Whichever is more alarmed wins.
  // One set of breakpoints, in gauge degrees C. The die sensor is measured to run dieOffsetC hotter
  // than the gauge under load, so its reading is referred back to the gauge scale.
  int16_t warmC = 40;            // burst is over, decay toward the sustainable level
  int16_t sustainC = 50;         // at the sustainable level, and hold it here
  int16_t hotC = 56;             // hotter than we ever meant to run; start giving up more
  int16_t criticalC = 60;        // last resort
  int16_t dieOffsetC = 5;        // how much hotter the die reads than the gauge under load
  int16_t thermalHysteresisC = 2;
  unsigned dieSampleMS = 1000;  // reading it costs a 1ms stall on core0, so keep it infrequent

  // ---- dynamics ----
  // The boot baseline sets the starting brightness outright: the first ambient reading snaps the
  // output to its level on the curve (under the thermal ceiling) instead of ramping up from
  // minBrightness, so a short session in a lit room is not spent mostly dim.
  // After that, deliberately slow: the boot baseline is the anchor and live updates only nudge it. A brightness
  // change in the room has to persist for several seconds (rise) to half a minute (fall) before the
  // panel follows, and updates only land on clean frames anyway, which stretches these further.
  unsigned updateIntervalMS = 10;
  unsigned riseTauMS = 3000;        // ambient filter, getting brighter
  unsigned fallTauMS = 6000;       // ambient filter, getting darker
  unsigned debounceMS = 700;        // a change of direction must hold this long before we act on it
  unsigned riseSlewMS = 250;        // then one step at a time
  unsigned fallSlewMS = 500;

  // A sensor's frame is clean when the panel's predicted light at it, times this margin, is still
  // small next to the ambient level it is reporting. The margin covers how far a unit's real coupling
  // was measured to differ from the shared geometry model (up to ~3x).
  uint8_t gateMarginX = 4;

  // The phototransistor lags the panel, so the prediction is filtered to match before it is used;
  // without this a fast pattern's decay tail reads as panel light on frames that look clean.
  unsigned sensorTauMS = AB_SENSOR_TAU_MS;

  // A sensor that has not had a clean frame for this long is quoting a room it can no longer see,
  // so its held reading drifts toward whichever sensor still can -- and only while one can: with
  // every sensor swamped there is no evidence in either direction, and a bright pattern routinely
  // blinds all of them for half a minute at a stretch.
  unsigned staleAfterMS = 10000;
  unsigned staleTauMS = 60000;

  // A sensor that never responds to the panel is broken or covered for good, not in a dark room:
  // its nearest pixels at a predicted blindPanelCounts would lift any working sensor well clear of
  // the dark floor (measured: hundreds of counts). If every sensor is blind there is no ambient
  // information at all, and the panel runs at fallbackBrightness instead of sitting in the dark.
  uint16_t blindPanelCounts = 100;
  unsigned blindAfterMS = 2000;
  uint8_t fallbackBrightness = 15;

  bool logChanges = false;
  bool thermalEnabled = true;   // bench hook: run the control law without the thermal ceiling
  // bench hook: pin the output brightness so the estimator can be characterized at a chosen level.
  // -1 releases it.
  int16_t pinnedBrightness = -1;
  // bench hook: feed the controller a synthetic ambient level (counts*16) instead of the sensors,
  // so the curve, hysteresis and debounce can be exercised without a light source.
  int32_t injectedAmbient16 = -1;

private:
  const int *readPins;
  int sensorCount;
  int powerPin;

  static const int kReadShift = 4;                          // 16 ADC samples per sensor per update
  static const int kReadSamples = 1 << kReadShift;

  unsigned long lastUpdate = 0;
  unsigned long lastSlew = 0;
  unsigned long directionSince = 0;
  int8_t pendingDirection = 0;
  int8_t moving = 0;
  uint8_t current = 0;
  bool thermalLimiting = false;

  uint32_t ambientFilt16 = 0;
  bool haveAmbient = false;
  bool snapped = false;   // output has been set from the boot baseline

  // Filter states carry kFiltShift fractional bits on top of counts*16. Without them the
  // truncating step in iir() either strands the filter short of its target or, with the minimum
  // step that fixes that, moves 1/16 count every update regardless of tau -- 6 counts/s, which
  // made a 60 s stale drift converge in 3 s (measured).
  static const int kFiltShift = 8;

  // Each sensor keeps its own estimate, updated only on its clean frames. The sensors do not agree
  // on what the same room reads (measured: 2.3x apart on one unit), so handing the answer between
  // them made the estimate lurch; each holding its own and taking the max does not.
  uint32_t sensorFilt[kMaxSensors] = {0};
  bool haveSensor[kMaxSensors] = {false};
  unsigned long sensorSeen[kMaxSensors] = {0};
  unsigned blindMS[kMaxSensors] = {0};

  // ambient level at which each brightness step is reached, built once by configure()
  static const int kMaxLevels = 64;
  uint32_t stepAt[kMaxLevels] = {0};
  int levelCount = 0;
  uint32_t hystMul1024 = 1024;  // one deadband's worth of ambient ratio, x1024

  // diagnostics
  uint32_t panel16[kMaxSensors] = {0};
  uint32_t panelFilt[kMaxSensors] = {0};
  uint32_t raw16[kMaxSensors] = {0};
  bool clean[kMaxSensors] = {false};
  int32_t lastAmbient16 = -1;
  uint8_t thermalCap = 255;
  float dieC = 0;
  unsigned long lastDieSample = 0;

  uint32_t readSensorSum(int s) {
    uint32_t sum = 0;
    for (int i = 0; i < kReadSamples; ++i) {
      sum += analogRead(readPins[s]);
    }
    return sum;
  }

  // Exponential filter with a time constant, stepped by dt, on kFiltShift fixed-point state.
  //
  // state += (input - state) * dt / tau, but that division truncates, so the step reaches zero while
  // the gap is still tau/dt wide and the filter would stop short of its target for good. So always
  // move at least one unit toward the target (with the fractional bits that is a negligible
  // 1/4096 count per update) and never step past it.
  static uint32_t iir(uint32_t state, uint32_t input, unsigned dtMS, unsigned tauMS) {
    if (dtMS >= tauMS) return input;
    int64_t diff = (int64_t)input - (int64_t)state;
    if (diff == 0) return state;
    int64_t step = diff * (int)dtMS / (int)tauMS;
    if (step == 0) step = (diff > 0 ? 1 : -1);
    int64_t next = (int64_t)state + step;
    if (diff > 0 ? next > (int64_t)input : next < (int64_t)input) next = (int64_t)input;
    return (uint32_t)max((int64_t)0, next);
  }

public:
  HexaAutoBrightness(const int *readPins, int sensorCount, int powerPin)
    : readPins(readPins), sensorCount(min(sensorCount, kMaxSensors)), powerPin(powerPin) { }

  void setup() {
    analogReadResolution(12);
    for (int s = 0; s < sensorCount; ++s) {
      pinMode(readPins[s], INPUT);
    }
    if (powerPin != -1) {
      pinMode(powerPin, OUTPUT);
      digitalWrite(powerPin, true); // readings are inconsistent if this is cycled, so leave it up
    }
    current = minBrightness;
    configure();
  }

  // ---- diagnostics, for the bench (all counts*16) ----
  int sensors() const { return sensorCount; }
  uint32_t sensorRaw16(int s) const { return raw16[s]; }
  uint32_t sensorPanel16(int s) const { return panelFilt[s] >> kFiltShift; }
  uint32_t sensorEstimate16(int s) const { return sensorFilt[s] >> kFiltShift; }
  bool sensorClean(int s) const { return clean[s]; }
  bool sensorBlind(int s) const { return blindMS[s] >= blindAfterMS; }
  int32_t ambient16() const { return lastAmbient16; }
  uint32_t ambientFiltered16() const { return ambientFilt16; }
  uint8_t brightness() const { return current; }
  uint8_t thermalCeiling() const { return thermalCap; }

  // One line of everything the estimator knows, in ADC counts: each sensor's raw reading, what the
  // panel is predicted to contribute to it, its held estimate and whether it was trusted this frame.
  void logDiagnostics(uint16_t tempK) const {
    char line[256];
    int n = snprintf(line, sizeof(line), "AB ambient=%.1f sensors", lastAmbient16 / 16.0);
    for (int s = 0; s < sensorCount && n < (int)sizeof(line); ++s) {
      n += snprintf(line + n, sizeof(line) - n, " [raw %.1f panel %.1f est %.1f%s%s]", raw16[s] / 16.0,
                    sensorPanel16(s) / 16.0, sensorEstimate16(s) / 16.0, clean[s] ? " clean" : "",
                    sensorBlind(s) ? " BLIND" : "");
    }
    if (n < (int)sizeof(line)) {
      snprintf(line + n, sizeof(line) - n, " brightness=%i cap=%i curve=%i gauge=%uK die=%.1fC", current, thermalCap,
               haveAmbient ? levelFor(ambientFilt16) : -1, tempK, dieC);
    }
    logf("%s", line);
  }

  // How much of each sensor's reading the panel is probably responsible for right now, in counts*16:
  // each nearby pixel's drive (FastLED's own APA102-HD split into an 8-bit PWM value and a 5-bit
  // current level, multiplied together, so gamma and global brightness are handled the same way the
  // pixels are) weighted by its ring and scaled by the revision's counts-per-drive. Blue emits ~1.2x
  // green per unit drive and red less; red is left at 1 since over-predicting is the safe direction.
  void computePanel(const CRGB *leds, uint8_t appliedBrightness) {
    for (int s = 0; s < sensorCount; ++s) {
      uint32_t acc = 0;
      for (int i = 0; i < kNearPixelCount[s]; ++i) {
        CRGB c = leds[kNearPixels[s][i].pixel];
        if (!c || appliedBrightness == 0) continue;
        CRGB out;
        uint8_t pwr = 0;
        fl::five_bit_hd_gamma_bitshift(c, CRGB(255, 255, 255), appliedBrightness, &out, &pwr);
        uint32_t drive = ((uint32_t)out.r + out.g + out.b + (out.b >> 2)) * pwr;
        acc += drive * kNearPixels[s][i].weight;
      }
      panel16[s] = (acc >> 8) * (AB_COUNTS_PER_DRIVE << kReadShift);
    }
  }

  void filterPanel(unsigned dtMS) {
    for (int s = 0; s < sensorCount; ++s) {
      panelFilt[s] = iir(panelFilt[s], panel16[s] << kFiltShift, dtMS, sensorTauMS);
    }
  }

  // Reads the sensors and returns ambient in counts*16, or -1 to hold.
  //
  // A sensor only speaks on a clean frame: the panel's predicted light at it, with gateMarginX of
  // headroom for how wrong the geometry model can be on this unit, must be small next to the level
  // that sensor is reporting. On such a frame the reading is essentially ambient, so it is taken as
  // is. At boot the pixels are off, every frame is clean, and the first reading becomes the session
  // baseline.
  int32_t measureAmbient(unsigned dtMS, unsigned long now) {
    const uint32_t dark = (uint32_t)darkCounts << (kReadShift + kFiltShift);
    int32_t bestClean = -1;
    for (int s = 0; s < sensorCount; ++s) {
      raw16[s] = readSensorSum(s);
      const uint32_t raw = raw16[s] << kFiltShift;
      uint32_t level = haveSensor[s] ? sensorFilt[s] : 0;
      clean[s] = panelFilt[s] * gateMarginX <= max(level / 4, dark);

      // blind check: the panel should be lighting it up and it is still reading the floor
      if (raw >= dark + ((uint32_t)4 << (kReadShift + kFiltShift))) {
        blindMS[s] = 0;
      } else if (panelFilt[s] >= (uint32_t)blindPanelCounts << (kReadShift + kFiltShift)) {
        blindMS[s] = min(blindMS[s] + dtMS, blindAfterMS);
      }

      if (clean[s] && !sensorBlind(s)) {
        if (!haveSensor[s]) {
          sensorFilt[s] = raw;   // boot baseline: first clean frame snaps
          haveSensor[s] = true;
        } else {
          sensorFilt[s] = iir(sensorFilt[s], raw, dtMS, raw > sensorFilt[s] ? riseTauMS : fallTauMS);
        }
        sensorSeen[s] = now;
        if ((int32_t)sensorFilt[s] > bestClean) bestClean = (int32_t)sensorFilt[s];
      }
    }

    // A sensor blind past staleAfterMS drifts toward whichever sensor still sees -- only while one
    // does. With every sensor swamped there is no evidence either way, and anything else here turns
    // "I cannot see" into "it is dark" and dims the panel to nothing mid-pattern (measured).
    if (bestClean >= 0) {
      for (int s = 0; s < sensorCount; ++s) {
        if (haveSensor[s] && !clean[s] && now - sensorSeen[s] > staleAfterMS) {
          sensorFilt[s] = iir(sensorFilt[s], (uint32_t)bestClean, dtMS, staleTauMS);
        }
      }
    }

    // The room is as bright as the best-placed sensor says: a swamped or covered sensor can only
    // under-report, so the maximum is the tightest claim available.
    int32_t best = -1;
    for (int s = 0; s < sensorCount; ++s) {
      if (haveSensor[s] && !sensorBlind(s) && (int32_t)sensorFilt[s] > best) best = (int32_t)sensorFilt[s];
    }
    lastAmbient16 = (best >= 0) ? best >> kFiltShift : -1;
    return lastAmbient16;
  }

  bool allBlind() const {
    for (int s = 0; s < sensorCount; ++s) {
      if (!sensorBlind(s)) return false;
    }
    return true;
  }

  // Ambient level (counts*16) at which brightness step b is reached. Built once so the hot path
  // stays integer (and so it can be walked in either direction for the hysteresis).
  void configure() {
    levelCount = min(kMaxLevels, (int)maxBrightness - (int)minBrightness + 1);
    if (levelCount < 1) levelCount = 1;
    const float offset = (float)min(curveOffsetCounts, (uint16_t)(darkCounts - 1)) * kReadSamples;
    const float dark = (float)darkCounts * kReadSamples - offset;
    const float bright = (float)max(brightCounts, (uint16_t)(darkCounts + 1)) * kReadSamples - offset;
    const float perStep = (levelCount > 1) ? powf(bright / dark, 1.0f / (levelCount - 1)) : 1.0f;
    for (int i = 0; i < levelCount; ++i) {
      stepAt[i] = (uint32_t)(offset + dark * powf(perStep, i) + 0.5f);
    }
    hystMul1024 = (uint32_t)(powf(perStep, hysteresisStepPct / 100.0f) * 1024 + 0.5f);
  }

  uint32_t stepThreshold(int b) const {
    int i = b - (int)minBrightness;
    if (i <= 0) return 0;
    if (i >= levelCount) return 0xFFFFFFFF;
    return stepAt[i];
  }

  // Brightness the curve puts an ambient level (counts*16) at, with no hysteresis.
  uint8_t levelFor(uint32_t ambient) const {
    int i = 0;
    while (i + 1 < levelCount && ambient >= stepAt[i + 1]) ++i;
    return minBrightness + i;
  }

  // Piecewise rolloff, in whatever temperature unit the breakpoints are expressed in.
  //
  //   <= warm            maxBrightness      the panel is cool; burst freely
  //   warm .. sustain    max -> sustain     burst is spent, decay to what we can hold
  //   sustain .. hot     sustainBrightness  plateau: normal warm running lives here
  //   hot .. crit        sustain -> floor   hotter than intended; give up more
  //   >= crit            thermalFloor
  //
  // The plateau is the point of this: a single ramp from maxBrightness to thermalFloor sails
  // straight through the level the hardware can actually sustain and parks well below it (measured:
  // it settled at 11 where 15 is fine). Holding the sustainable level across the whole normal warm
  // band means the panel only keeps dimming when something is genuinely wrong.
  uint8_t rolloff(int32_t temp, int32_t warm, int32_t sustain, int32_t hot, int32_t crit) {
    uint8_t plateau = min(sustainBrightness, maxBrightness);
    if (temp <= warm) return maxBrightness;
    if (temp < sustain) {
      if (sustain <= warm) return plateau;
      return (uint8_t)(maxBrightness - (int32_t)(maxBrightness - plateau) * (temp - warm) / (sustain - warm));
    }
    if (temp <= hot) return plateau;
    if (temp >= crit || crit <= hot) return thermalFloor;
    return (uint8_t)(plateau - (int32_t)(plateau - thermalFloor) * (temp - hot) / (crit - hot));
  }

  // Temperature ceiling. Backing brightness off can help prevent overheat.
  uint8_t thermalCeilingFor(uint16_t tempK, unsigned long now) {
    int hyst = thermalLimiting ? thermalHysteresisC : 0;
    uint8_t cap = maxBrightness;
    bool critical = false;
    if (tempK != 0) {  // 0 means the gauge hasn't been read yet
      int gaugeC = (int)tempK - 273;
      cap = rolloff(gaugeC, warmC - hyst, sustainC - hyst, hotC, criticalC);
      critical = (gaugeC >= criticalC);
    }
    if (!lastDieSample || now - lastDieSample >= dieSampleMS) {
      lastDieSample = now;
      dieC = analogReadTemp();
    }
    if (dieC > 0) {
      int dieAsGaugeC = (int)(dieC + 0.5f) - dieOffsetC;
      cap = min(cap, rolloff(dieAsGaugeC, warmC - hyst, sustainC - hyst, hotC, criticalC));
      critical = critical || (dieAsGaugeC >= criticalC);
    }
    // past this the rolloff has already bottomed out and it is still climbing, so give up the last
    // step too -- there is nothing else here that can shed heat.
    if (critical) cap = criticalBrightness;
    thermalLimiting = (cap < maxBrightness);
    return cap;
  }

  // Call every frame with the pixel buffer as it will be shown and the brightness FastLED is
  // actually applying to it. controlOutput=false keeps the estimate running without moving the
  // output, which is what the bench does while it is driving the panel itself.
  void loop(const CRGB *leds, uint8_t appliedBrightness, uint16_t tempK, bool controlOutput = true) {
    unsigned long now = millis();
    if (lastUpdate && now - lastUpdate < updateIntervalMS) return;
    unsigned dt = lastUpdate ? (unsigned)(now - lastUpdate) : updateIntervalMS;
    lastUpdate = now;

    computePanel(leds, appliedBrightness);
    filterPanel(dt);
    int32_t amb = measureAmbient(dt, now);
    if (injectedAmbient16 >= 0) amb = injectedAmbient16;

    // the per-sensor estimates already carry the rise/fall dynamics, so this is not filtered again
    if (amb >= 0) {
      ambientFilt16 = (uint32_t)amb;
      haveAmbient = true;
    }
    if (thermalEnabled) {
      thermalCap = thermalCeilingFor(tempK, now);
    } else {
      thermalCap = maxBrightness;
      thermalLimiting = false;
    }
    if (pinnedBrightness >= 0) {
      current = (uint8_t)pinnedBrightness;
      return;
    }
    if (!controlOutput) return;

    // Boot: the first ambient reading is the panel-off baseline; go straight to its level.
    if (!snapped && (haveAmbient || allBlind())) {
      snapped = true;
      uint8_t want = allBlind() ? fallbackBrightness : levelFor(ambientFilt16);
      want = min(want, thermalCap);
      if (logChanges) {
        logf("autobrightness: boot ambient=%.1f counts -> %i (thermal cap %i)", ambientFilt16 / 16.0, want, thermalCap);
      }
      current = want;
      return;
    }
    // Schmitt trigger around the current step: going up has to clear the next threshold by the
    // deadband, coming down has to fall below this one by it.
    uint8_t want = current;
    if (allBlind()) {
      want = fallbackBrightness;
    } else if (haveAmbient) {
      uint32_t up = stepThreshold(current + 1);
      uint32_t down = stepThreshold(current);
      if (current < maxBrightness && up != 0xFFFFFFFF &&
          ambientFilt16 >= (uint64_t)up * hystMul1024 / 1024) {
        want = current + 1;
      } else if (current > minBrightness && ambientFilt16 < (uint64_t)down * 1024 / hystMul1024) {
        want = current - 1;
      }
    } else {
      want = minBrightness;
    }
    if (want > thermalCap) want = thermalCap;
    if (want < minBrightness) want = min(minBrightness, thermalCap);

    // Debounce, on the *direction* rather than on every step: an estimate wobbling across a
    // threshold has to hold its new side for debounceMS before the panel moves at all, but once a
    // real change is under way the ramp isn't made to re-earn that wait at every level. Thermal
    // backoff skips the wait entirely.
    bool urgent = (want < current && thermalLimiting);
    int8_t dir = (want > current) ? 1 : (want < current ? -1 : 0);
    if (dir == 0) {
      moving = 0;
      pendingDirection = 0;
      return;
    }
    if (dir != pendingDirection) {
      pendingDirection = dir;
      directionSince = now;
    }
    if (moving != dir) {
      if (!urgent && now - directionSince < debounceMS) return;
      moving = dir;
    }
    unsigned slew = (dir > 0 ? riseSlewMS : fallSlewMS);
    if (!urgent && now - lastSlew < slew) return;
    lastSlew = now;
    uint8_t next = current + dir;
    if (logChanges) {
      logf("autobrightness: ambient=%.1f counts target=%i thermal=%i : %i->%i",
           ambientFilt16 / 16.0, want, thermalCap, current, next);
    }
    current = next;
  }
};

#endif // AUTOBRIGHTNESS_H
