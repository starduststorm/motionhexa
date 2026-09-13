#ifndef PHOTOBENCH_H
#define PHOTOBENCH_H

// Bench for the photosensor / autobrightness path (src/autobrightness.h).
//
// Serial-driven harness. While a job that draws is running the bench owns the pixels and the frame
// loop; the pattern manager is skipped and the estimator runs without controlling brightness.
// Everything it emits is CSV so scripts can drive it.
//
// Commands (case sensitive, one per line):
//   PB?                       status
//   PB END                    stop the running job, hand the pixels back
//   PB DARK                   read every sensor with all pixels off
//   PB STEP <px> <ch> <val> <gb>            settle-time capture: 1kHz reads across a pixel on/off edge
//   PB FILL <r> <g> <b> <gb> <secs>         solid fill + 2Hz sensor/temp monitor (thermal test)
//   PB MON <secs> [hz]                      monitor sensors/temp with the pixels left alone
//   PB PAT <secs> [hz]                      let patterns draw, monitor + report the estimator's view
//   PB AMB <secs> [hz]                      human-readable estimator readout, for anchoring the curve
//   PB HOLD <b>                             pin the output brightness (-1 releases)
//   PB FAKE <counts>                        inject a synthetic ambient level (-1 releases)
//   PB THERM <0|1>                          disable / enable the thermal ceiling (for testing on a hot bench)
//   PB ADC                                  raw 64-sample sums of every ADC channel (GPIO26-29)
//   PB RUN <0|1>, PB NEXT, PB RAIL <0|1>    pattern-drawing mode, next pattern, pixel rail

#include <Arduino.h>
#include <FastLED.h>
#include "pinout.h"
#if AUTO_BRIGHTNESS
#include "autobrightness.h"
#endif

#ifndef PHOTO_BENCH
#define PHOTO_BENCH 0
#endif

#if PHOTO_BENCH

// ADC reads are averaged in the integer domain: every value the bench reports is a sum of
// kSampleShift-worth of 12-bit reads, so the host divides by (1<<kSampleShift) for sub-LSB resolution.
static const int kBenchSampleShift = 6; // 64 samples
static const int kBenchSamples = 1 << kBenchSampleShift;
static const int kBenchMaxSensors = 3;
// the pixel rail comes up through bulk capacitance; the first show() after it is switched on takes a
// while to actually appear, so every job waits this long before its first measurement.
static const unsigned kRailWarmupMS = 250;
// the phototransistor decays to the floor in ~40ms on v7
static const unsigned kDarkSettleMS = 60;

class PhotoBench {
public:
  enum Job : uint8_t { jobNone, jobStep, jobFill, jobMon, jobPat, jobAmb };

private:
  CRGBArray<LED_COUNT> &leds;
  const int *readPins;
  int sensorCount;
  int powerPin;

  Job job = jobNone;
  unsigned long jobStart = 0;
  unsigned long nextActionMS = 0;
  unsigned long endMS = 0;
  unsigned periodMS = 500;

  // step cursor
  int px = 0, ch = 0, val = 255, gb = 0x20;
  uint8_t phase = 0;

  uint32_t readSum(int s) {
    uint32_t sum = 0;
    for (int i = 0; i < kBenchSamples; ++i) {
      sum += analogRead(readPins[s]);
    }
    return sum;
  }

  void readAll(uint32_t *out) {
    for (int s = 0; s < sensorCount; ++s) {
      out[s] = readSum(s);
    }
  }

  // CSV rows are built whole and emitted with one logf: loglf fragments can be split by core1's own
  // logging, which makes the capture unparseable.
  char row[224];
  int rowLen = 0;
  void rowReset() { rowLen = 0; row[0] = '\0'; }
  void rowAdd(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(row + rowLen, sizeof(row) - rowLen, fmt, ap);
    va_end(ap);
    if (n > 0) rowLen = min((int)sizeof(row) - 1, rowLen + n);
  }
  void rowAddSensors(const uint32_t *v) {
    for (int s = 0; s < sensorCount; ++s) {
      rowAdd(",%lu", (unsigned long)v[s]);
    }
  }
  void rowEmit() { logf("%s", row); }

  void show(uint8_t brightness) {
    FastLED.setBrightness(brightness);
    FastLED.show();
  }

  void finish() {
    bool owned = ownsPixels();
    logf("PB,done,%s", jobName());
    leds.fill_solid(CRGB::Black);
    show(0);
    job = jobNone;
    if (owned) {
      // 271 idle SK9822s still draw ~1mA each; in a sealed box that is enough to warm the whole
      // thing several degrees, so don't leave the rail up after a job.
      digitalWrite(LED_LINE_0_PWR_PIN, false);
    }
  }

  const char *jobName() {
    switch (job) {
      case jobStep: return "STEP";
      case jobFill: return "FILL";
      case jobMon: return "MON";
      case jobPat: return "PAT";
      case jobAmb: return "AMB";
      default: return "NONE";
    }
  }

public:
  // filled by main.cpp each frame so the bench can log them alongside the raw reads
  uint16_t tempK = 0;
#if AUTO_BRIGHTNESS
  // the live estimator, so MON/PAT rows can carry its internals next to the raw reads
  HexaAutoBrightness *ab = nullptr;
#endif
  // wired up by main.cpp so the bench can put the device into pattern-drawing mode
  std::function<void(bool)> setRunning = nullptr;
  std::function<void()> nextPattern = nullptr;

  PhotoBench(CRGBArray<LED_COUNT> &leds, const int *readPins, int sensorCount, int powerPin)
    : leds(leds), readPins(readPins), sensorCount(min(sensorCount, kBenchMaxSensors)), powerPin(powerPin) { }

  bool isActive() { return job != jobNone; }
  // true while the bench is drawing; main.cpp must not let patterns touch the pixels
  bool ownsPixels() { return job == jobStep || job == jobFill; }

  void setup() {
    analogReadResolution(12);
    for (int s = 0; s < sensorCount; ++s) {
      pinMode(readPins[s], INPUT);
    }
    if (powerPin != -1) {
      pinMode(powerPin, OUTPUT);
      digitalWrite(powerPin, true);
    }
  }

  // returns true if the line was a bench command
  bool handleCommand(char *line) {
    if (!line || strncmp(line, "PB", 2) != 0) return false;
    char *args = line + 2;
    while (*args == ' ') args++;

    if (*args == '?' || *args == '\0') {
      logf("PB,status,job=%s,gb=%i,sensors=%i,shift=%i,temp=%u",
           jobName(), (int)FastLED.getBrightness(), sensorCount, kBenchSampleShift, tempK);
      return true;
    }
    if (strncmp(args, "END", 3) == 0) {
      finish();
      return true;
    }
#if AUTO_BRIGHTNESS
    if (strncmp(args, "FAKE", 4) == 0) {
      // inject a synthetic ambient level (ADC counts) to exercise the control law
      float e = -1;
      sscanf(args + 4, "%f", &e);
      if (ab) ab->injectedAmbient16 = (e < 0 ? -1 : (int32_t)(e * 16));
      logf("PB,fake,%0.2f", e);
      return true;
    }
    if (strncmp(args, "THERM", 5) == 0) {
      int on = 1;
      sscanf(args + 5, "%i", &on);
      if (ab) ab->thermalEnabled = on;
      logf("PB,therm,%i", on);
      return true;
    }
    if (strncmp(args, "HOLD", 4) == 0) {
      int b = -1;
      sscanf(args + 4, "%i", &b);
      if (ab) ab->pinnedBrightness = b;
      logf("PB,hold,%i", b);
      return true;
    }
#endif
    if (strncmp(args, "RUN", 3) == 0) {
      int on = 1;
      sscanf(args + 3, "%i", &on);
      if (setRunning) setRunning(on);
      logf("PB,run,%i", on);
      return true;
    }
    if (strncmp(args, "NEXT", 4) == 0) {
      if (nextPattern) nextPattern();
      logf("PB,next");
      return true;
    }
    if (strncmp(args, "RAIL", 4) == 0) {
      int on = 0;
      sscanf(args + 4, "%i", &on);
      digitalWrite(LED_LINE_0_PWR_PIN, on);
      logf("PB,rail,%i", on);
      return true;
    }
    if (strncmp(args, "ADC", 3) == 0) {
      rowReset();
      rowAdd("ADC,%lu", millis());
      for (int pin = 26; pin <= 29; ++pin) {
        uint32_t sum = 0;
        for (int i = 0; i < kBenchSamples; ++i) sum += analogRead(pin);
        rowAdd(",%lu", (unsigned long)sum);
      }
      rowEmit();
      return true;
    }
    if (strncmp(args, "DARK", 4) == 0) {
      leds.fill_solid(CRGB::Black);
      show(0);
      delay(kDarkSettleMS);
      uint32_t v[kBenchMaxSensors];
      readAll(v);
      rowReset();
      rowAdd("DK,%lu", millis());
      rowAddSensors(v);
      rowAdd(",%u", tempK);
      rowEmit();
      return true;
    }
    if (strncmp(args, "STEP", 4) == 0) {
      px = 0; ch = 0; val = 255; gb = 0x20;
      sscanf(args + 4, "%i %i %i %i", &px, &ch, &val, &gb);
      px = constrain(px, 0, LED_COUNT - 1);
      leds.fill_solid(CRGB::Black);
      show(gb);
      job = jobStep; phase = 0; jobStart = millis(); nextActionMS = millis() + kRailWarmupMS;
      logf("PB,begin,STEP,px=%i,ch=%i,val=%i,gb=%i", px, ch, val, gb);
      return true;
    }
    if (strncmp(args, "FILL", 4) == 0) {
      int r = 0, g = 0, b = 0, secs = 30;
      gb = 0x20;
      sscanf(args + 4, "%i %i %i %i %i", &r, &g, &b, &gb, &secs);
      leds.fill_solid(CRGB(r, g, b));
      show(gb);
      job = jobFill; jobStart = millis(); endMS = millis() + 1000UL * secs;
      periodMS = 500; nextActionMS = millis() + kRailWarmupMS;
      logf("PB,begin,FILL,rgb=%i.%i.%i,gb=%i,secs=%i", r, g, b, gb, secs);
      return true;
    }
#if AUTO_BRIGHTNESS
    if (strncmp(args, "AMB", 3) == 0) {
      int secs = 60, hz = 2;
      sscanf(args + 3, "%i %i", &secs, &hz);
      periodMS = (hz > 0 ? 1000 / hz : 500);
      job = jobAmb; jobStart = millis(); endMS = millis() + 1000UL * secs; nextActionMS = millis();
      logf("PB,begin,AMB,secs=%i,hz=%i  (figures are ADC counts)", secs, hz);
      return true;
    }
#endif
    if (strncmp(args, "MON", 3) == 0 || strncmp(args, "PAT", 3) == 0) {
      bool pat = (args[0] == 'P');
      int secs = 30, hz = 4;
      sscanf(args + 3, "%i %i", &secs, &hz);
      periodMS = (hz > 0 ? 1000 / hz : 250);
      job = pat ? jobPat : jobMon;
      jobStart = millis(); endMS = millis() + 1000UL * secs; nextActionMS = millis();
      logf("PB,begin,%s,secs=%i,hz=%i", pat ? "PAT" : "MON", secs, hz);
      return true;
    }
    logf("PB,err,unknown,%s", args);
    return true;
  }

  void loop() {
    if (job == jobNone) return;
    unsigned long now = millis();

    switch (job) {
      case jobStep: {
        // 1kHz single-sample reads across a light-on then light-off edge, to find the settle time
        if (now < nextActionMS) return;
        unsigned long t = now - jobStart;
        if (phase == 0) {
          logf("PB,step,on,%lu", t);
          leds[px] = CRGB(ch == 0 ? val : 0, ch == 1 ? val : 0, ch == 2 ? val : 0);
          show(gb);
          jobStart = millis();
          phase = 1;
          return;
        }
        if (phase == 1 && t > 150) {
          logf("PB,step,off,%lu", t);
          leds[px] = CRGB::Black;
          show(gb);
          phase = 2;
        }
        if (phase == 2 && t > 300) {
          finish();
          return;
        }
        rowReset();
        rowAdd("ST,%lu", t);
        for (int s = 0; s < sensorCount; ++s) {
          rowAdd(",%i", analogRead(readPins[s]));
        }
        rowEmit();
        nextActionMS = now + 1;
        break;
      }

      case jobAmb: {
        if (now >= endMS) { finish(); return; }
        if (now < nextActionMS) return;
        nextActionMS = now + periodMS;
#if AUTO_BRIGHTNESS
        if (!ab) { logf("AMB: no estimator"); return; }
        rowReset();
        rowAdd("AMB t=%5.1fs ambient=%6.1f sensors", (now - jobStart) / 1000.0, ab->ambient16() / 16.0);
        for (int s = 0; s < ab->sensors(); ++s) {
          rowAdd(" [raw %.1f panel %.1f est %.1f%s%s]", ab->sensorRaw16(s) / 16.0, ab->sensorPanel16(s) / 16.0,
                 ab->sensorEstimate16(s) / 16.0, ab->sensorClean(s) ? " clean" : "", ab->sensorBlind(s) ? " BLIND" : "");
        }
        rowAdd(" brightness=%i cap=%i gauge=%uK die=%0.1fC", ab->brightness(), ab->thermalCeiling(), tempK, analogReadTemp());
        rowEmit();
#endif
        break;
      }

      case jobFill:
      case jobMon:
      case jobPat: {
        if (now >= endMS) { finish(); return; }
        if (now < nextActionMS) return;
        nextActionMS = now + periodMS;
        uint32_t v[kBenchMaxSensors];
        readAll(v);
        rowReset();
        rowAdd("MN,%s,%lu,%i", jobName(), now - jobStart, (int)FastLED.getBrightness());
        rowAddSensors(v);
#if AUTO_BRIGHTNESS
        // the estimator's own view: what it thinks the panel is contributing to each sensor, what
        // each sensor's held estimate is, and which sensors it trusted this frame.
        if (ab) {
          for (int s = 0; s < sensorCount; ++s) rowAdd(",%lu", (unsigned long)ab->sensorRaw16(s));
          for (int s = 0; s < sensorCount; ++s) rowAdd(",%lu", (unsigned long)ab->sensorPanel16(s));
          for (int s = 0; s < sensorCount; ++s) rowAdd(",%lu", (unsigned long)ab->sensorEstimate16(s));
          for (int s = 0; s < sensorCount; ++s) rowAdd(",%i", ab->sensorClean(s) ? 1 : 0);
          rowAdd(",%li,%lu,%i,%i", (long)ab->ambient16(), (unsigned long)ab->ambientFiltered16(),
                 ab->brightness(), ab->thermalCeiling());
        }
#endif
        rowAdd(",%u,%0.1f", tempK, analogReadTemp());
        rowEmit();
        break;
      }
      default: break;
    }
  }
};

#endif // PHOTO_BENCH
#endif // PHOTOBENCH_H
