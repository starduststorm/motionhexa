#ifndef HWTEST_H
#define HWTEST_H

// HWTEST serial command: a few-second hardware self-test for unit intake, driven by ./hwtest on the host.
// The firmware only measures; every line is "HWTEST <section> key=value ..." and the host applies the limits and keeps the
// log, so limits can move without reflashing and the raw numbers accumulate into a per-part spread across units.
// While it runs it owns the panel: dark, then full red / green / blue / white, so a person can spot dead pixels and the
// photosensors can confirm the pixel rail actually lights (lit vs dark counts).
// Once the host has judged the run it sends back "HWTEST RESULT PASS|WARN|FAIL", and the unit blinks a small hexagon in
// the middle of the panel green / yellow / red a few times (or until the next serial command), so a person working a
// rack of units can see which one to pull, then goes back to running normally.
// Include after bench.h's dependencies (power.h, MotionManager.h, audioInput, updater) in main.cpp.
// A minihexa has none of the parts (no i2c, gauge, imu, mic, photosensors, button): its test is the panel fills, the boot
// log and the serial round trip, so intake still gets a result per unit instead of a timeout.

#if HAS_HWTEST

const char *kHWTestCommand = "HWTEST";
const char *kHWTestResultCommand = "HWTEST RESULT ";

// core0 -> core1 -> core0: the parts of the test that need Wire, which core1 owns
struct HWTestBusReport {
  volatile bool requested = false;
  volatile bool done = false;
  uint8_t sda = 0, scl = 0;  // idle line levels, both should be high
  uint8_t acks[8] = {0};     // addresses that ACK
  uint8_t ackCount = 0;
  int magAck = -1, magProductId = -1;
  uint16_t gaugeDeviceType = 0;
};
HWTestBusReport hwTestBus;

void hwTestCore1() {
  assert(1 == get_core_num(), "hwTestCore1 not on core1");
  if (!hwTestBus.requested) return;
#if HAS_BATTERY
  hwTestBus.sda = gpio_get(SDA);
  hwTestBus.scl = gpio_get(SCL);
  hwTestBus.ackCount = 0;
  if (hwTestBus.sda && hwTestBus.scl) { // a held line makes every probe below run to its timeout
    for (uint8_t a = 1; a < 127 && hwTestBus.ackCount < ARRAY_SIZE(hwTestBus.acks); ++a) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission(true) == 0) hwTestBus.acks[hwTestBus.ackCount++] = a;
    }
#if MOTION_HW_BMI270_MMC5603
    Wire.beginTransmission(MMC56X3_DEFAULT_ADDRESS);
    Wire.write(0x39); // product id, 0x10 expected
    hwTestBus.magAck = Wire.endTransmission(false);
    if (Wire.requestFrom((uint8_t)MMC56X3_DEFAULT_ADDRESS, (uint8_t)1) == 1) hwTestBus.magProductId = Wire.read();
    else Wire.endTransmission(true);
#endif
    hwTestBus.gaugeDeviceType = lipo.deviceType();
  }
#endif
  hwTestBus.requested = false;
  hwTestBus.done = true;
}

class HWTest {
  static const unsigned long kDarkMS = 400, kColorMS = 650;   // dark, then 4 colors: 3.0s of panel
  static const unsigned long kGaugeWaitMS = 7000;             // a fresh gauge takes its ~1.5s config path, then a few seconds to FCC != 0
  // whole-panel fills are the heaviest thing this board ever draws and intake units sit on USB with flat cells:
  // ~0.25A for a primary, ~0.35A for white
  static const uint8_t kColorBrightness = 12, kWhiteBrightness = 6;
  // result hexagon: 19 pixels, so it can be brighter than the fills
  static const unsigned long kResultBlinkMS = 1000, kResultMS = 3 * kResultBlinkMS;
  static const uint8_t kResultRadius = 2, kResultBrightness = 48;
  unsigned long startedAt = 0;
  bool running = false;
  CRGB resultColor = CRGB::Black; // black: no result showing
  unsigned long resultAt = 0;
  // motion
  uint32_t imuFrames = 0, magFirstCount = 0, magLastCount = 0, magFrames = 0;
  double accSumG = 0, magSumUT = 0;
  float gyrMaxDps = 0, accMinG = 0, accMaxG = 0;
  // microphone
  uint32_t micSamples = 0;
  int32_t micMin = 0, micMax = 0;
  double micSum = 0, micSumSq = 0;
  // photosensors
  uint32_t photoDark[3] = {0}, photoLit[3] = {0};
  uint16_t photoDarkN = 0, photoLitN = 0;

  static uint32_t readPhoto(int s) {
#if HAS_AUTO_BRIGHTNESS
    return analogRead(kPhotoPins[s]);
#else
    return 0;
#endif
  }

  void sample(const MotionFrame &mf) {
    if (mf.hasAccelGyro) {
      float g = sqrtf((float)mf.acc.x * mf.acc.x + (float)mf.acc.y * mf.acc.y + (float)mf.acc.z * mf.acc.z) / MotionManager::accelToGScale;
      float dps = sqrtf((float)mf.gyr.x * mf.gyr.x + (float)mf.gyr.y * mf.gyr.y + (float)mf.gyr.z * mf.gyr.z) / 16.4f;
      if (imuFrames == 0 || g < accMinG) accMinG = g;
      if (imuFrames == 0 || g > accMaxG) accMaxG = g;
      gyrMaxDps = max(gyrMaxDps, dps);
      accSumG += g;
      imuFrames++;
    }
    if (mf.hasMag) {
      if (magFrames == 0) magFirstCount = mf.magCount;
      magLastCount = mf.magCount;
      magSumUT += sqrtf((float)mf.mag.x * mf.mag.x + (float)mf.mag.y * mf.mag.y + (float)mf.mag.z * mf.mag.z) / 10.0f;
      magFrames++;
    }
#if HAS_MICROPHONE
    int16_t samples[DEFAULT_NSAMP];
    size_t n = audioInput.read(samples, sizeof(samples)) / sizeof(samples[0]);
    for (size_t i = 0; i < n; ++i) {
      if (micSamples == 0 || samples[i] < micMin) micMin = samples[i];
      if (micSamples == 0 || samples[i] > micMax) micMax = samples[i];
      micSum += samples[i];
      micSumSq += (double)samples[i] * samples[i];
      micSamples++;
    }
#endif
  }

  void report(const MotionFrame &mf, BatteryData &bd, bool vbus, bool button) {
#if HAS_BATTERY
    char scan[48] = "";
    for (int i = 0, n = 0; i < hwTestBus.ackCount; ++i) n += snprintf(scan + n, sizeof(scan) - n, "%s0x%02x", i ? "," : "", hwTestBus.acks[i]);
    logf("HWTEST i2c report=%i sda=%u scl=%u scan=%s mag_ack=%i mag_id=0x%02x gauge_type=0x%x", hwTestBus.done, hwTestBus.sda, hwTestBus.scl,
         scan[0] ? scan : "none", hwTestBus.magAck, hwTestBus.magProductId & 0xff, hwTestBus.gaugeDeviceType);
    logf("HWTEST gauge init=%i boot_type=0x%x sampled=%u ready=%i plausible=%i mv=%u soc=%u ma=%i temp_k=%u flags=0x%x status=0x%x full_mah=%u presence=%u",
         powerState.batteryInitialized, gaugeDeviceTypeRead, bd.sampled, bd.gaugingReady(), bd.sampled && bd.plausible(), bd.voltage, bd.stateOfCharge,
         bd.current, bd.temperature, bd.flags, bd.controlStatus, bd.fullCapacity, bd.presence);
#endif
#if HAS_MOTION
    logf("HWTEST imu present=%i frames=%lu acc_g=%.3f acc_min_g=%.3f acc_max_g=%.3f gyr_max_dps=%.2f temp_c=%.1f", MotionManager::manager().hasSensor(),
         imuFrames, imuFrames ? accSumG / imuFrames : 0.0, accMinG, accMaxG, gyrMaxDps, mf.tempC);
    logf("HWTEST mag present=%i init_retries=%u frames=%lu samples=%lu field_ut=%.1f calibrated=%i", MotionManager::manager().hasMagSensor(),
         mf.magInitRetries, magFrames, magLastCount - magFirstCount, magFrames ? magSumUT / magFrames : 0.0, mf.magCalibrated);
#endif
#if HAS_MICROPHONE
    double mean = micSamples ? micSum / micSamples : 0;
    logf("HWTEST mic samples=%lu min=%li max=%li mean=%.1f rms=%.1f", micSamples, micMin, micMax, mean,
         micSamples ? sqrt(max(0.0, micSumSq / micSamples - mean * mean)) : 0.0);
#endif
#if HAS_AUTO_BRIGHTNESS
    char photo[96] = "";
    for (int s = 0, n = 0; s < (int)ARRAY_SIZE(kPhotoPins); ++s) {
      n += snprintf(photo + n, sizeof(photo) - n, " dark%i=%lu lit%i=%lu", s, photoDarkN ? photoDark[s] / photoDarkN : 0, s, photoLitN ? photoLit[s] / photoLitN : 0);
    }
    logf("HWTEST photo n_dark=%u n_lit=%u%s", photoDarkN, photoLitN, photo);
#endif
#ifdef GPOUT_PIN
    logf("HWTEST io vbus=%i button=%i gpout=%i", vbus, button, digitalRead(GPOUT_PIN));
#else
    logf("HWTEST io vbus=%i button=%i", vbus, button);
#endif
    logf("HWTEST END ms=%lu", millis() - startedAt);
  }

  void showResult() {
    unsigned long t = millis() - resultAt;
    if (t >= kResultMS) {
      resultColor = CRGB::Black;
      return;
    }
    bool on = t % kResultBlinkMS < kResultBlinkMS * 2 / 3;
    for (PixelIndex px = 0; px < LED_COUNT; ++px) {
      Axial ax = axial.axialFromPixelIndex(px); // the center pixel is (0,0)
      int ring = max(abs(ax.q()), max(abs(ax.r()), abs(ax.q() + ax.r())));
      ctx.leds[px] = on && ring <= kResultRadius ? resultColor : CRGB::Black;
    }
#ifdef LED_LINE_0_PWR_PIN
    digitalWrite(LED_LINE_0_PWR_PIN, true);
#endif
    FastLED.setBrightness(kResultBrightness);
    FastLED.show();
  }

public:
  bool active() { return running || resultColor != CRGB(CRGB::Black); }

  // every serial line: HWTEST starts a run, HWTEST RESULT <result> shows the host's result, anything else clears it
  void command(const char *line, const char *hardwareVersion) {
    if (running) return;
    resultColor = CRGB::Black;
    if (strcmp(line, kHWTestCommand) == 0) {
      begin(hardwareVersion);
    } else if (strncmp(line, kHWTestResultCommand, strlen(kHWTestResultCommand)) == 0) {
      const char *result = line + strlen(kHWTestResultCommand);
      resultColor = strcmp(result, "PASS") == 0 ? CRGB::Green : strcmp(result, "WARN") == 0 ? CRGB::Yellow : CRGB::Red;
      resultColor.scale8(0x20);
      resultAt = millis();
      logf("HWTEST RESULT %s", result);
    }
  }

  void begin(const char *hardwareVersion) {
    char serialNumber[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    pico_get_unique_board_id_string(serialNumber, sizeof(serialNumber));
    *this = HWTest();
    running = true;
    startedAt = millis();
    hwTestBus.done = false;
    hwTestBus.requested = true;
#if HAS_AUTO_BRIGHTNESS
    // as HexaAutoBrightness::setup() leaves them (it may be compiled out of a bench build); the sensor supply stays up
    analogReadResolution(12);
    pinMode(PHOTOSENSOR_POWER_PIN, OUTPUT);
    digitalWrite(PHOTOSENSOR_POWER_PIN, true);
#endif
    logf("HWTEST BEGIN sn=%s fw=%s hw=%s uptime_ms=%lu watchdog_reboot=%i", serialNumber, FW_VERSION, hardwareVersion, millis(),
         watchdog_enable_caused_reboot()); // not watchdog_caused_reboot(): the bootrom reboots through the watchdog after a UF2 flash
#if LOG_BOOT_CAPTURE_BYTES
    // what was logged before anyone was listening, a line at a time
    char line[200];
    size_t n = 0;
    for (const char *c = bootLog(); ; ++c) {
      if (*c == '\n' || *c == '\0' || n == sizeof(line) - 1) {
        line[n] = '\0';
        if (n && strncmp(line, "HWTEST", 6) != 0) logf("HWTEST BOOT %s", line);
        n = 0;
        if (*c == '\0') break;
      } else if (*c != '\r') {
        line[n++] = *c;
      }
    }
#endif
  }

  // core0, every frame while active, in place of the pattern pipeline
  void loop(const MotionFrame &mf, BatteryData &bd, bool vbus, bool button) {
    if (!running) {
      showResult();
      return;
    }
    unsigned long t = millis() - startedAt;
    const CRGB colors[] = {CRGB::Red, CRGB::Green, CRGB::Blue, CRGB::White};
    unsigned long panelMS = kDarkMS + ARRAY_SIZE(colors) * kColorMS;
#if HAS_BATTERY
    bool gaugeSettled = bd.sampled && (bd.gaugingReady() || !powerState.batteryInitialized);
#else
    bool gaugeSettled = true;
#endif
    if (t >= panelMS && hwTestBus.done && (gaugeSettled || t >= kGaugeWaitMS)) {
      ctx.leds.fill_solid(CRGB::Black);
      FastLED.show();
      report(mf, bd, vbus, button);
      running = false;
      return;
    }
    sample(mf);
    CRGB color = CRGB::Black;
    if (t >= kDarkMS && t < panelMS) color = colors[(t - kDarkMS) / kColorMS];
#if HAS_AUTO_BRIGHTNESS
    // photosensors: settled dark, and settled white
    unsigned long phaseT = t < kDarkMS ? t : (t - kDarkMS) % kColorMS;
    if (phaseT > 150) {
      bool lit = color == CRGB(CRGB::White);
      if (lit || t < kDarkMS) {
        for (int s = 0; s < (int)ARRAY_SIZE(kPhotoPins); ++s) (lit ? photoLit : photoDark)[s] += readPhoto(s);
        (lit ? photoLitN : photoDarkN)++;
      }
    }
#endif
    ctx.leds.fill_solid(color);
#ifdef LED_LINE_0_PWR_PIN
    digitalWrite(LED_LINE_0_PWR_PIN, true);
#endif
    FastLED.setBrightness(color == CRGB(CRGB::White) ? kWhiteBrightness : kColorBrightness);
    FastLED.show();
  }
};
HWTest hwTest;

#endif // HAS_HWTEST
#endif
