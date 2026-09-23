#define DEBUG 0
#define LOG_BOOT_CAPTURE_BYTES 3072 // keep the boot log for HWTEST (nothing is listening on Serial that early)
#define WAIT_FOR_SERIAL 0
#define PERF_TIMING 1 // log detailed per-frame phase timing

// Boot timing / core1 stall instrumentation. 
#define DEBUG_BOOT_TIMING 0
#if DEBUG_BOOT_TIMING
#define btlogf(format, ...) logf(format, ## __VA_ARGS__)
const unsigned long kStallLogMS = 15; // ~3 motion frames' worth
#else
#define btlogf(format, ...)
#endif

// manually-bumped versioning
#define SOFTWARE_VERSION "1.1"

#include <Arduino.h>
#include <SPI.h>
#include "pico/multicore.h"

#include "Wire.h"

#include "pinout.h"

#define FASTLED_USE_PROGMEM 1
#define FASTLED_USE_GLOBAL_BRIGHTNESS 1
#define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>
#include <functional>

#define DUSTLIB_SHARED_COLORMANAGER true
#include <util.h>
#include <apa102pio.h>
#include "ledgraph.h"

#include <patterning.h>
#include <controls.h>
#include <drawing.h>
#include <updating.h>

#include "power.h"

#include "MotionManager.h"
#include "compass.h"

#define AUTO_BRIGHTNESS (HAS_AUTO_BRIGHTNESS)

#if AUTO_BRIGHTNESS
#include "autobrightness.h"
#endif
#include "photobench.h"

// photosensor-driven global brightness
#if PHOTOSENSOR_COUNT > 0
static const int kPhotoPins[] = {
#if PHOTOSENSOR_COUNT > 1
  PHOTOSENSOR_READ_PIN, PHOTOSENSOR1_READ_PIN, PHOTOSENSOR2_READ_PIN
#else
  PHOTOSENSOR_READ_PIN
#endif
};
#endif
#if AUTO_BRIGHTNESS
HexaAutoBrightness *autoBrightness;
#endif
#if PHOTO_BENCH
PhotoBench *photoBench;
#endif
uint8_t kDefaultBrightness = 15;

DrawingContext ctx;
HardwareControls controls;
SPSTButton *mainButton = NULL;

FrameCounter fc;
PatternManager patternManager(ctx);

#include <audio.h>
#if HAS_MICROPHONE
AudioInputPDM audioInput(PDM_DATA, PDM_CLK, (HARDWARE_VERSION >= 4));
#else
ShimAudioProcessing audioInput; // synthetic samples so the audio patterns still run
#endif
// TODO: fft numBins should be pattern-determined. how to rationalize this with a shared fft?
FFTProcessing fftProcessing(audioInput, 10, 128);

#include "patterns.h"

IndexedPatternRunner *indexedRunner; // main pattern runner
std::shared_ptr<PatternRunner> powerOnOffRunner;
std::shared_ptr<PatternRunner> lowBatteryRunner; // refused power-on indication; owns the panel until it powers us off

RP2040Updater *updater;

static bool serialTimeout = false;
static unsigned long setupDoneTime;

const unsigned kBootDelay = 1000;

void init_i2c() {
  assert(1 == get_core_num(), "init_i2c not on core1");
  Wire.setSDA(SDA);
  Wire.setSCL(SCL);
  Wire.setClock(400000);
  Wire.begin();
}

void init_serial() {
  Serial.begin(57600);
#if WAIT_FOR_SERIAL
  long setupStart = millis();
  while (!Serial) {
    if (millis() - setupStart > 8000) {
      serialTimeout = true;
      break;
    }
    delay(10);
  }
  Serial.flush();
  delay(10); // Serial needs a bit more time before it'll actually log?
  logf("begin - waited %ims for Serial", millis() - setupStart);
#endif
}

void serialTimeoutIndicator() {
  FastLED.setBrightness(10);
  ctx.leds.fill_solid(CRGB::Black);
  if ((millis() - setupDoneTime) % 250 < 100) {
    ctx.leds.fill_solid(CRGB::Red);
  }
#ifdef LED_LINE_0_PWR_PIN
  gpio_put(LED_LINE_0_PWR_PIN, true);
#endif
  FastLED.show();
  delay(20);
}

auto_init_mutex(core1DataLock);
MotionFrame _gMotionFrame; // locked motion read
BatteryData _gBatteryData = {0}; // locked BatteryData read
bool _gCore1DataGetNext = true; // prevent core1 from doing multiple motion reads in a single frame
volatile bool compassCalRequested = false; // core0 -> core1: COMPASSCAL, discard the hard-iron offset and recalibrate from zero (core1 owns the sensors)
volatile bool magSetResetRequested = false; // core0 -> core1: MAGSET bench diagnostic (v7 MMC5603NJ)
volatile bool i2cScanRequested = false;      // core0 -> core1: I2CSCAN bench diagnostic, lists ACKing addresses on Wire
void hwTestCore1();                          // hwtest.h: the HWTEST self-test's share of core1 (it owns Wire)

void getAsyncData(MotionFrame *motionFrameRef, BatteryData *batteryDataRef) {
  assert(0 == get_core_num(), "getAGMT not on core0");
  mutex_enter_blocking(&core1DataLock);
  MotionFrame motionFrame = _gMotionFrame;
  BatteryData bd = _gBatteryData;
  _gCore1DataGetNext = true;
  mutex_exit(&core1DataLock);

  if (motionFrameRef) *motionFrameRef = motionFrame;
  if (batteryDataRef) *batteryDataRef = bd;
}

inline void logMotionPublishGap() {
#if DEBUG_BOOT_TIMING
  static unsigned long lastMotionPublish = 0;
  unsigned long now = millis();
  if (lastMotionPublish != 0 && now - lastMotionPublish >= kStallLogMS) {
    btlogf("[t=%lu] core1: %lums gap between published motion frames", now, now - lastMotionPublish);
  }
  lastMotionPublish = now;
#endif
}

#if !HAS_MOTION
static void synthesizeMotionFrame(MotionFrame &frame) {
  const float rotation = 2 * PI * (millis() % 36000) / 36000.0f;
  frame.acc = vector16(8000 * sinf(rotation), 8000 * cosf(rotation), 0);
  frame.accG = vectorf(frame.acc.x / MotionManager::accelToGScale, frame.acc.y / MotionManager::accelToGScale, 0);
  frame.gyr = vector16(100 * sinf(rotation), 100 * cosf(rotation), 0);
  frame.hasAccelGyro = true;
}
#endif

void hard_reset_check_core1() {
#if HAS_BUTTON
  assert(1 == get_core_num(), "hard_reset_check_core1 not on core1");
    // hard reset
  static unsigned long lastButtonReleased = 0 ;
  static unsigned long lastMillis = 0;
  unsigned long curMillis = millis();
  if (curMillis < lastMillis) {
    // handle millis overflow
    lastButtonReleased = curMillis;
  }
  if (digitalRead(BUTTON_0) == BUTTON_PRESSED_STATE) {
    if (millis() - lastButtonReleased > 10000) {
      logf("hard reset!");
      Serial.flush();
      watchdog_reboot(0,0,0);
    }
  } else {
    lastButtonReleased = curMillis;
  }
  lastMillis = curMillis;
#endif
}

// we use arduino-pico's setup1()/loop1() for EEPROM support. core1_separate_stack gives it an 8KB heap stack
// instead of pico-sdk's 2KB default in SCRATCH_X; the BMI270 init path (Bosch API + SPI + logf/USB) is a little (~1.2KB) deep.
bool core1_separate_stack = true;
static unsigned long motionStartedAt = 0;

void setup1() {
  assert(1 == get_core_num(), "setup1 not on core1");

#if HAS_MOTION || HAS_BATTERY
  init_i2c();
#endif

  unsigned long motionInitStart = millis();
  MotionManager::manager().init(kHexaMotionPlacement);
  motionStartedAt = millis();
  btlogf("[t=%lu] core1: motion init took %lums (i2c up at t=%lu)",
         motionStartedAt, motionStartedAt - motionInitStart, motionInitStart);
}

void loop1() {
  {
    while (!_gCore1DataGetNext) {
      hard_reset_check_core1();
      delayMicroseconds(100); // FIXME: i would rather do this with multicore fifo but cannot seem to get fifo to work at all
    }
    hard_reset_check_core1();
    if (compassCalRequested) {
      compassCalRequested = false;
      MotionManager::manager().restartCompassCalibration();
    }
#if MOTION_HW_BMI270_MMC5603
    if (magSetResetRequested) {
      magSetResetRequested = false;
      MotionManager::manager().magSetReset();
    }
    if (i2cScanRequested) {
      i2cScanRequested = false;
      MotionManager::manager().i2cScan();
    }
#endif
#if HARDWARE_VERSION >= 5
    hwTestCore1();
#endif
    MotionFrame motionFrame = MotionManager::manager().loop();
#if !HAS_MOTION
    synthesizeMotionFrame(motionFrame);
#endif

    // Publish motion before touching the gauge, so battery i2c lands where core1 would otherwise
    // be waiting on _gCore1DataGetNext rather than inside a frame core0 is waiting on.
    mutex_enter_blocking(&core1DataLock);
    _gMotionFrame = motionFrame;
    _gCore1DataGetNext = false;
    mutex_exit(&core1DataLock);
    logMotionPublishGap();

#if HARDWARE_VERSION >= 5
    BatteryData bd = {0};
    if (battery_step_core1(motionStartedAt, bd)) {
      mutex_enter_blocking(&core1DataLock);
      _gBatteryData = bd;
      mutex_exit(&core1DataLock);
#if DEBUG_BOOT_TIMING
      static bool loggedFirstReady = false;
      if (!loggedFirstReady && bd.gaugingReady()) {
        loggedFirstReady = true;
        logf("[t=%lu] core1: first gauging-ready sample (soc=%u%%, flags=%X, status=%X, detected=%i)",
             millis(), bd.stateOfCharge, bd.flags, bd.controlStatus, bd.batteryDetected());
      }
#endif
    }
#endif
  }
}

volatile bool buttonWake = false;
void buttonUpISR() {
  buttonWake = true;
}

#if HARDWARE_VERSION >= 4
void stopHexa() {
  indexedRunner->stop();
  powerState.setRunning(false);
  // v5 units sometimes get stuck with vbus remaining powered here even if we are unplugged
  // so always power off in this case to reset state, rather than checking vbus.
  powerOff();
}

// animated power off from the running state (long press, low battery)
void beginPowerOff() {
  if (patternManager.hasTestRunner()) {
    // special case test runner since the power off animation will not run
    powerOff();
  } else if (!powerOnOffRunner) {
    powerOnOffRunner = patternManager.runOneShotPattern([](PatternRunner&) {
      return new PowerOnOffAnimation(false);
    }, 0xFF, 0xFF, [](PatternRunner&) {
      stopHexa();
      powerOnOffRunner.reset();
    });
    powerOnOffRunner->animateDim = true;
  }
}

#if HARDWARE_VERSION >= 7
// Low battery at power-on: drop whatever the power-on sequence got to, pulse the indication, power off.
// (powerOff() idles dark for as long as the button stays held.)
void refuseStartForLowBattery() {
  lowBatteryRunner = patternManager.runOneShotPattern([](PatternRunner&) {
    return new LowBatteryIndicator();
  }, 0xFF, 0xFF, [](PatternRunner&) {
    ctx.leds.fill_solid(CRGB::Black);
    FastLED.show();
    stopHexa();
    lowBatteryRunner.reset(); // still here: VBUS arrived, or the LOWBATT bench command while plugged in
  });
  if (powerOnOffRunner) {
    // its completion sees lowBatteryRunner and stands down
    patternManager.removeRunner(powerOnOffRunner);
  }
  if (powerState.isRunning()) {
    indexedRunner->stop();
    powerState.setRunning(false);
  }
}
#endif
#endif

void startupCompleted() {
  logf("Startup completed");
  powerState.setRunning(true);
  indexedRunner->runPatternAtIndex(0);
}

#include "bench.h"
#include "hwtest.h"
const char *hardwareVersionString = "";

/* ------ Setup ------------------------------------------------------------------------------------------------------------ */

void setup() {
  init_serial();

#if !DEBUG
  // watchdog barks if we hang or hardfault
  // load register is 24 bits of microseconds; RP2040 (errata E1) ticks it twice per µs so 8388ms is its ceiling, RP2350 ticks
  // once per µs and could go to ~16.7s. pico-sdk clamps either way, so 8388 is a safe max on both.
  watchdog_enable(8388, true);
#endif

#if HAS_BUTTON
  mainButton = controls.addButton(BUTTON_0, BUTTON_PRESSED_STATE);
#endif

#if HARDWARE_VERSION >= 4
  // on v4, power-on happens by squeezing the unit, which often happens in a bag. 
  // if the device is squeezed to hard reset (10s), wait in a lower-power state until button-up before doing anything
  // this will also happen after crash/hang and after reprogramming, which is fine.
  if (watchdog_caused_reboot() && mainButton->isButtonPressed()) {
    logf("Watchdog detected, button pressed. Sleeping until button up...");
    Serial.flush();
    attachInterrupt(digitalPinToInterrupt(BUTTON_0), buttonUpISR, (BUTTON_PRESSED_STATE == HIGH ? FALLING : RISING));
    do {
      __wfi();
    } while (!buttonWake);
    // it's likely that we actually powered off here. in case we didn't, startup normally.
    detachInterrupt(digitalPinToInterrupt(BUTTON_0));
    delay(100);
    init_serial();
    logf("Wake up, Neo");
    buttonWake = false;
  }

#if SOFTWARE_CHARGE_LIMITER
  // disable lipo charging so we can figure out if there is a battery; ChargeController re-enables it.
  pinMode(DISABLE_CHARGE_PIN, OUTPUT);
  digitalWrite(DISABLE_CHARGE_PIN, true);
#endif

#endif

  pinMode(UNCONNECTED_PIN_1, INPUT);
  auto noise = lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint32_t));
  randomSeed(lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint32_t)));
  random16_add_entropy(lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint16_t)));

#if HARDWARE_VERSION > 1
  pinMode(LED_LINE_0_PWR_PIN, true);
#endif
#if HARDWARE_VERSION > 2
#if HARDWARE_VERSION < 5
  pinMode(PWR_SWITCH_PIN, INPUT_PULLDOWN);
#endif
  pinMode(VBUS_SENSOR_PIN, INPUT_PULLDOWN);
  
#if HARDWARE_VERSION >= 7
  // v7 pulls GPOUT up to the gauge's own 1.8V regulator output (R6); our 3.3V pull-up on top would back-feed that rail
  pinMode(GPOUT_PIN, INPUT);
#else
  pinMode(GPOUT_PIN, INPUT_PULLUP);
#endif
#endif
#if HARDWARE_VERSION >= 4
#ifdef EN_BOOST_PIN
  pinMode(EN_BOOST_PIN, OUTPUT);
#endif

  // if we booted this far, maintain our own power.
  gpio_set_function(EN_LDO_PIN, GPIO_FUNC_SIO);
  gpio_set_dir(EN_LDO_PIN, true);
  gpio_put(EN_LDO_PIN, true);

#else // HARDWARE_VERSION < 4
  powerState.setRunning(true);
#endif
  bool v6Hardware = false;
#if HARDWARE_VERSION >= 5 && HARDWARE_VERSION < 7
  pinMode(V6_DETECTOR_PIN, INPUT);
  v6Hardware = (digitalRead(V6_DETECTOR_PIN) != 0);
  logdf("v6Hardware = %i", v6Hardware);
#endif

#if defined(LED_SERIAL_DATA)
  FastLED.addLeds<WS2812B, LED_SERIAL_DATA, GRB>(ctx.leds, LED_COUNT);
#elif defined(LED_SPI0_TX)
  // APA102/SK9822: our own PIO+DMA transport at 16MHz
  static APA102PIOController<BGR> ledController(LED_SPI0_TX, LED_SPI0_SCK, 16000000);
  FastLED.addLeds(&ledController, &ctx.leds[0], LED_COUNT);//.setCorrection(0xFFB0C0);
#else
#error "no pixel data pin in pinout.h"
#endif

#if DEBUG
#ifdef LED_LINE_0_PWR_PIN
  digitalWrite(LED_LINE_0_PWR_PIN, true);
#endif
  ctx.leds.fill_solid(CRGB::Red);
  FastLED.setBrightness(1);
  FastLED.show();
  FastLED.delay(10);
#endif

  patternManager.registerPattern<MotionHexa>();
  patternManager.registerPattern<TriBounce>();
#if HAS_MOTION
  patternManager.registerPattern<PixelDust>();
  patternManager.registerPattern<PixelSand>();
  patternManager.registerPattern<LargeBouncyBall>();
  patternManager.registerPattern<PulseHexaSmooth>();
#endif
  patternManager.registerPattern<PridefulSpinnyThing>();
#if HAS_MOTION
  patternManager.registerPattern<TriangleSpin>();
  patternManager.registerPattern<CompassPattern>();
#endif
#if HAS_MICROPHONE
  patternManager.registerPattern<SparkleDroplets>();
#endif
  patternManager.registerPattern<BlobDroplets>();
  patternManager.registerPattern<SoundBits>();
  
#if HARDWARE_VERSION >= 3
  patternManager.registerPattern<ChargingPattern>(1);
  auto chargingRunner = patternManager.setupConditionalRunner<ChargingPattern>([](PatternRunner &runner) -> uint8_t {
    return chargingPatternCheck(runner, powerState);
  }, 0xFD, 0xFF);
  chargingRunner->animateDim = true;
#endif
  
#if HAS_BUTTON
  indexedRunner = patternManager.setupIndexedRunner(0);

  mainButton->ignoreEventsUntilFirstButtonUp = true;
  mainButton->onSinglePress([]() {
    if (powerState.isRunning()) {
      indexedRunner->nextPattern();
    }
  });
  mainButton->onDoublePress([]() {
    if (powerState.isRunning()) {
      indexedRunner->previousPattern();
    }
  });
#else
  // no button: autoprogression through patterns
  indexedRunner = patternManager.setupRandomRunner(60*1000, 500);
#endif
#if DEBUG_PHYSICS && HAS_BUTTON
  mainButton->onDoubleLongPress([]() {    
    physicsDebugFlag = !physicsDebugFlag;
  });
#endif
#if HARDWARE_VERSION >= 4
  mainButton->longPressInterval = 1000;
  mainButton->onLongPress([]() {
    logf("Long press! isHexaRunning = %i", powerState.isRunning());
    if (powerState.isRunning()) {
      beginPowerOff();
    }
  });
#endif

  initLEDGraph();
  assert(ledgraph.adjList.size() == LED_COUNT, "adjlist size should match LED_COUNT");

#if AUTO_BRIGHTNESS
  autoBrightness = new HexaAutoBrightness(kPhotoPins, ARRAY_SIZE(kPhotoPins), PHOTOSENSOR_POWER_PIN);
  autoBrightness->setup();
  autoBrightness->logChanges = true;
#endif

#if PHOTO_BENCH
  photoBench = new PhotoBench(ctx.leds, kPhotoPins, ARRAY_SIZE(kPhotoPins), PHOTOSENSOR_POWER_PIN);
  photoBench->setup();
#if AUTO_BRIGHTNESS
  photoBench->ab = autoBrightness;
#endif
#endif

  patternManager.setup();

  // stream audio forever, since stopping and starting PDM introduces a noticeable hitch during pattern switching.
  // TODO: stop audio device when not in use by a pattern, but don't toggle twice between two audio patterns?
  audioInput.subscribe();

#if MINI_VERSION
  hardwareVersionString = "mini" xstr(MINI_VERSION);
#else
  hardwareVersionString = (v6Hardware ? "6" : xstr(HARDWARE_VERSION));
#endif
  updater = new RP2040Updater("motionhexa", SOFTWARE_VERSION, hardwareVersionString, [](void) {
    patternManager.runOneShotPattern<BlinkIdentifyPattern>(0xFE, 0xFF);
  });

  setupDoneTime = millis();
  logf("setup done");
}

/* ------ Loop ------------------------------------------------------------------------------------------------------------ */

#if PERF_TIMING
static uint32_t perfPatternUS = 0, perfShowUS = 0, perfLoopUS = 0;
static uint32_t perfFrames = 0;
static unsigned long perfLastLog = 0;
#define PERF_MARK() uint32_t _perfMark = micros()
#define PERF_ACCUM(counter) counter += micros() - _perfMark
#else
#define PERF_MARK()
#define PERF_ACCUM(counter)
#endif

void loop() {
#if PERF_TIMING
  uint32_t perfLoopStart = micros();
#endif
#if !DEBUG
  // pet the dog
  watchdog_update();
#endif

  if (serialTimeout && millis() - setupDoneTime < 1000) {
    serialTimeoutIndicator();
    return;
  }

  bool isVBUSPowered = false;
#if HARDWARE_VERSION > 2
  bool isButtonPressed = mainButton->isButtonPressed();
  isVBUSPowered = digitalRead(VBUS_SENSOR_PIN);
#if HARDWARE_VERSION >= 5
  isVBUSPowered = vbusSense.update(isVBUSPowered);
#endif
#endif
#if HARDWARE_VERSION >= 4
  if (lowBatteryRunner) {
    // refusing to start; the indication's completion powers off
  } else if (!powerState.isRunning()) {
    if (powerOnOffRunner) {
      PowerOnOffAnimation *pattern = (PowerOnOffAnimation *)powerOnOffRunner->pattern;
      assert(pattern, "PowerOnOffAnimation exists but no pattern?");
      if (pattern) {
        if (!isButtonPressed && pattern->animatingPowerOn && pattern->progress() > 0.6) {
          // call it good if power-on animation is almost finished when button is released
          startupCompleted();
          // allow the first pattern to animate in even before we're done
          powerOnOffRunner->dimAmount = 0;
        } else {
          // set animation direction
          pattern->setPoweringOn(isButtonPressed);
        }
      }
#if HARDWARE_VERSION >= 7
    } else if (!lowBattery.startResolved(isVBUSPowered)) {
      // hold the power-on animation for the first gauge sample: a flat cell gets the low battery indication instead
#endif
    } else if (isButtonPressed && !patternManager.hasTestRunner()) {
      // we need to pause button events here since we don't know how many times it will be pressed and released before the animation is done
      mainButton->pauseEvents = true;
      powerOnOffRunner = patternManager.runOneShotPattern([](PatternRunner&) {
        return new PowerOnOffAnimation(true);
      }, 0xFF, 0xFF, [](PatternRunner&) {
        if (lowBatteryRunner) {
          powerOnOffRunner.reset();
          return;
        }
        if (!powerState.isRunning()) { // we might have called it good early
          bool isButtonPressed = mainButton->isButtonPressed();
          if (isButtonPressed) {
            startupCompleted();
          } else {
            logf("Startup aborted. Powering off...");
            stopHexa();
          }
        }
        powerOnOffRunner.reset();
        // in case button is still down, don't change patterns on this next button up
        mainButton->seenFirstButtonUp = false;
        // and only unpause events now that the animation is complete, since we may have stopped the animation early
        mainButton->pauseEvents = false;
      });
      powerOnOffRunner->animateDim = true;
    } else if (patternManager.hasTestRunner()) {
      powerState.setRunning(true);
    }
  }

  if (!isButtonPressed && !isVBUSPowered && !powerState.isRunning() && !powerOnOffRunner && !lowBatteryRunner) {
    // unplugged USB while not drawing patterns or released button early during power on
    logf("No USB, no button, and no intent to run. Powering off...");
    powerOff();
    return;
  }
#endif

#if SOFTWARE_CHARGE_LIMITER
  // sample the analog battery sense divider for core1's battery log (adc must stay core0-only)
  static unsigned long lastBatterySenseRead = 0;
  if (millis() - lastBatterySenseRead > 1000) {
    lastBatterySenseMV = batterySenseMV();
    lastBatterySenseRead = millis();
  }
#endif

  getAsyncData(&MotionManager::motionFrame, &batteryData);
  powerState.update(isVBUSPowered, batteryData);
#if HARDWARE_VERSION >= 7
  if (!lowBatteryRunner) {
    LowBatteryMonitor::Result lowBatteryResult = lowBattery.update(isVBUSPowered, batteryData);
    if (lowBatteryResult == LowBatteryMonitor::shutdown && powerState.isRunning() && !powerOnOffRunner) {
      logf("Low battery (%umV). Powering off...", batteryData.voltage);
      beginPowerOff();
    } else if (lowBatteryResult == LowBatteryMonitor::refuseStart) {
      // powering on from a flat cell: say so rather than start what we can't sustain
      logf("Low battery (%umV), not starting", batteryData.voltage);
      refuseStartForLowBattery();
    }
  }
#endif

  Compass::update(MotionManager::motionFrame);

  // shared fft cache reset
  fftProcessing.frameReset();

  char *serialLine = readSerialLine();
  updater->loop(serialLine);
  benchLoop(serialLine);
#if HARDWARE_VERSION >= 5
  if (serialLine) {
    hwTest.command(serialLine, hardwareVersionString);
  }
  if (hwTest.active()) {
    hwTest.loop(MotionManager::motionFrame, batteryData, isVBUSPowered, isButtonPressed);
    fc.loop();
    return;
  }
#endif

#if PHOTO_BENCH
  photoBench->tempK = batteryData.temperature;
  photoBench->setRunning = [](bool running) { powerState.setRunning(running); };
  photoBench->nextPattern = []() { indexedRunner->nextPattern(); };
  photoBench->handleCommand(serialLine);
  if (photoBench->ownsPixels()) {
    // bench drives the pixels directly; keep the LED rail up and leave patterns alone. The estimator
    // still runs so its residual can be read against a known panel state, but it doesn't set brightness.
    digitalWrite(LED_LINE_0_PWR_PIN, true);
#if AUTO_BRIGHTNESS
    autoBrightness->loop(ctx.leds, FastLED.getBrightness(), batteryData.temperature, false);
#endif
    photoBench->loop();
    fc.loop();
    fc.clampToFramerate(1000);
    return;
  }
  photoBench->loop();
#endif

  indexedRunner->paused = !powerState.isRunning();
  controls.update();
  {
    PERF_MARK();
    patternManager.loop();
    PERF_ACCUM(perfPatternUS);
  }
 
  bool pixelsNeedPower = ctx.leds;
#ifdef LED_LINE_0_PWR_PIN
  static bool pixelsHavePower = false;
  static unsigned long lastPixelsNeedPower = 0;
  if (pixelsNeedPower) {
    lastPixelsNeedPower = millis();
  }
  if (pixelsNeedPower != pixelsHavePower 
    && (pixelsNeedPower || millis() - lastPixelsNeedPower > (fc.hasFPSAssertion() ? 10000 : 500))) { // don't turn off panel for very brief periods
    logf("Turn %s pixels", pixelsNeedPower?"on":"off");
    pixelsHavePower = pixelsNeedPower;
    digitalWrite(LED_LINE_0_PWR_PIN, pixelsNeedPower);
  }
#else
  const bool pixelsHavePower = true; // no pixel power switch: always show, including black
#endif

#if DEBUG
#if HARDWARE_VERSION > 2
  ctx.leds[0] = isVBUSPowered ? CRGB::Red : CRGB::Black;
  ctx.leds[3] = isButtonPressed ? CRGB::Magenta : CRGB::Black;
#endif
  ctx.leds[1] = powerState.isRunning() ? CRGB::Green : CRGB::Black;
  ctx.leds[2] = powerState.isCharging() ? CRGB::Blue : CRGB::Black;
  ctx.leds[4] = powerState.batteryInitialized ? CRGB::Yellow : CRGB::Black;
#ifdef LED_LINE_0_PWR_PIN
  digitalWrite(LED_LINE_0_PWR_PIN, true);
#endif
#endif

#if AUTO_BRIGHTNESS
  // baselines ambient at boot while the pixels are off, then updates opportunistically. It is told the brightness the
  // panel was actually shown at, which a pattern override may have set above its own level.
  autoBrightness->loop(ctx.leds, FastLED.getBrightness(), batteryData.temperature);
  uint8_t frameBrightness = autoBrightness->brightness();
#else
  uint8_t frameBrightness = kDefaultBrightness;
#endif
  if (patternBrightnessOverride >= 0) {
    frameBrightness = patternBrightnessOverride;
    patternBrightnessOverride = -1;
  }
  FastLED.setBrightness(frameBrightness);

  if (pixelsHavePower || fc.hasFPSAssertion()) {
    PERF_MARK();
    FastLED.show();
    PERF_ACCUM(perfShowUS);
  }

#if PERF_TIMING
  perfLoopUS += micros() - perfLoopStart;
  perfFrames++;
  if (millis() - perfLastLog > 2000) {
    if (perfLastLog != 0 && perfFrames > 0) {
      logf("perf avg us/frame: pattern=%lu show=%lu loop=%lu (n=%lu)",
           perfPatternUS / perfFrames, perfShowUS / perfFrames, perfLoopUS / perfFrames, perfFrames);
    }
    perfPatternUS = perfShowUS = perfLoopUS = 0;
    perfFrames = 0;
    perfLastLog = millis();
  }
#endif

  fc.loop();
  // fc.clampToFramerate(240);

  if (!pixelsNeedPower) {
    // FIXME: proper sleep
    fc.idleDelay(100);
  }
}
