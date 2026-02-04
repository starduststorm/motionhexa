#define DEBUG 0
#define WAIT_FOR_SERIAL 0

// for memory logging
#ifdef __arm__
extern "C" char* sbrk(int incr);
#else
extern char *__brkval;
#endif

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
#include "ledgraph.h"

#include <patterning.h>
#include <controls.h>
#include <drawing.h>

#include "power.h"

PowerManager powerState;

#include "MotionManager.h"

#define MEASURE_PHOTO_SENSOR_BASELINE false
PhotoSensorBrightness *autoBrightness;
const uint8_t kDefaultBrightness = 15;

DrawingContext ctx;
HardwareControls controls;
SPSTButton *mainButton = NULL;

FrameCounter fc;
PatternManager patternManager(ctx);

#include <audio.h>
AudioInputPDM audioInput(PDM_DATA, PDM_CLK, (HARDWARE_VERSION >= 4));
// TODO: fft numBins should be pattern-determined. how to rationalize this with a shared fft?
FFTProcessing fftProcessing(audioInput, 10, 128);

#include "patterns.h"

IndexedPatternRunner *indexedRunner; // main pattern runner
std::shared_ptr<PatternRunner> powerOnOffRunner;

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
  gpio_put(LED_LINE_0_PWR_PIN, true);
  FastLED.show();
  delay(20);
}

mutex_t core1DataLock;
MotionFrame _gMotionFrame; // locked AGMT/motion read
BatteryData _gBatteryData = {0}; // locked BatteryData read
bool _gCore1DataGetNext = true; // prevent core1 from doing multiple motion reads in a single frame

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

void hard_reset_check_core1() {
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
}

void core1_main() {
  assert(1 == get_core_num(), "core1_main not on core1");
  // Motion data reads take upwards of 4.5ms so we're doing them on core1
  init_i2c();

  MotionManager::manager().init();

  bool batterySucess = false;
#if HARDWARE_VERSION >= 4
  batterySucess = initializeBattery();
  powerState.batteryInitialized = batterySucess; // should be fine to write this bool from core1?
  logf("initializeBattery = %i", batterySucess);
  digitalWrite(DISABLE_CHARGE_PIN, false);
#endif

  static unsigned long lastBatteryPoll = 0;
  while (1) {
    while (!_gCore1DataGetNext) {
      hard_reset_check_core1();
      delayMicroseconds(100); // FIXME: i would rather do this with multicore fifo but cannot seem to get fifo to work at all
    }
    hard_reset_check_core1();
    MotionFrame motionFrame = MotionManager::manager().loop();
    BatteryData bd = {0};
    bool batteryDateUpdated = false;
#if HARDWARE_VERSION > 2
    if (lastBatteryPoll == 0 || millis() - lastBatteryPoll > 3000) {
      if (batterySucess) {
        bd = getBatteryData();
        bd.print();
      }
      batteryDateUpdated = true;
      lastBatteryPoll = millis();
    }
#endif

    mutex_enter_blocking(&core1DataLock);
    _gMotionFrame = motionFrame;
    if (batteryDateUpdated) _gBatteryData = bd;
    _gCore1DataGetNext = false;
    mutex_exit(&core1DataLock);
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
#endif

void startupCompleted() {
  logf("Startup completed");
  powerState.setRunning(true);
  indexedRunner->runPatternAtIndex(0);
}

/* ------ Setup ------------------------------------------------------------------------------------------------------------ */

void setup() {
  init_serial();

#if !DEBUG && !MEASURE_PHOTO_SENSOR_BASELINE
  // watchdog barks if we hang or hardfault
  watchdog_enable(8388 /* max value is 0xffffffu decremented twice per microsecond, roughly 8.3s */, true);
#endif

#if HARDWARE_VERSION >= 4
  // on v4, power-on happens by squeezing the unit, which often happens in a bag. 
  // if the device is squeezed to hard reset (10s), wait in a lower-power state until button-up before doing anything
  // this will also happen after crash/hang and after reprogramming, which is fine.
  if (watchdog_caused_reboot() && digitalRead(BUTTON_0) == BUTTON_PRESSED_STATE) {
    logf("Watchdog detected, button pressed. Sleeping until button up...");
    Serial.flush();
    attachInterrupt(digitalPinToInterrupt(BUTTON_0), buttonUpISR, (BUTTON_PRESSED_STATE == HIGH ? FALLING : RISING));
    set_sys_clock_khz(10*1000, false);
    do {
      __wfi();
    } while (!buttonWake);
    // it's likely that we actually powered off here. in case we didn't, startup normally.
    set_sys_clock_khz(133*1000, false);
    detachInterrupt(digitalPinToInterrupt(BUTTON_0));
    delay(100);
    init_serial();
    logf("Wake up, Neo");
    buttonWake = false;
  }

  // disable lipo charging so we can figure out if there is a battery
  pinMode(DISABLE_CHARGE_PIN, OUTPUT);
  digitalWrite(DISABLE_CHARGE_PIN, true);

#endif

  mutex_init(&core1DataLock);
#if !MEASURE_PHOTO_SENSOR_BASELINE
  multicore_launch_core1(core1_main);
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
  pinMode(CHRG_PIN, INPUT);
  pinMode(VBUS_SENSOR_PIN, INPUT_PULLDOWN);
  
  pinMode(GPOUT_PIN, INPUT_PULLUP);
#endif
#if HARDWARE_VERSION >= 4
  pinMode(EN_BOOST_PIN, OUTPUT);
  
  // if we booted this far, maintain our own power.
  gpio_set_function(EN_LDO_PIN, GPIO_FUNC_SIO);
  gpio_set_dir(EN_LDO_PIN, true);
  gpio_put(EN_LDO_PIN, true);

  int batteryVoltageRead = analogRead(BATTERY_VOLTAGE_PIN);
#else // HARDWARE_VERSION < 4
  powerState.setRunning(true);
#endif
#if HARDWARE_VERSION >= 5
  pinMode(V6_DETECTOR_PIN, INPUT);
  bool v6Hardware = (digitalRead(V6_DETECTOR_PIN) != 0);
  logdf("v6Hardware = %i", v6Hardware);
#endif

  FastLED.addLeds<SK9822HD, LED_SPI0_TX, LED_SPI0_SCK, BGR, DATA_RATE_MHZ(16)>(ctx.leds, LED_COUNT);//.setCorrection(0xFFB0C0);

#if DEBUG
  digitalWrite(LED_LINE_0_PWR_PIN, true);
  ctx.leds.fill_solid(CRGB::Red);
  FastLED.setBrightness(1);
  FastLED.show();
  FastLED.delay(10);
#endif

  patternManager.registerPattern<MotionHexa>();
  patternManager.registerPattern<TriBounce>();
  patternManager.registerPattern<PixelDust>();
  patternManager.registerPattern<PixelSand>();
  patternManager.registerPattern<LargeBouncyBall>();
  patternManager.registerPattern<PulseHexaSmooth>();
  patternManager.registerPattern<PridefulSpinnyThing>();
  patternManager.registerPattern<TriangleSpin>();
  patternManager.registerPattern<SparkleDroplets>();
  patternManager.registerPattern<BlobDroplets>();
  patternManager.registerPattern<SoundBits>();
  
#if HARDWARE_VERSION >= 3
  patternManager.registerPattern<ChargingPattern>(1);
  patternManager.setupConditionalRunner<ChargingPattern>([](PatternRunner &runner) -> uint8_t {
    return chargingPatternCheck(runner, powerState);
  }, 0xFD, 0xFF);
#endif
  
  indexedRunner = patternManager.setupIndexedRunner(0);
  
  mainButton = controls.addButton(BUTTON_0, BUTTON_PRESSED_STATE);
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
#if DEBUG_PHYSICS
  mainButton->onDoubleLongPress([]() {    
    physicsDebugFlag = !physicsDebugFlag;
  });
#endif
#if HARDWARE_VERSION >= 4
  mainButton->longPressInterval = 1000;
  mainButton->onLongPress([]() {
    logf("Long press! isHexaRunning = %i", powerState.isRunning());
    if (powerState.isRunning()) {
      bool usbPower = digitalRead(VBUS_SENSOR_PIN);
      // turn off
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
      }
    }
  });
#endif

  initLEDGraph();
  assert(ledgraph.adjList.size() == LED_COUNT, "adjlist size should match LED_COUNT");

  autoBrightness = new PhotoSensorBrightness(PHOTOSENSOR_READ_PIN, PHOTOSENSOR_POWER_PIN);
  autoBrightness->maxBrightness = 20; // needs to be lowish or we will overheat
  autoBrightness->logChanges = true;

#if MEASURE_PHOTO_SENSOR_BASELINE
  autoBrightness->measureBaseline(ctx.leds, 0x15, photosensorNearbyPixels, ARRAY_SIZE(photosensorNearbyPixels));
#endif

  patternManager.setup();

  // stream audio forever, since stopping and starting PDM introduces a noticeable hitch during pattern switching.
  // TODO: stop audio device when not in use by a pattern, but don't toggle twice between two audio patterns?
  audioInput.subscribe();

  setupDoneTime = millis();
  logf("setup done");
} 

/* ------ Loop ------------------------------------------------------------------------------------------------------------ */

void loop() {
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
  bool isButtonPressed = (digitalRead(BUTTON_0) == BUTTON_PRESSED_STATE);
  isVBUSPowered = digitalRead(VBUS_SENSOR_PIN);
#endif
#if HARDWARE_VERSION >= 4
  if (!powerState.isRunning()) {
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
    } else if (isButtonPressed && !patternManager.hasTestRunner()) {
      // we need to pause button events here since we don't know how many times it will be pressed and released before the animation is done
      mainButton->pauseEvents = true;
      powerOnOffRunner = patternManager.runOneShotPattern([](PatternRunner&) {
        return new PowerOnOffAnimation(true);
      }, 0xFF, 0xFF, [](PatternRunner&) {
        if (!powerState.isRunning()) { // we might have called it good early
          bool isButtonPressed = (digitalRead(BUTTON_0) == BUTTON_PRESSED_STATE);
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
    } else if (patternManager.hasTestRunner()) {
      powerState.setRunning(true);
    }
  }

  if (!isButtonPressed && !isVBUSPowered && !powerState.isRunning() && !powerOnOffRunner) {
    // unplugged USB while not drawing patterns or released button early during power on
    logf("No USB, no button, and no intent to run. Powering off...");
    powerOff();
    return;
  }
#endif

  getAsyncData(&MotionManager::motionFrame, &batteryData);
  powerState.update(isVBUSPowered, batteryData);

  // shared fft cache reset
  fftProcessing.frameReset();

  indexedRunner->paused = !powerState.isRunning();
  controls.update();
  patternManager.loop();
 
#if HARDWARE_VERSION > 1
  static bool pixelsHavePower = false;
  static unsigned long lastPixelsNeedPower = 0;
  bool pixelsNeedPower = ctx.leds;
  if (pixelsNeedPower) {
    lastPixelsNeedPower = millis();
  }
  if (pixelsNeedPower != pixelsHavePower 
    && (pixelsNeedPower || millis() - lastPixelsNeedPower > (fc.hasFPSAssertion() ? 10000 : 500))) { // don't turn off panel for very brief periods
    logf("Turn %s pixels", pixelsNeedPower?"on":"off");
    pixelsHavePower = pixelsNeedPower;
    digitalWrite(LED_LINE_0_PWR_PIN, pixelsNeedPower);
  }
#endif

#if DEBUG
#if HARDWARE_VERSION > 2
  ctx.leds[0] = isVBUSPowered ? CRGB::Red : CRGB::Black;
  ctx.leds[3] = isButtonPressed ? CRGB::Magenta : CRGB::Black;
#endif
  ctx.leds[1] = powerState.isRunning() ? CRGB::Green : CRGB::Black;
  ctx.leds[2] = powerState.isCharging() ? CRGB::Blue : CRGB::Black;
  ctx.leds[4] = powerState.batteryInitialized ? CRGB::Yellow : CRGB::Black;
  
  digitalWrite(LED_LINE_0_PWR_PIN, true);
#endif

#if AUTO_BRIGHTNESS
  // opportunistically grab photo reads when nearby pixels are off, otherwise they are too bright and impact the sensor
  // TODO: the effect of this is not great. it would be better to properly calibrate the brightness sensor with my baseline nearby pixel readings so we can adjust it constantly.
  int nearbyBrightness = 0;
  for (int i = 0; i < ARRAY_SIZE(photosensorNearbyPixels); ++i) {
    int px = photosensorNearbyPixels[i];
    int b = ctx.leds[px].r + 2*ctx.leds[px].g + 4 * ctx.leds[px].b;
    if (b > nearbyBrightness) {
      nearbyBrightness = b;
    }
  }
  if (nearbyBrightness < 6) {
    autoBrightness->loop();
  }
#else
  FastLED.setBrightness(kDefaultBrightness);
#endif

  if (pixelsHavePower || fc.hasFPSAssertion()) {
    FastLED.show();
  }

  fc.loop();
  fc.clampToFramerate(240);

  if (!pixelsNeedPower) {
    // FIXME: proper sleep
    fc.idleDelay(100);
  }
}
