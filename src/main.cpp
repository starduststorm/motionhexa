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

#if defined(PDM_BCLK)
#include <PDM.h>
#endif

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

PhotoSensorBrightness *autoBrightness;

DrawingContext ctx;
HardwareControls controls;

FrameCounter fc;
PatternManager patternManager(ctx);

#include "patterns.h"

IndexedPatternRunner *indexedRunner; // main pattern runner

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

#if defined(PDM_BCLK)
void init_pdm() {
    PDM.setCLK(PDM_BCLK);
    PDM.setDIN(PDM_DATA);
    assert(1 == PDM.begin(1, 16384), "Failed to initialize PDM device");
}
#endif

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

  initializeBattery();

  MotionManager::manager().init();
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
      bd = getBatteryData();
      bd.print();
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

/* ------ Setup ------------------------------------------------------------------------------------------------------------ */

void setup() {
  init_serial();

#if !DEBUG
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
#endif

  mutex_init(&core1DataLock);
  multicore_launch_core1(core1_main);

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
  pinMode(DISABLE_CHARGE_PIN, OUTPUT);
  pinMode(EN_BOOST_PIN, OUTPUT);
  
  // if we booted this far, maintain our own power.
  gpio_set_function(EN_LDO_PIN, GPIO_FUNC_SIO);
  gpio_set_pulls(EN_LDO_PIN, true, false); // pull up
  gpio_set_drive_strength(EN_LDO_PIN, GPIO_DRIVE_STRENGTH_4MA);
  gpio_set_dir(EN_LDO_PIN, true);
  gpio_put(EN_LDO_PIN, true);

  int batteryVoltageRead = analogRead(BATTERY_VOLTAGE_PIN);
#else // HARDWARE_VERSION < 4
  runState.setRunning(true);
#endif

  FastLED.addLeds<SK9822HD, LED_SPI0_TX, LED_SPI0_SCK, BGR, DATA_RATE_MHZ(16)>(ctx.leds, LED_COUNT);//.setCorrection(0xFFB0C0);

#if DEBUG
  digitalWrite(LED_LINE_0_PWR_PIN, true);
  ctx.leds.fill_solid(CRGB::Red);
  FastLED.setBrightness(1);
  FastLED.show();
  FastLED.delay(10);
#endif

#if defined(PDM_BCLK)
  init_pdm();
#endif

  patternManager.registerPattern<MotionHexa>();
  patternManager.registerPattern<TriBounce>();
  patternManager.registerPattern<PixelDust>();
  patternManager.registerPattern<PixelSand>();
  patternManager.registerPattern<PulseHexaSmooth>();
  patternManager.registerPattern<PulseHexa>();  
  patternManager.registerPattern<LineTest>();
  patternManager.registerPattern<TriangleSpin>();
  
  // patternManager.setTestRunner<TriangleSpin>();
  
#if HARDWARE_VERSION >= 3
  patternManager.registerPattern<ChargingPattern>(1);
  patternManager.setupConditionalRunner<ChargingPattern>([](PatternRunner &runner) -> uint8_t {
    return chargingPatternCheck(runner, powerState);
  }, 0xFF, 0xFF);
#endif
  
  indexedRunner = patternManager.setupIndexedRunner(0);
  
  SPSTButton *button = controls.addButton(BUTTON_0, BUTTON_PRESSED_STATE);
  button->ignoreEventsUntilFirstButtonUp = true;
  button->onSinglePress([]() {
    if (powerState.isRunning()) {
      indexedRunner->nextPattern();
    }
  });
  button->onDoublePress([]() {
    if (powerState.isRunning()) {
      indexedRunner->previousPattern();
    }
  });
#if HARDWARE_VERSION >= 4
  button->longPressInterval = 1000;
  button->onLongPress([]() {
    logf("Long press! isHexaRunning = %i", powerState.isRunning());
    if (powerState.isRunning()) {
      bool usbPower = digitalRead(VBUS_SENSOR_PIN);
      if (!usbPower) {
        // power down
        if (patternManager.hasTestRunner()) {
          // special case test runner since the power off animation will not run
          powerOff();
        } else {
          patternManager.runOneShotPattern<PowerOffAnimation>(0xFF, 0xFF, [](PatternRunner&) {
            indexedRunner->stop();
            bool usbPower = digitalRead(VBUS_SENSOR_PIN);
            if (usbPower) {
              powerState.setRunning(false);
              indexedRunner->stop();
            } else {
              powerOff();
              while (digitalRead(BUTTON_0) == BUTTON_PRESSED_STATE) {
                // handle the button being left pressed after power off
                delay(50);
              }
            }
          });
        }
      } else { // usb power, so just stop the main runner
        powerState.setRunning(false);
        indexedRunner->stop();
      }
    } else { // turn on
      powerState.setRunning(true);
      indexedRunner->runPatternAtIndex(0);
    }
  });
#endif

  initLEDGraph();
  assert(ledgraph.adjList.size() == LED_COUNT, "adjlist size should match LED_COUNT");

  patternManager.setup();

  autoBrightness = new PhotoSensorBrightness(PHOTOSENSOR_READ_PIN, PHOTOSENSOR_POWER_PIN);
  autoBrightness->maxBrightness = 0x10; // needs to be lowish on usb bc v2 lipo charger cuts out at 1A draw
  autoBrightness->logChanges = true;

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

#if HARDWARE_VERSION > 2
  bool isButtonPressed = (digitalRead(BUTTON_0) == BUTTON_PRESSED_STATE);
  bool isVBUSPowered = digitalRead(VBUS_SENSOR_PIN);
#endif
#if HARDWARE_VERSION >= 4
  static bool didButtonBoot = false;
  if (!didButtonBoot && millis() > kBootDelay) {
    didButtonBoot = true;
    powerState.setRunning(isButtonPressed && !isVBUSPowered); // draw patterns if we weren't powered up by usb
    if (!isButtonPressed && !isVBUSPowered) {
      // powered on via button, released prior to threshold
      logf("Startup interrupted. Powering off...");
      powerOff();
      return;
    }
  }

  if (!isButtonPressed && !isVBUSPowered && !powerState.isRunning()) {
    // likely unplugged USB while not drawing patterns
    logf("No USB, no button, and no intent to run. Powering off...");
    powerOff();
    return;
  }
#endif

  getAsyncData(&MotionManager::motionFrame, &batteryData);
  powerState.update(isVBUSPowered, batteryData);

  indexedRunner->paused = !powerState.isRunning();
  controls.update();
  patternManager.loop();

  // FIXME: we might need to opportunistically grab photo reads when nearby pixels are off, otherwise they are too bright
  FastLED.setBrightness(15);
  // autoBrightness->loop();
 
#if HARDWARE_VERSION > 1
  static bool pixelsHavePower = false;
  static unsigned long lastPixelsNeedPower = 0;
  bool pixelsNeedPower = ctx.leds(0, LED_COUNT-1);
  if (pixelsNeedPower) {
    lastPixelsNeedPower = millis();
  }
  if (pixelsNeedPower != pixelsHavePower 
    && (pixelsNeedPower || millis() - lastPixelsNeedPower > 300)) { // don't turn off panel for very brief periods
    logdf("Turn %s pixels", pixelsNeedPower?"on":"off");
    pixelsHavePower = pixelsNeedPower;
    digitalWrite(LED_LINE_0_PWR_PIN, pixelsNeedPower);
  }
#endif
  
#if DEBUG
#if HARDWARE_VERSION > 2
  ctx.leds[0] = digitalRead(VBUS_SENSOR_PIN) ? CRGB::Red : CRGB::Black;
#endif
  ctx.leds[1] = powerState.isRunning() ? CRGB::Green : CRGB::Black;
  ctx.leds[2] = powerState.isCharging() ? CRGB::Blue : CRGB::Black;
  
  digitalWrite(LED_LINE_0_PWR_PIN, true);
#endif

  if (pixelsNeedPower) {
    FastLED.show();
  }

  fc.loop();
  fc.clampToFramerate(240);

  if (!pixelsNeedPower) {
    // FIXME: proper sleep
    FastLED.delay(100);
  }
}
