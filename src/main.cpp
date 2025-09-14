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

#if defined(I2S_BCLK)
#include <I2S.h>
I2S i2s(INPUT);
#endif

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

#include <BQ27427.h>
struct BatteryData {
  uint16_t stateOfCharge;   // %
  uint16_t stateOfHealth;   // %
  uint16_t voltage;         // mV
  uint16_t currentCapacity; // mAh
  uint16_t fullCapacity;    // mAh
  int16_t powerDraw;        // mW
  uint16_t temperature;     // K
  uint16_t flags;
  void print() {
    logf("battery: %s, soc: %u%%, soh: %u%%, voltage: %umV, capacity: %umAh / %umAh, power: %imW, temp: %uK, flags: %X",
      batteryDetected()?"yes":"no", stateOfCharge, stateOfHealth, voltage, currentCapacity, fullCapacity, powerDraw, temperature, flags);
  }
  bool batteryDetected() {
    // Issue: almost always detects a battery, presumably mistaking the lipo charger, powering the load, as a battery
    //   but if the charger is off then the i2c reads will be all 1s, SoC will be 0xFFFF. 
    //   if charger is on then voltage will be > max battery voltage, so we can do guesswork.
    return stateOfCharge <= 100 && voltage < 4350 && flags & 1<<3;
  }
};
const unsigned int BATTERY_CAPACITY = 2000;
BatteryData batteryData = {0};

class RunState {
  bool running;
  unsigned long lastChange;
public:
  bool isRunning() {
    return running;
  }
  void setRunning(bool isRunning) {
    running = isRunning;
    lastChange = millis();
  }
  unsigned long lastRunStateChange() {
    return lastChange;
  }
};

RunState runState;

const int kFullCharge = 98; // %
bool isCharging;
unsigned long lastChargingStateChange; // ms
bool powerSwitchOn = true;
unsigned long lastPowerSwitchChange;   // ms
int knownChargePercent;                // 0 - 100%
unsigned long lastReachedFullCharge;   // ms

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

#if defined(I2S_BCLK)
void init_i2s() {
  i2s.setBCLK(I2S_BCLK);
  i2s.setDATA(I2S_DATA);
  i2s.setBitsPerSample(32);
  i2s.setFrequency(16000);
  assert(i2s.begin(),"i2s");
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
  delay(10); // Serial needs a bit more time before it'll actually log?
  logf("begin - waited %ims for Serial", millis() - setupStart);
#elif DEBUG
  // delay(2000); // FIXME: put back debug delay?
  // Serial.println("Done waiting at boot.");
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

BatteryData getBatteryData()
{
  assert(1 == get_core_num(), "getBatteryData not on core1");
  BatteryData data;
#if HARDWARE_VERSION > 2
  // TODO: fetch only the data items we'll actually use, for perf
  data.stateOfCharge = lipo.soc(FILTERED);
  data.stateOfHealth = lipo.soh(PERCENT);
  data.voltage = lipo.voltage();
  data.currentCapacity = lipo.capacity(REMAIN);
  data.fullCapacity = lipo.capacity(FULL);
  data.powerDraw = lipo.power();
  data.temperature = lipo.temperature(INTERNAL_TEMP)/10.;
  data.flags = lipo.flags();
#endif
  return data;
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

#if HARDWARE_VERSION > 2
  // logf("device type = %i", lipo.deviceType());
  // NOTE: I am seeing my BQ27427 say lipo.deviceType is 0x427, though this library expects 0x0421..?
  lipo.enterConfig(true);
  lipo.setCapacity(BATTERY_CAPACITY);
  lipo.setChemID(CHEM_B);
  lipo.exitConfig(true);
#endif

  MotionManager::manager().init();
  static unsigned long lastBatteryPoll = 0;
  while (1) {
    while (!_gCore1DataGetNext) {
      hard_reset_check_core1();
      delayMicroseconds(100); // FIXME: i would rather do this with multicore fifo but cannot seem to get fifo to work at all
    }
    hard_reset_check_core1();
    MotionFrame motionFrame = MotionManager::manager().loop();
#if HARDWARE_VERSION > 2
    BatteryData batteryData;
    if (lastBatteryPoll == 0 || millis() - lastBatteryPoll > 3000) {
      batteryData = getBatteryData();
      batteryData.print();
      lastBatteryPoll = millis();
    }
#endif

    mutex_enter_blocking(&core1DataLock);
    _gMotionFrame = motionFrame;
    _gBatteryData = batteryData;
    _gCore1DataGetNext = false;
    mutex_exit(&core1DataLock);
  }
}

#if HARDWARE_VERSION >= 4
void powerOff() {
  // gpio_set_pulls(EN_LDO_PIN, false, false); // stop pulling up // FIXME: needed?
  // FIXME: there seems to be behavior difference between gpio_put here and digitalWrite?
  gpio_put(EN_LDO_PIN, false);
  // digitalWrite(EN_LDO_PIN, false);
  // should power off here, but delay a little since sometimes we bounce back on
  delay(500);
}
#endif

void setup() {
  init_serial();

#if !DEBUG
  // watchdog barks if we hang or hardfault
  watchdog_enable(8388 /* max value is 0xffffffu decremented twice per microsecond, roughly 8.3s */, true);
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
  pinMode(PWR_SWITCH_PIN, INPUT_PULLDOWN);
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

#if defined(I2S_BCLK)
  init_i2s();
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
    const int kChargePatternOverlayDuration = 1500; // how long to show charge pattern while pwrSwitchOn
    const int kFadeTime = 300;
    const int kSitTimeAtFullCharge = 5000;
    const int kRecentStateChangeDelay = 200;
    
    // logf("isCharging=%i, isHexaRunning=%i, hasPattern = %i, lastReachedFullCharge= %i, lastChargingStateChange= %i, millis=%i", isCharging, runState.isRunning(), (bool)(runner.pattern), lastReachedFullCharge, lastChargingStateChange, millis());

    uint8_t chargeAlpha = 0;

    if (isCharging) {
      if (runner.pattern) {
        int runTime = runner.pattern->runTime();
        if (runState.isRunning() && runTime > kChargePatternOverlayDuration) {
          // fade down overlay while device on
          // logf("fade down overlay device on");
          chargeAlpha = max(0L, 0xFF - 0xFF * (long)(millis() - max(runState.lastRunStateChange(), lastChargingStateChange)) / kFadeTime);
        } else if (lastReachedFullCharge && millis() - lastReachedFullCharge > kSitTimeAtFullCharge && runTime > kChargePatternOverlayDuration) {
          // fade down charging ui when fully charged
          // logf("fade down fully charged");
          chargeAlpha = max(0L, 0xFF - 0xFF * (long)(millis() - lastReachedFullCharge - kSitTimeAtFullCharge) / kFadeTime);
        } else if (runTime < kFadeTime) {
          // fade up overlay
          // logf("fade up overlay");
          chargeAlpha = min(0xFFL, (long)0xFF * runTime / kFadeTime);
        } else {
          // run overlay
          chargeAlpha = 0xFF;
        }
      } else if ((!runState.isRunning() && lastReachedFullCharge == 0) 
              || millis() - lastChargingStateChange < kRecentStateChangeDelay 
              || (millis() - runState.lastRunStateChange() < kRecentStateChangeDelay && lastReachedFullCharge == 0)) {
        // start overlay
        // logf("start overlay");
        chargeAlpha = 0x1;
      }
    } else {
      if (runner.pattern) {
        // fadedown overlay due to not charging
        // logf("fade down overlay not charging");
        chargeAlpha = max(0L, 0xFF - 0xFF * (long)(millis() - lastChargingStateChange) / kFadeTime);
      }
    }
    // if (chargeAlpha) {
    //   logf("  charge overlay alpha = %02X", chargeAlpha);
    // }
    return chargeAlpha;
  }, 0xFF, 0xFF);
#endif
  
  indexedRunner = patternManager.setupIndexedRunner(0);
  
  SPSTButton *button = controls.addButton(BUTTON_0, BUTTON_PRESSED_STATE);
  button->ignoreEventsUntilFirstButtonUp = true;
  button->onSinglePress([]() {
    if (runState.isRunning()) {
      indexedRunner->nextPattern();
    }
  });
  button->onDoublePress([]() {
    if (runState.isRunning()) {
      indexedRunner->previousPattern();
    }
  });
#if HARDWARE_VERSION >= 4
  button->longPressInterval = 1000;
  button->onLongPress([]() {
    logf("Long press! isHexaRunning = %i", runState.isRunning());
    if (runState.isRunning()) {
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
              runState.setRunning(false);
            } else {
              powerOff();
              while (digitalRead(BUTTON_0) == BUTTON_PRESSED_STATE) {
                // handle the button being left pressed after power off
                delay(50);
              }
            }
          });
        }
      } else {
        runState.setRunning(false);
        indexedRunner->stop();
      }
    } else { // turn on
      runState.setRunning(true);
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

void powerStateLoop() {
  #if HARDWARE_VERSION > 2
  bool isButtonPressed = (digitalRead(BUTTON_0) == BUTTON_PRESSED_STATE);
  bool isVBUSPowered = digitalRead(VBUS_SENSOR_PIN);
#endif

#if HARDWARE_VERSION >= 4
  static bool didButtonBoot = false;
  if (!didButtonBoot && millis() > kBootDelay) {
    didButtonBoot = true;
    runState.setRunning(isButtonPressed && !isVBUSPowered); // draw patterns if we weren't powered up by usb
    if (!isButtonPressed && !isVBUSPowered) {
      // powered on via button, released prior to threshold
      logf("Startup interrupted. Powering off...");
      powerOff();
      return;
    }
  }
#endif
 
#if HARDWARE_VERSION >= 3
  static unsigned long lastLipoChargeIndicator = 0;
  // The charge indicator read from the lipo charger is too noisy/bouncy to use for charging UI. 
  // Instead we'll check if VBUS is powered and that the lipo charger has indicated it's charging anytime within the last 10 seconds.
#if HARDWARE_VERSION >= 4
  if (!isButtonPressed && !isVBUSPowered && !runState.isRunning()) {
    // likely unplugged USB while hexa not running
    logf("No USB, no button, and no intent to run. Powering off...");
    powerOff();
    return;
  }
#endif
  bool isLipoCharging = !digitalRead(CHRG_PIN);
  if (isLipoCharging) {
    lastLipoChargeIndicator = millis();
  }
  bool isChargingRead = isVBUSPowered && batteryData.batteryDetected();
  
  if (isCharging != isChargingRead) {
    logdf("Charging State Change %i -> %i, millis since last lipo change = %i", isCharging, isChargingRead, millis() - lastLipoChargeIndicator);
    isCharging = isChargingRead;
    lastChargingStateChange = millis();
  }
#endif
  
  getAsyncData(&MotionManager::motionFrame, &batteryData);

  if (batteryData.stateOfCharge >= kFullCharge && knownChargePercent < kFullCharge) {
    logdf("Reached Full Charge!");
    lastReachedFullCharge = millis();
  } else if (batteryData.stateOfCharge < kFullCharge-1) {
    lastReachedFullCharge = 0;
  }
  knownChargePercent = batteryData.stateOfCharge;
}

void loop() {
#if !DEBUG
  // pet the dog
  watchdog_update();
#endif

  if (serialTimeout && millis() - setupDoneTime < 1000) {
    serialTimeoutIndicator();
    return;
  }

  powerStateLoop();
  indexedRunner->paused = !runState.isRunning();
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
  ctx.leds[1] = runState.isRunning() ? CRGB::Green : CRGB::Black;
  ctx.leds[2] = isCharging ? CRGB::Blue : CRGB::Black;
  
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
