#define DEBUG 0
#define WAIT_FOR_SERIAL 0

// for memory logging
#ifdef __arm__
extern "C" char* sbrk(int incr);
#else
extern char *__brkval;
#endif

#include "pico/multicore.h"

#include "Wire.h"
#define SDA 8
#define SCL 9

#include <Arduino.h>
#include <SPI.h>

#if HARDWARE_VERSION == 3

#define PDM_BCLK 23
#define PDM_LRCLK (BCLK+1)
#define PDM_DATA 25

#define UNCONNECTED_PIN_1 29

#define LED_SPI0_TX 19
#define LED_SPI0_SCK 22

#define PHOTOSENSOR_POWER_PIN 26
#define PHOTOSENSOR_READ_PIN 27

#define BUTTON_0 17

#define MOTION_INT_PIN 2
#define LED_LINE_0_PWR_PIN 3

#define PWR_SWITCH_PIN 28
#define VBUS_SENSOR_PIN 5

#define CHRG_PIN 16
#define GPOUT_PIN 0

#elif HARDWARE_VERSION == 2

#define I2S_BCLK 23
#define I2S_LRCLK (BCLK+1)
#define I2S_DATA 25

#define UNCONNECTED_PIN_1 26

#define LED_SPI0_TX 19
#define LED_SPI0_SCK 22

#define PHOTOSENSOR_POWER_PIN 27
#define PHOTOSENSOR_READ_PIN 28

#define BATTERY_VOLTAGE_PIN 29

#define BUTTON_0 16

#define MOTION_INT_PIN 2
#define LED_LINE_0_PWR_PIN 3

#else // first hardware rev

#define I2S_BCLK 1
#define I2S_LRCLK (BCLK+1)
#define I2S_DATA 3

#define UNCONNECTED_PIN_1 27

// These SPI pins are swapped from spec - FML but does not prevent FastLED from working
#define LED_SPI0_SCK 19
#define LED_SPI0_TX 18

#define PHOTOSENSOR_POWER_PIN 28
#define PHOTOSENSOR_READ_PIN 29

#define BUTTON_0 25
#endif

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
  void print() {
    logdf("soc: %u%%, soh: %u%%, voltage: %umV, capacity: %umAh / %umAh, power: %imW, temp: %uK", 
        stateOfCharge, stateOfHealth, voltage, currentCapacity, fullCapacity, powerDraw, temperature);
  }
};
const unsigned int BATTERY_CAPACITY = 2000;
BatteryData batteryData;

const int kFullCharge = 98; // %
bool isCharging;
unsigned long lastChargingStateChange; // ms
bool powerSwitchOn;
unsigned long lastPowerSwitchChange;   // ms
int knownChargePercent;                // 0 - 100%
unsigned long lastReachedFullCharge;   // ms

#include "patterns.h"

#include "MotionManager.h"

PhotoSensorBrightness *autoBrightness;

DrawingContext ctx;
HardwareControls controls;

FrameCounter fc;
PatternManager patternManager(ctx);

IndexedPatternRunner *indexedRunner; // main pattern runner

static bool serialTimeout = false;
static unsigned long setupDoneTime;


void init_i2c() {
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

void init_spi() {
    gpio_set_function(PIN_SPI0_MOSI, GPIO_FUNC_NULL);
    gpio_set_function(PIN_SPI0_SCK, GPIO_FUNC_NULL);

    spi_inst_t *spi = spi0;
    spi_init(spi, 8 * 1000000); // 16 MHz

    gpio_set_function(LED_SPI0_SCK, GPIO_FUNC_SPI);
    gpio_set_function(LED_SPI0_TX, GPIO_FUNC_SPI);
}

void init_serial() {
  Serial.begin(57600);
#if WAIT_FOR_SERIAL
  long setupStart = millis();
  while (!Serial) {
    if (millis() - setupStart > 10000) {
      serialTimeout = true;
      break;
    }
    delay(10);
  }
  delay(10); // Serial needs a bit more time before it'll actually log?
  logf("begin - waited %ims for Serial", millis() - setupStart);
#elif DEBUG
  delay(2000);
  Serial.println("Done waiting at boot.");
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
  data.stateOfCharge = lipo.soc();
  data.stateOfHealth = lipo.soh(PERCENT);
  data.voltage = lipo.voltage();
  data.currentCapacity = lipo.capacity(REMAIN);
  data.fullCapacity = lipo.capacity(FULL);
  data.powerDraw = lipo.power();
  data.temperature = lipo.temperature(INTERNAL_TEMP)/10.;
#endif
  return data;
}

mutex_t core1DataLock;
ICM_20948_AGMT_t _gAGMT;   // locked AGMT read
BatteryData _gBatteryData; // locked BatteryData read
bool _gCore1DataGetNext = true; // prevent core1 from doing multiple motion reads in a single frame

void getAsyncData(ICM_20948_AGMT_t *agmtRef, BatteryData *batteryDataRef) {
  assert(0 == get_core_num(), "getAGMT not on core0");
  mutex_enter_blocking(&core1DataLock);
  ICM_20948_AGMT_t agmt = _gAGMT;
  BatteryData batteryData = _gBatteryData;
  _gCore1DataGetNext = true;
  mutex_exit(&core1DataLock);

  if (agmtRef) *agmtRef = agmt;
  if (batteryDataRef) *batteryDataRef = batteryData;
}

void core1_main() {
  // Motion data reads take upwards of 4.5ms so we're doing them on core1
  init_i2c();

#if HARDWARE_VERSION > 2
  // if (!lipo.begin(SDA, SCL)) {
  //   logf("Error: Unable to communicate with BQ27427.");
  // } else {
  //   logf("Connected to BQ27427!");
  // }
  // logf("device type = %i", lipo.deviceType());
  // NOTE: I am seeing my BQ27427 say lipo.deviceType is 0x427, though this library expects 0x0421
  // i2c is already initialized anyway.

  // FIXME: //   // set_lipo_config took 3111852us
  // not sure why this is taking 3 seconds. probably hitting a watchdog from repeated writes.
  //
  // TIMEIT(set_lipo_config, {
  // lipo.enterConfig(true);
  // lipo.setCapacity(BATTERY_CAPACITY);
  // lipo.setGPOUTPolarity(LOW); // Set GPOUT to active-low
  // lipo.setGPOUTFunction(BAT_LOW);
  // lipo.exitConfig(true);
  // });
#endif

  MotionManager::manager().init();
  static unsigned long lastBatteryPoll = 0;
  while (1) {
    while (!_gCore1DataGetNext) {
      delayMicroseconds(100); // FIXME: i would rather do this with multicore fifo but cannot seem to get fifo to work at all
    }
    ICM_20948_AGMT_t agmt = MotionManager::manager().loop();
    
    BatteryData batteryData;
    if (millis() - lastBatteryPoll > 3000) {
      batteryData = getBatteryData();
      batteryData.print();
      lastBatteryPoll = millis();
    }

    mutex_enter_blocking(&core1DataLock);
    _gAGMT = agmt;
    _gBatteryData = batteryData;
    _gCore1DataGetNext = false;
    mutex_exit(&core1DataLock);
  }
}

void setup() {
  init_serial();

  mutex_init(&core1DataLock);
  multicore_launch_core1(core1_main);

  pinMode(UNCONNECTED_PIN_1, INPUT);
  auto noise = lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint32_t));
  randomSeed(lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint32_t)));
  random16_add_entropy(lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint16_t)));

#if HARDWARE_VERSION > 1
  pinMode(LED_LINE_0_PWR_PIN, true);
#endif

#if defined(I2S_BCLK)
  init_i2s();
#endif
#if defined(PDM_BCLK)
  init_pdm();
#endif
  init_spi();

#if HARDWARE_VERSION > 2
  pinMode(PWR_SWITCH_PIN, INPUT_PULLDOWN);
  pinMode(CHRG_PIN, INPUT);
  pinMode(VBUS_SENSOR_PIN, INPUT_PULLDOWN);
  pinMode(GPOUT_PIN, INPUT_PULLUP);
#endif
  
  FastLED.addLeds<SK9822HD, LED_SPI0_TX, LED_SPI0_SCK, BGR, DATA_RATE_MHZ(16)>(ctx.leds, LED_COUNT);//.setCorrection(0xFFB0C0);

  patternManager.registerPattern<MotionHexa>();
  patternManager.registerPattern<TriBounce>();
  patternManager.registerPattern<PixelDust>();
  patternManager.registerPattern<PulseHexa>();

#if HARDWARE_VERSION > 2
  patternManager.registerPattern<ChargingPattern>(1);
  patternManager.setupConditionalRunner<ChargingPattern>([](PatternRunner &runner) -> uint8_t {
    const int kChargePatternOverlayDuration = 1500; // how long to show charge pattern while pwrSwitchOn
    const int kFadeTime = 300;
    const int kSitTimeAtFullCharge = 5000;
    const int kRecentStateChangeDelay = 200;
    
    // logf("isCharging=%i, powerSwitchOn=%i, hasPattern = %i", isCharging, powerSwitchOn, (bool)(runner.pattern));

    uint8_t chargeAlpha = 0;

    if (isCharging) {
      if (runner.pattern) {
        int runTime = runner.pattern->runTime();
        if (powerSwitchOn && runTime > kChargePatternOverlayDuration) {
          // fade down overlay while device on
          // logf("fade down overlay device on");
          chargeAlpha = max(0L, 0xFF - 0xFF * (long)(millis() - max(lastPowerSwitchChange, lastChargingStateChange)) / kFadeTime);
        } else if (lastReachedFullCharge && millis() - lastReachedFullCharge > kSitTimeAtFullCharge && runTime > kChargePatternOverlayDuration) {
          // fade down charging ui when fully charged
          // logf("fade down fully charged");
          chargeAlpha = max(0L, 0xFF - 0xFF * (long)(millis() - lastReachedFullCharge + kSitTimeAtFullCharge) / kFadeTime);
        } else if (runTime < kFadeTime) {
          // fade up overlay
          // logf("fade up overlay");
          chargeAlpha = min(0xFFL, (long)0xFF * runTime / kFadeTime);
        } else {
          // run overlay
          chargeAlpha = 0xFF;
        }
      } else if ((!powerSwitchOn && lastReachedFullCharge == 0) 
              || millis() - lastChargingStateChange < kRecentStateChangeDelay 
              || (millis() - lastPowerSwitchChange < kRecentStateChangeDelay && lastReachedFullCharge == 0)) {
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
  
  // patternManager.registerPattern<LineSweep>();
  // patternManager.registerPattern<RandomDust>();
  
  indexedRunner = patternManager.setupIndexedRunner(0);
  
  SPSTButton *button = controls.addButton(BUTTON_0);
  button->onSinglePress([]() {
    indexedRunner->nextPattern();
  });
  button->onDoublePress([]() {
    indexedRunner->previousPattern();
  });
  button->onLongPress([]() {
    // hard reset
    logf("hard reset!");
    Serial.flush();
    watchdog_reboot(0,0,100);
  });

  // patternManager.setTestPattern<MotionHexa>();
  // patternManager.setTestPattern<LineSweep>();
  
  initLEDGraph();
  assert(ledgraph.adjList.size() == LED_COUNT, "adjlist size should match LED_COUNT");

  patternManager.setup();

  autoBrightness = new PhotoSensorBrightness(PHOTOSENSOR_READ_PIN, PHOTOSENSOR_POWER_PIN);
  autoBrightness->maxBrightness = 0x10; // needs to be lowish on usb bc v2 lipo charger cuts out at 1A draw
  autoBrightness->logChanges = true;

  setupDoneTime = millis();
  logf("setup done");
} 

void startupWelcome() {
  int welcomeDuration = 333;

  ctx.leds.fill_solid(CRGB::Black);
  gpio_put(LED_LINE_0_PWR_PIN, true);

  HexaShells hexaShells;
  uint8_t hue = random8();
  DrawModal(120, welcomeDuration, [hue, welcomeDuration, hexaShells](unsigned long elapsed) {
    const int fadeRings = 6;
    FastLED.setBrightness(3);
      int maxShell = (hexaShells.shells.size() + fadeRings/2) * elapsed / welcomeDuration;
      for (int s = 0; s < min(maxShell, hexaShells.shells.size()); ++s) {
        for (int px : hexaShells.shells[s]) {
          uint8_t b = (s <= maxShell && s > maxShell - fadeRings) ? triwave8(0xFF * (maxShell - s+1) / fadeRings) : 0;
          ctx.leds[px] = CHSV(hue, b, b);
        }
      }
   });
  ctx.leds.fill_solid(CRGB::Black);
  FastLED.show();
}

void loop() {
  if (serialTimeout && millis() - setupDoneTime < 1000) {
    serialTimeoutIndicator();
    return;
  }

#if HARDWARE_VERSION > 2
  static unsigned long lastLipoChargeIndicator = 0;
  // The charge indicator read from the lipo charger is too noisy/bouncy to use for charging UI. 
  // Instead we'll check if VBUS is powered and that the lipo charger has indicated it's charging anytime within the last 10 seconds.
  bool isVBUSPowered = digitalRead(VBUS_SENSOR_PIN);
  bool isLipoCharging = !digitalRead(CHRG_PIN);
  if (isLipoCharging) {
    lastLipoChargeIndicator = millis();
  }
  bool isChargingRead = isVBUSPowered && (millis() - lastLipoChargeIndicator < 10000);
  
  if (isCharging != isChargingRead) {
    logdf("Charging State Change %i -> %i", isCharging, isChargingRead);
    isCharging = isChargingRead;
    lastChargingStateChange = millis();
  }
  
  bool powerSwitchOnRead = !digitalRead(PWR_SWITCH_PIN);
  if (powerSwitchOn != powerSwitchOnRead) {
    logdf("PowerSwitch State Change %i -> %i", powerSwitchOn, powerSwitchOnRead);
    powerSwitchOn = powerSwitchOnRead;
    lastPowerSwitchChange = millis();
    if (powerSwitchOn) {
      indexedRunner->runPatternAtIndex(0);
      if (indexedRunner->pattern) indexedRunner->pattern->setAlpha(0, false);
    } else {
      // FIXME: fadedown
      indexedRunner->stop();
    }
  }
#endif

  static bool firstLoop = true;
  if (firstLoop) {
    startupWelcome();
    firstLoop = false;
  }

  getAsyncData(&MotionManager::agmt, &batteryData);
  if (batteryData.stateOfCharge >= kFullCharge && knownChargePercent < kFullCharge) {
    logdf("Reached Full Charge!");
    lastReachedFullCharge = millis();
  } else if (batteryData.stateOfCharge < kFullCharge-1) {
    lastReachedFullCharge = 0;
  }
  knownChargePercent = batteryData.stateOfCharge;

  indexedRunner->paused = !powerSwitchOn;
  patternManager.loop();
  controls.update();

  // FIXME: we might need to opportunistically grab photo reads when nearby pixels are off, otherwise they are too bright
  FastLED.setBrightness(15);
  // autoBrightness->loop();
 
  bool pixelsNeedPower = ctx.leds(0, LED_COUNT-1);
#if HARDWARE_VERSION > 1
  static bool pixelsHavePower = false;
  if (pixelsNeedPower != pixelsHavePower) {
    logdf("Turn %s line 0", pixelsNeedPower?"on":"off");
    pixelsHavePower = pixelsNeedPower;
    gpio_put(LED_LINE_0_PWR_PIN, pixelsNeedPower);
  }
#endif
  if (pixelsNeedPower) {
    FastLED.show();
  }

  fc.loop();
  fc.clampToFramerate(240);

  if (!isCharging && !powerSwitchOn && !pixelsNeedPower) {
    // FIXME: proper sleep
    FastLED.delay(200);
  }
}
