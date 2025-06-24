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

#include <I2S.h>
I2S i2s(INPUT);
#if HARDWARE_VERSION > 1
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

#include "patterns.h"


#include "MotionManager.h"

PhotoSensorBrightness *autoBrightness;

DrawingContext ctx;
HardwareControls controls;

FrameCounter fc;
PatternManager patternManager(ctx);

static bool serialTimeout = false;
static unsigned long setupDoneTime;

void init_i2c() {
  Wire.setSDA(SDA);
  Wire.setSCL(SCL);
  Wire.begin();
}

void init_i2s() {
  i2s.setBCLK(I2S_BCLK);
  i2s.setDATA(I2S_DATA);
  i2s.setBitsPerSample(32);
  i2s.setFrequency(16000);
  assert(i2s.begin(),"i2s");
}

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
  FastLED.show();
  delay(20);
}

mutex_t agmtLock;
ICM_20948_AGMT_t _gAGMT;   // locked AGMT read
bool _gAGMTGetNext = true; // prevent core1 from doing multiple motion reads in a single frame

ICM_20948_AGMT_t getAGMT() {
  assert(0 == get_core_num(), "getAGMT not on core0");
  mutex_enter_blocking(&agmtLock);
  ICM_20948_AGMT_t agmt = _gAGMT;
  _gAGMTGetNext = true;
  mutex_exit(&agmtLock);
  return agmt;
}

void core1_main() {
  // Motion data reads take upwards of 4.5ms so we're doing them on core1
  init_i2c();
  MotionManager::manager().init();
  while (1) {
    while (!_gAGMTGetNext) {
      delayMicroseconds(100); // FIXME: i would rather do this with multicore fifo but cannot seem to get fifo to work at all
    }
    ICM_20948_AGMT_t agmt = MotionManager::manager().loop();

    mutex_enter_blocking(&agmtLock);
    _gAGMT = agmt;
    _gAGMTGetNext = false;
    mutex_exit(&agmtLock);
  }
}

void setup() {
  init_serial();

  mutex_init(&agmtLock);
  multicore_launch_core1(core1_main);

  pinMode(UNCONNECTED_PIN_1, INPUT);
  auto noise = lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint32_t));
  randomSeed(lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint32_t)));
  random16_add_entropy(lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint16_t)));

#if HARDWARE_VERSION > 1
  pinMode(LED_LINE_0_PWR_PIN, true);
#endif

  init_i2s();
  init_spi();
  
  FastLED.addLeds<SK9822HD, LED_SPI0_TX, LED_SPI0_SCK, BGR, DATA_RATE_MHZ(16)>(ctx.leds, LED_COUNT);//.setCorrection(0xFFB0C0);

  patternManager.registerPattern<PulseHexa>();
  patternManager.registerPattern<MotionHexa>();
  patternManager.registerPattern<TriBounce>();
  
  patternManager.registerPattern<PixelDust>();

  // patternManager.registerPattern<LineSweep>();
  // patternManager.registerPattern<RandomDust>();

  
  IndexedPatternRunner *indexedRunner = patternManager.setupIndexedRunner(1);
  SPSTButton *button = controls.addButton(BUTTON_0);
  button->onSinglePress([indexedRunner]() {
    indexedRunner->nextPattern();
  });
  button->onDoublePress([indexedRunner]() {
    indexedRunner->previousPattern();
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

  static bool firstLoop = true;
  if (firstLoop) {
    startupWelcome();
    firstLoop = false;
  }
  MotionManager::agmt = getAGMT();
  
  patternManager.loop();
  controls.update();
  autoBrightness->loop();
 
  // EVERY_N_MILLIS(1000) {
  //   int avg = 0;
  //   for (int i = 0 ; i < 100; ++i) {
  //     int batteryRead = analogRead(BATTERY_VOLTAGE_PIN);
  //     avg = (i * avg + batteryRead) / (i + 1);
  //   }
  //   logf("batteryRead = %i", avg);
  // }

  static bool pixelsHavePower = false;
  bool pixelsNeedPower = ctx.leds(0, LED_COUNT-1);
  
#if HARDWARE_VERSION > 1  
  if (pixelsNeedPower != pixelsHavePower) {
    // logf("turn %s line 0", line0poweron?"on":"off");
    pixelsHavePower = pixelsNeedPower;
    gpio_put(LED_LINE_0_PWR_PIN, pixelsNeedPower);
  }
#endif
  if (pixelsNeedPower) {
    FastLED.show();
  }

  fc.loop();
  fc.clampToFramerate(240);
  
}
