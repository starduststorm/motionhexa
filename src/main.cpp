#define DEBUG 1

// for memory logging
#ifdef __arm__
extern "C" char* sbrk(int incr);
#else
extern char *__brkval;
#endif

#include "Wire.h"
#define SDA 8
#define SCL 9

#include <I2S.h>
I2S i2s(INPUT);
#define I2S_BCLK 1
#define I2S_LRCLK (BCLK+1)
#define I2S_DATA 3

#define UNCONNECTED_PIN_1 27

#include <Arduino.h>
#include <SPI.h>

// These SPI pins are swapped from spec - FML but does not prevent FastLED from working
#define LED_SPI0_SCK 19
#define LED_SPI0_TX 18

#define FASTLED_USE_PROGMEM 1
#define FASTLED_USE_GLOBAL_BRIGHTNESS 1
#define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>
#include <functional>

#define DUSTLIB_SHARED_COLORMANAGER true
#define LED_COUNT (271)
#include <util.h>
#include <patterning.h>
#include <controls.h>
#include <drawing.h>

#include "patterns.h"
#include "ledgraph.h"

#include "MotionManager.h"

#define WAIT_FOR_SERIAL 0

#define PHOTOSENSOR_POWER_PIN 28
#define PHOTOSENSOR_READ_PIN 29

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

void setup() {
  init_serial();

  pinMode(UNCONNECTED_PIN_1, INPUT);
  auto noise = lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint32_t));
  randomSeed(lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint32_t)));
  random16_add_entropy(lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint16_t)));

  init_i2c();
  init_i2s();
  init_spi();

  MotionManager::manager().init();
  
  FastLED.addLeds<SK9822, LED_SPI0_TX, LED_SPI0_SCK, BGR, DATA_RATE_MHZ(16)>(ctx.leds, LED_COUNT);//.setCorrection(0xFFB0C0);

  patternManager.registerPattern<PulseHexa>();
  patternManager.registerPattern<MotionHexa>();
  patternManager.registerPattern<TriBounce>();
  patternManager.registerPattern<PixelDust>();

  // patternManager.registerPattern<LineSweep>();
  // patternManager.registerPattern<RandomDust>();

  
  IndexedPatternRunner *indexedRunner = patternManager.setupIndexedPattern(1);
  SPSTButton *button = controls.addButton(25);
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
  autoBrightness->flipSensor = true;
  autoBrightness->maxBrightness = 0x15; // conservative max brightness here because we have no thermistor to prevent thermal damage

  setupDoneTime = millis();
  logf("setup done");
} 

void startupWelcome() {
  int welcomeDuration = 666;

  ctx.leds.fill_solid(CRGB::Black);

  HexaShells hexaShells;
   uint8_t hue = random8();
   DrawModal(120, welcomeDuration, [hue, welcomeDuration, hexaShells](unsigned long elapsed) {
     FastLED.setBrightness(3);
     int s = hexaShells.shells.size() * elapsed / (welcomeDuration/3);
     ctx.leds.fadeToBlackBy(66 - 44 * min(s, hexaShells.shells.size())/hexaShells.shells.size());
     if (s < hexaShells.shells.size()) {
       for (int px : hexaShells.shells[s]) {
         uint8_t b = 0xFF - 0x66 * s/hexaShells.shells.size();
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
  // TODO: takes roughly 4.5ms to read AGMT. consider offloading to other core?
  MotionManager::manager().loop();
  
  patternManager.loop();
  controls.update();
  autoBrightness->loop();
  
  FastLED.show();
  fc.loop();
  fc.clampToFramerate(120);
  
}
