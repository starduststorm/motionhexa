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

#define UNCONNECTED_PIN_1 17
#define UNCONNECTED_PIN_2 18

#include <Arduino.h>
#include <SPI.h>

// These SPI pins are swapped from spec - FML but does not prevent FastLED from working
#define LED_SPI0_SCK 19
#define LED_SPI0_TX 18

#define FASTLED_USE_PROGMEM 1
#define FASTLED_USE_GLOBAL_BRIGHTNESS 1
#define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>

#include "util.h"
#include "drawing.h"
#include "PatternManager.h"
#include "ledgraph.h"
#include "controls.h"

#include <functional>

#include "MotionManager.h"

#define WAIT_FOR_SERIAL 1

DrawingContext ctx;
HardwareControls controls;

FrameCounter fc;
PatternManager<DrawingContext> patternManager(ctx);

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

int framelimit = 120;

void setup() {
  init_serial();
  randomSeed(lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint32_t)));
  random16_add_entropy(lsb_noise(UNCONNECTED_PIN_2, 8 * sizeof(uint16_t)));

  init_i2c();
  init_i2s();
  init_spi();
  FastLED.addLeds<SK9822, LED_SPI0_TX, LED_SPI0_SCK, BGR, DATA_RATE_MHZ(16)>(ctx.leds, LED_COUNT);//.setCorrection(0xFFB0C0);

  SPSTButton *button = controls.addButton(25);
  button->onSinglePress([]() {
    if (framelimit == 5) {
      framelimit = 120;
    } else {
      framelimit = 5;
    }
  });
  

  fc.tick();

  patternManager.setup();

  initLEDGraph();
  assert(ledgraph.adjList.size() == LED_COUNT, "adjlist size should match LED_COUNT");

  
  gpio_init(11);
  gpio_set_dir(11, GPIO_OUT);

  setupDoneTime = millis();
} 

// drawing

void drawLoop() {
  // TODO: brightness update from sensor

  FastLED.show();
  fc.tick();
  fc.clampToFramerate(framelimit);
}

void startupWelcome() {
  int welcomeDuration = 500;

  ctx.leds.fill_solid(CRGB::Black);

  DrawModal(120, welcomeDuration, [](unsigned long elapsed) {
    
    drawLoop();
  });
  ctx.leds.fill_solid(CRGB::Black);
  FastLED.show();
}

void loop() {
  if (serialTimeout && millis() - setupDoneTime < 1000) {
    serialTimeoutIndicator();
    return;
  }

  // int32_t l, r;
  // i2s.read32(&l, &r);
  // Serial.printf("%d %d\r\n", l, r);

  static bool firstLoop = true;
  if (firstLoop) {
    startupWelcome();
    firstLoop = false;
  }

  FastLED.setBrightness(0x10);
  patternManager.loop();
  controls.update();
  
  // graphTest(ctx);
  drawLoop();
}

