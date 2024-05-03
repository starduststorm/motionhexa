#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>
#include <stdarg.h>     /* va_list, va_start, va_arg, va_end */
#include <functional>

#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))
#define ARRAY_SAMPLE(a) (ARRAY_SIZE(a) < 255 ? a[random8(ARRAY_SIZE(a))] : a[random16(ARRAY_SIZE(a))])

#ifndef MIN
#define MIN(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef MAX
#define MAX(a,b) ((a)>(b)?(a):(b))
#endif

int freeRAM();

static int vasprintf(char** strp, const char* fmt, va_list ap) {
  va_list ap2;
  va_copy(ap2, ap);
  char tmp[1];
  int size = vsnprintf(tmp, 1, fmt, ap2);
  if (size <= 0) {
    *strp=NULL;
    return size;
  }
  va_end(ap2);
  size += 1;
  *strp = (char*)malloc(size * sizeof(char));
  return vsnprintf(*strp, size, fmt, ap);
}

static void _logf(bool newline, const char *format, va_list argptr)
{
  if (!Serial) return;
  if (strlen(format) == 0) {
    if (newline) {
      Serial.println();
    }
    return;
  }
  char *buf;
  vasprintf(&buf, format, argptr);
  if (newline) {
    Serial.println(buf ? buf : "LOGF MEMORY ERROR");
  } else {
    Serial.print(buf ? buf : "LOGF MEMORY ERROR");
  }
#if DEBUG
  Serial.flush();
#endif
  free(buf);
}

#if DEBUG
#define logdf(format, ...) logf(format, ## __VA_ARGS__)
#else
#define logdf(format, ...)
#endif

void logf(const char *format, ...)
{
  va_list argptr;
  va_start(argptr, format);
  _logf(true, format, argptr);
  va_end(argptr);
}

void loglf(const char *format, ...)
{
  va_list argptr;
  va_start(argptr, format);
  _logf(false, format, argptr);
  va_end(argptr);
}

#undef assert
#define assert(expr, reasonFormat, ...) assert_func((expr), #expr, reasonFormat, ## __VA_ARGS__);

void assert_func(bool result, const char *pred, const char *reasonFormat, ...) {
  if (!result) {
    logf("ASSERTION FAILED: %s", pred);
    va_list argptr;
    va_start(argptr, reasonFormat);
    _logf(true, reasonFormat, argptr);
    va_end(argptr);
#if DEBUG
    while (1) delay(100);
#endif
  }
}

#define MOD_DISTANCE(a, b, m) (m / 2. - fmod((3 * m) / 2 + a - b, m))

inline int mod_wrap(int x, int m) {
  int result = x % m;
  return result < 0 ? result + m : result;
}

inline float fmod_wrap(float x, int m) {
  float result = fmod(x, m);
  return result < 0 ? result + m : result;
}

void DrawModal(int fps, unsigned long durationMillis, std::function<void(unsigned long elapsed)> tick) {
  int delayMillis = 1000/fps;
  unsigned long start = millis();
  unsigned long elapsed = 0;
  do {
    tick(elapsed);
    FastLED.show();
    FastLED.delay(delayMillis);
    elapsed = millis() - start;
  } while (elapsed < durationMillis);
}

class FrameCounter {
  private:
    unsigned long lastPrint = 0;
    long frames = 0;
    long lastClamp = 0;
  public:
    long printInterval = 2000;
    int fps() {
      unsigned long mil = millis();
      long elapsed = MAX(1, mil - lastPrint);
      return (int)(frames / (float)elapsed * 1000);
    }

    void tick() {
      unsigned long mil = millis();
      long elapsed = MAX(1, mil - lastPrint);
      if (elapsed > printInterval) {
        if (lastPrint != 0) {
          // arduino-samd-core can't sprintf floats??
          // not sure why it's not working for me, I should have Arduino SAMD core v1.8.9
          // https://github.com/arduino/ArduinoCore-samd/issues/407
          logf("Framerate: %i, free mem: %i", fps(), freeRAM());
        }
        frames = 0;
        lastPrint = mil;
      }
      ++frames;
    }
    void clampToFramerate(int fps) {
      int delayms = 1000 / fps - (millis() - lastClamp);
      if (delayms > 0) {
#ifdef FASTLED_VERSION
        FastLED.delay(delayms);
#else
        delay(delayms);
#endif
      }
      lastClamp = millis();
    }
};

template <uint8_t SIZE>
void shuffle(int arr[SIZE]) {
  for (unsigned i = 0; i < SIZE; ++i) {
    uint8_t swap = random8(SIZE);
    uint8_t tmp;
    tmp = arr[i];
    arr[i] = arr[swap];
    arr[swap] = tmp;
  }
}

int lsb_noise(int pin, int numbits) {
  // TODO: Use Entropy.h? Probs not needed just to randomize pattern.
  int noise = 0;
  for (int i = 0; i < numbits; ++i) {
    int val = analogRead(pin);
    noise = (noise << 1) | (val & 1);
  }
  return noise;
}

void printColor(CRGB color) {
  loglf("CRGB(0x%x, 0x%x, 0x%x)", color.r, color.g, color.b);
}

void printColor(CHSV color) {
  loglf("CHSV(0x%x, 0x%x, 0x%x)", color.h, color.s, color.v);
}

int freeRAM() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif
}

#endif
