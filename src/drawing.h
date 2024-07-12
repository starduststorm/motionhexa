
#ifndef DRAWING_H
#define DRAWING_H

#include <stack>
#include <FastLED.h>

// Workaround for linker issues when using copy-constructors for DrawStyle struct (since something is built with -fno-exceptions)
// https://forum.pjrc.com/threads/57192-Teensy-4-0-linker-issues-with-STL-libraries
extern "C"{
  int __exidx_start(){ return -1;}
  int __exidx_end(){ return -1; }
}
//

enum BlendMode {
  blendSourceOver, blendBrighten, blendDarken, blendSubtract, /* add blending? but how to encode alpha? need CRGBA buffers, probs not worth it with current resolution */
};

struct DrawStyle {
public:
  BlendMode blendMode = blendSourceOver;
  bool wrap = false;
};

template<uint16_t WIDTH, uint16_t HEIGHT, class PixelType, typename PixelSetType>
class CustomDrawingContext {
private:
  void set_px(CustomDrawingContext<WIDTH, HEIGHT, PixelType, PixelSetType> &dstCtx, PixelType src, int index, BlendMode blendMode, uint8_t brightness) {
    src.nscale8(brightness);
    switch (blendMode) {
      case blendSourceOver:
        dstCtx.leds[index] = src;
        break;
      case blendBrighten: {
        PixelType dst = dstCtx.leds[index];
        dstCtx.leds[index] = PixelType(std::max(src.r, dst.r), std::max(src.g, dst.g), std::max(src.b, dst.b));
        break;
      }
      case blendDarken: {
        PixelType dst = dstCtx.leds[index];
        dstCtx.leds[index] = PixelType(std::min(src.r, dst.r), std::min(src.g, dst.g), std::min(src.b, dst.b));
      }
      case blendSubtract: {
        PixelType dst = dstCtx.leds[index];
        dstCtx.leds[index] = dst - src;
      }
    }
  }
public:
  PixelSetType leds;
  const uint16_t width = WIDTH;
  const uint16_t height = HEIGHT;
  CustomDrawingContext() {  
    leds.fill_solid(CRGB::Black);
  }
  
  void blendIntoContext(CustomDrawingContext<WIDTH, HEIGHT, PixelType, PixelSetType> &otherContext, BlendMode blendMode, uint8_t brightness=0xFF) {
    assert(otherContext.leds.size() == this->leds.size(), "context blending requires same-size buffers");
    for (int i = 0; i < leds.size(); ++i) {
      set_px(otherContext, leds[i], i, blendMode, brightness);
    }
  }

  void point(uint16_t x, uint16_t y, CRGB c, BlendMode blendMode = blendSourceOver) {
    assert(x < WIDTH, "x=%u is out of range [0,%u]", width-1);
    assert(y < HEIGHT, "x%u is out of range [0,%u]", height-1);
    if (x < width && y < height) {
      set_px(*this, c, WIDTH*y + x, blendMode, 0xFF);
    }
  }
};

/* Floating-point pixel buffer support */

typedef struct FCRGB {
  union {
    struct {
      union {
        float r;
        float red;
      };
      union {
        float g;
        float green;
      };
      union {
        float b;
        float blue;
      };
    };
    float raw[3];
  };
public:
  FCRGB() { }
  FCRGB(CRGB color) : red(color.r), green(color.g), blue(color.b) { }
  FCRGB(float r, float g, float b) : red(r), green(g), blue(b) { }
  inline float& operator[] (uint8_t x) __attribute__((always_inline)) {
    return raw[x];
  }
} FCRGB;

template<int SIZE>
class FCRGBArray {
  FCRGB entries[SIZE];
public:
  inline FCRGB& operator[] (uint16_t x) __attribute__((always_inline)) {
    return entries[x];
  };
};

#endif
