
#ifndef DRAWING_H
#define DRAWING_H

#include <stack>
#include <FastLED.h>
#include <util.h>

enum BlendMode {
  blendSourceOver, blendBrighten, blendDarken, blendSubtract, /* add blending? but how to encode alpha? need CRGBA buffers, probs not worth it with current resolution */
};

struct DrawStyle {
public:
  BlendMode blendMode = blendSourceOver;
};

template<int COUNT, class PixelType=CRGB, template<int SIZE> typename PixelSetType=CRGBArray>
class PixelStorage {
private:
  inline void set_px(PixelType src, int index, BlendMode blendMode, uint8_t brightness) {
    src.nscale8(brightness);
    switch (blendMode) {
      case blendSourceOver:
        leds[index] = src;
        break;
      case blendBrighten: {
        PixelType dst = leds[index];
        leds[index] = PixelType(std::max(src.r, dst.r), std::max(src.g, dst.g), std::max(src.b, dst.b));
        break;
      }
      case blendDarken: {
        PixelType dst = leds[index];
        leds[index] = PixelType(std::min(src.r, dst.r), std::min(src.g, dst.g), std::min(src.b, dst.b));
        break;
      }
      case blendSubtract: {
        PixelType dst = leds[index];
        leds[index] = dst - src;
        break;
      }
    }
  }
public:
  PixelSetType<COUNT> leds;
  const uint16_t count;
  PixelStorage() : count(COUNT) {
    leds.fill_solid(CRGB::Black);
  }
  
  void blendIntoContext(PixelStorage<COUNT, PixelType, PixelSetType> &otherContext, BlendMode blendMode, uint8_t brightness=0xFF) {
    if (brightness > 0) {
      // TODO: if blendMode is sourceOver can we use memcpy?
      assert(otherContext.leds.size() == this->leds.size(), "context blending requires same-size buffers");
      for (int i = 0; i < leds.size(); ++i) {
        otherContext.set_px(leds[i], i, blendMode, brightness);
      }
    }
  }

  void point(unsigned int index, CRGB c, BlendMode blendMode = blendSourceOver) {
    assert(index < COUNT, "index=%u is out of range [0,%u]", count-1);  
    if (index < COUNT) {
      set_px(index, blendMode, 0xFF);
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
