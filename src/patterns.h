#ifndef PATTERN_H
#define PATTERN_H

#include <FastLED.h>
#include <vector>
#include <functional>
#include <optional>

#include <util.h>
#include <paletting.h>
#include <patterning.h>

#include "ledgraph.h"
#include "drawing.h"
#include "MotionManager.h"
#include "hexaphysics.h"


struct HexaShells {
  vector<vector<int> > shells = vector<vector<int> >();
  HexaShells() {
    vector<PixelIndex> shellStarts = {0};
    // get a diagonal line from edge to center
    while (shellStarts.back() != kHexaCenterIndex) {
      shellStarts.push_back(hexGrid[shellStarts.back()]->named.dr->data());
    }
    for (int i = shellStarts.size() - 1; i >= 0; --i) {
      PixelIndex startIndex = shellStarts[i];
      PixelIndex index = startIndex;
      shells.emplace_back();
      while (1) {
        shells.back().push_back(index);
        vector<Edge> edges = ledgraph.adjacencies(index, MakeEdgeTypesQuad(EdgeType::clockwise));
        if (edges.size() == 1) {
          index = edges[0].to;
        } else {
          break;
        }
      };
    }
  }
};

// FIXME: pull out PaletteRotation in favor of sharedColorManager?
class PulseHexa : public Pattern, PaletteRotation<CRGBPalette256> {
public:
  HexaShells hexaShells;
  PulseHexa() {
    maxColorJump = 50;
    secondsPerPalette = 15;
  }

  void update() {
    for (int s = 0 ; s < hexaShells.shells.size(); ++s) {
      for (int px : hexaShells.shells[s]) {
        uint8_t brightness = beatsin8(60, 0, 255, 0, -beatsin16(2, 250, 350)*s/hexaShells.shells.size());
        brightness = scale8(brightness, brightness);
        // ctx.leds[px] = CHSV(millis()/20+s*10, 0xFF, brightness);
        CRGB c = this->getMirroredPaletteColor(millis()/100 + s*15);
        c = c.scale8(brightness);
        ctx.leds[px] = c;
      }
    }
  }

  const char *description() {
    return "PulseHexa";
  }
};

/* Concept
  PulseHexa except each shell is a looped palette which rotates as you rotate the hexagon.
  Hexa zooms in and out with motion along z axis?
  in any case add parameters and link them to motion
*/
class MotionHexa : public Pattern, PaletteRotation<CRGBPalette256> {
public:
  HexaShells hexaShells;
  MotionHexa() {
    secondsPerPalette = 16;
    maxColorJump = 50;
  }

  vector32 gyrAccum32;
  vector32 accAccum32;

  void update() {
    const int gyrScale = 2000;
    const int accScale = 2000;
    ICM_20948_AGMT_t agmt = MotionManager::agmt;
    gyrAccum32 += vector16(agmt.gyr.axes.x, agmt.gyr.axes.y, agmt.gyr.axes.z);
    accAccum32 += vector16(agmt.acc.axes.x, agmt.acc.axes.y, agmt.acc.axes.z);
    vector16 gyrAccum = gyrAccum32 / gyrScale;
    vector16 accAccum = accAccum32 / accScale;
    // logf("gyr = (%i, %i, %i), gyrAccum = (%i, %i, %i), accel = (%i, %i, %i), accelAccum = (%i, %i, %i)", 
    //         agmt.gyr.axes.x/gyrScale, agmt.gyr.axes.y/gyrScale, agmt.gyr.axes.z/gyrScale,
    //         gyrAccum.x, gyrAccum.y, gyrAccum.z,
    //         agmt.acc.axes.x/accScale, agmt.acc.axes.y/accScale, agmt.acc.axes.z/accScale,
    //         accAccum.x, accAccum.y, accAccum.z);
    
    int index = 0;
    int shellCount = hexaShells.shells.size();
    for (int s = 0 ; s < hexaShells.shells.size(); ++s) {
      uint8_t shellSize = hexaShells.shells[s].size();
      
      const int16_t bandIndex = gyrAccum.y<<1; // TODO: tune this so it's roughly one half index change every complete flip
      const int16_t bandRotate = accAccum.y;
      const int16_t bandTwist = accAccum.x;//gyrAccum.z<<1;
      const int16_t bandThing = 0;//accAccum.x;
      const int bandCounts[] = {0, 1, 2, 3, 6, 9}; // i like this somewhat better than arbitrary band counts
      int bands = bandCounts[((uint16_t)(bandIndex+INT16_MAX) / (1<<12)) % ARRAY_SIZE(bandCounts)];
      int withinBand = (uint16_t)(bandIndex+INT16_MAX-(1<<11)) % (1<<12);
      uint8_t bandFadeIn = 0xFF - cos8(0xFF*withinBand / (1<<12));
      
      for (int si = 0; si < hexaShells.shells[s].size(); ++si) {
        uint8_t brightness = lerp8by8(sin8(-bandRotate/4 + bands*(0xFF*si - bandTwist) / shellSize - 0xFF * (s-bandThing)/shellCount), 0xFF, bandFadeIn);

        brightness = scale8(brightness, brightness);
        uint16_t gyrRotate = gyrAccum.z % 0x200;
        uint16_t radialH =  0x200 * si / shellSize;
        uint16_t twistFactor = s * gyrAccum.x/6 % 0x200 + s*millis()/500;
        uint16_t shellH = 0x200 * s/shellCount * beatsin16(3, 0, 0x200, 0, gyrAccum.y) / 0x200;
        uint16_t evolve = millis()/100;
        CRGB c = this->getMirroredPaletteColor(gyrRotate + radialH + twistFactor + shellH + evolve);
        
        // improvement: do this in certain accelerometer conditions
        // if (si%2) {
        //   brightness = scale8(brightness, beatsin8(10));
        // } else {
        //   brightness = scale8(brightness, beatsin8(10, 0, 0xFF, 0, 0x7F));
        // }
        c = c.scale8(brightness);
        ctx.leds[hexaShells.shells[s][si]] = c;
      }
    }
  }

  const char *description() {
    return "MotionHexa";
  }
};

/* ------------------------------------------------------------------------------- */

// Radar sweep
class LineSweep : public Pattern, PaletteRotation<CRGBPalette256> {
public:
  HexaShells hexaShells;
  int maxShellSize = 0;
  LineSweep() {
    maxColorJump = 7;
    secondsPerPalette = 7;
    minBrightness = 10;
    for (auto shell : hexaShells.shells) {
      if (shell.size() > maxShellSize) {
        maxShellSize = shell.size();
      }
    }
  }

  void update() {
    ctx.leds.fadeToBlackBy(18);
    for (int s = 0 ; s < hexaShells.shells.size(); ++s) {
      uint8_t shellSize = hexaShells.shells[s].size();
      
      for (int l = 0; l < 2; ++l) {
        unsigned long index = millis()/30;
        int si = ((shellSize * (index + l)) / maxShellSize)%shellSize;
        CRGB c = getMirroredPaletteColor(millis()/20, (l == 0 ? 0xFF : 0x7F));
        ctx.leds[hexaShells.shells[s][si]] = c;
      }
    }
  }

  const char *description() {
    return "LineSweep";
  }
};

/* ------------------------------------------------------------------------------- */

// broken version of LineSweep that Sequoia thought was neat
class LineSweepOops : public Pattern, PaletteRotation<CRGBPalette256> {
public:
  HexaShells hexaShells;
  LineSweepOops() {
    maxColorJump = 7;
    secondsPerPalette = 15;
  }

  void update() {
    ctx.leds.fadeToBlackBy(5);
    int shellCount = hexaShells.shells.size();
    for (int s = 0 ; s < hexaShells.shells.size(); ++s) {
      uint8_t shellSize = hexaShells.shells[s].size();
      
      for (int l = 0; l < 3; ++l) {
        unsigned long index = millis()/100;
        int si = (shellSize * index / shellSize)%shellSize;
        if (s == 0) {
          logf("si = %i", si);
        }
        CRGB c = CRGB::Red;
        ctx.leds[hexaShells.shells[s][si]] = c;
      }
    }
  }
  const char *description() {
    return "LineSweepOops";
  }
};

/* ------------------------------------------------------------------------------- */

class BouncyPixels : public Pattern, PaletteRotation<CRGBPalette256> {
public:
  const PixelIndex pixelCount;
  PixelPhysics<LED_COUNT> physics;
  BouncyPixels(PixelIndex pixelCount, uint8_t accelScaling, uint8_t elasticity, uint8_t elasticityMultiplier=1) : physics(hexGrid, pixelCount, accelScaling, elasticity, elasticityMultiplier), pixelCount(pixelCount) {
    minBrightness = 15;
  }

  virtual void update() {
    ctx.leds.fill_solid(CRGB::Black);
    physics.update([](PixelIndex index) {
      return accelerationAtPixelIndex(index, MotionManager::agmt);
    });
    int i = 0;
    for (PixelPhysics<LED_COUNT>::Particle *p : physics.particles) {
      CRGB color = getShiftingPaletteColor(0xFF * i++ / physics.particles.size());
      ctx.leds[p->index] = color;
    }
  }

  virtual const char *description() {
    return "BouncyPixels";
  }
};

class TriBounce : public BouncyPixels {
public:
  TriBounce() : BouncyPixels(3, 0x07, 0xFF, 2) {
  }
  void update() {
    BouncyPixels::update();
    int i = 0;
    for (PixelPhysics<LED_COUNT>::Particle *p : physics.particles) {
      CRGB color = CHSV(i++ * 0xFF/pixelCount, 0xFF, 0xFF);
      ctx.leds[p->index] = color;
    }
  }
  const char *description() {
    return "TriBounce";
  }
};

class PixelDust : public BouncyPixels {
public:
  PixelDust() : BouncyPixels(60, 0x07, 0xF4) {
  }
  const char *description() {
    return "PixelDust";
  }
};

class RandomDust : public BouncyPixels {
public:
  RandomDust() : BouncyPixels(random8(100)+1, random8(20), random8(255)) {
    logf("RandomDust chose pixelCount=%i, accelScaling=%i, elasticity=%i", physics.particles.size(), physics.accelScaling, physics.elasticity);
  }
  const char *description() {
    return "RandomDust";
  }
};

#endif
