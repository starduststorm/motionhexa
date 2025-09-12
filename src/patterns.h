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
  vector<vector<std::optional<PixelIndex> > > shells;

  HexaShells(PixelIndex center) {
    // generate a series of hexashells centered around the given pixel
    Axial ax = axial.axialFromPixelIndex(center);
    int centerQ = ax.q();
    int centerR = ax.r();

    int shellCount = (kMeridian+1) / 2 + abs(centerQ) + abs(centerR);

    shells.emplace_back();
    shells.back().push_back(axial.indexAtAxial(centerQ, centerR)); // center px

    for (int s = 1; s < shellCount; ++s) {
      shells.emplace_back();
      int q = centerQ + s; // start each shell at q+shellnum to the right
      int r = centerR;
      // go counterclockwise around the shell
      for (int si = 0; si < s; ++si) {
        shells.back().push_back(axial.indexAtAxial(q, --r)); 
      }
      for (int si = 0; si < s; ++si) {
        shells.back().push_back(axial.indexAtAxial(--q, r)); 
      }
      for (int si = 0; si < s; ++si) {
        shells.back().push_back(axial.indexAtAxial(--q, ++r)); 
      }
      for (int si = 0; si < s; ++si) {
        shells.back().push_back(axial.indexAtAxial(q, ++r)); 
      }
      for (int si = 0; si < s; ++si) {
        shells.back().push_back(axial.indexAtAxial(++q, r)); 
      }
      for (int si = 0; si < s; ++si) {
        shells.back().push_back(axial.indexAtAxial(++q, --r)); 
      }
    }
  }
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

class PulseHexa : public Pattern, PaletteRotation<CRGBPalette256> {
public:
  HexaShells hexaShells;
  PulseHexa() {
    maxColorJump = 30;
    secondsPerPalette = 15;
  }

  void update() {
    for (int s = 0 ; s < hexaShells.shells.size(); ++s) {
      for (std::optional<PixelIndex> pxOpt : hexaShells.shells[s]) {
        if (!pxOpt.has_value()) continue;
        PixelIndex px = pxOpt.value();
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

class PulseHexaSmooth : public Pattern, PaletteRotation<CRGBPalette256> {
public:
  Axial center;
  PulseHexaSmooth() {
    maxColorJump = 30;
    secondsPerPalette = 15;
  }

  vector32 smoothAcc;

  void update() {
    int mult = 1000; // smooth everything with integer math
    auto agmt = MotionManager::motionFrame.agmt;
    vector32 acc = vector32(agmt.acc.axes.x, agmt.acc.axes.y, agmt.acc.axes.z);
    smoothAcc = (9 * smoothAcc + acc) / 10;

    constexpr int kInverseRootThree = 1000*1/sqrt(3); // FIXME: save as integer?
    Axial offcenter = center;
    int q = offcenter.q() + smoothAcc.y + kInverseRootThree * smoothAcc.x / 1000;
    int r = offcenter.r() - smoothAcc.x;
    offcenter.setQR(q,r);

    for (PixelIndex px = 0; px < LED_COUNT; ++px) {
      Axial ax = axial.axialFromPixelIndex(px);
      ax *= mult;
      
      int distance = max(max(abs(offcenter.q() - ax.q()), abs(offcenter.r() - ax.r())), abs(offcenter.s() - ax.s()));
      
      uint8_t brightness = beatsin8(60, 0, 255, 0, -beatsin16(2, 250, 350)*distance/(kMeridian/2)/mult);
      CRGB c = this->getMirroredPaletteColor(millis()/100 + distance*15/mult);
      c = c.scale8(brightness);
      ctx.leds[px] = c;
    }
  }

  const char *description() {
    return "PulseHexaSmooth";
  }
};


/* Concept
  PulseHexa except each shell is a looped palette which rotates as you rotate the hexagon.
  Hexa zooms in and out with motion along z axis?
  in any case add parameters and link them to motion
*/
// FIXME: this has a continuity issue where the animation jumps across some probably modulus overflow OH or in the accAccum??
class MotionHexa : public Pattern, PaletteRotation<CRGBPalette256> {
public:
  HexaShells hexaShells;
  MotionHexa() {
    secondsPerPalette = 16;
    maxColorJump = 30;
  }

  vector32 gyrAccum32;
  vector32 accAccum32;

  void update() {
    const int gyrScale = 2000;
    const int accScale = 2000;
    ICM_20948_AGMT_t agmt = MotionManager::motionFrame.agmt;
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
      
      // fade in at start
      const long fadeinDuration = 1000;
      // uint8_t shellBrightness = runTime() < fadeinDuration ? max(0, min(0xFF, 0xFF * (runTime() - fadeinDuration/hexaShells.shells.size()*s)/fadeinDuration * (hexaShells.shells.size() - s) / hexaShells.shells.size())) : 0xFF;

      uint8_t shellBrightness = 0xFF;
      if (runTime() < fadeinDuration) {
        long fadeOverlap = hexaShells.shells.size()/2;
        long shellFadeTime = fadeinDuration/(hexaShells.shells.size() + fadeOverlap);
        shellBrightness = (runTime() > s * shellFadeTime ? min(0xFF, 0xFF * (runTime() - s*shellFadeTime) / (fadeOverlap * shellFadeTime)) : 0);
      }

      for (int si = 0; si < hexaShells.shells[s].size(); ++si) {
        auto pxOpt = hexaShells.shells[s][si];
        if (!pxOpt.has_value()) continue;
        PixelIndex px = pxOpt.value();

        uint8_t brightness = lerp8by8(sin8(-bandRotate/4 + bands*(0xFF*si - bandTwist) / shellSize - 0xFF * (s-bandThing)/shellCount), 0xFF, bandFadeIn);

        brightness = scale8(brightness, brightness);
        uint16_t gyrRotate = gyrAccum.z>>1 % 0x200;
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
        c.nscale8(brightness);
        c.nscale8(shellBrightness);
        ctx.leds[px] = c;
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
        ctx.leds[hexaShells.shells[s][si].value()] = c;
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
        ctx.leds[hexaShells.shells[s][si].value()] = c;
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
  int fadeDown = 0xFF;
  BouncyPixels(PixelIndex pixelCount, uint8_t accelScaling, uint8_t elasticity, uint8_t elasticityMultiplier=1) : physics(hexGrid, pixelCount, accelScaling, elasticity, elasticityMultiplier), pixelCount(pixelCount) {
    minBrightness = 15;
  }

  virtual void update() {
    ctx.leds.fadeToBlackBy(fadeDown);
    physics.update([](PixelIndex index) {
      return accelerationAtPixelIndex(index, MotionManager::motionFrame.agmt);
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

class PixelSand : public BouncyPixels {
public:
  PixelSand() : BouncyPixels(60, 0x07, 0xC0) {
  }
  const char *description() {
    return "PixelSand";
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

/* ------------------------------------------------------------------------------- */

class LineTest : public Pattern, PaletteRotation<CRGBPalette256> {
public:
  LineTest() {
    minBrightness = 20;
    setPalette(Trans_Flag_gp);
    pauseRotation = true;
  }
  void update() {
    ctx.leds.fadeToBlackBy(5);
    
    vectorT<float> pt1 = {kMeridian*cosf(millis()/500.), kMeridian*sinf(millis()/500.)};
    vectorT<float> pt2 = {kMeridian*-cosf(millis()/500.), kMeridian*-sinf(millis()/500.)};
    fAxial ax1 = axial.rectToHex(pt1, 1.0);
    fAxial ax2 = axial.rectToHex(pt2, 1.0);

    hexline(ctx, ax1, ax2, [this] (uint8_t progress) {
      return this->getPaletteColor(progress);
    });
  }
  const char *description() {
    return "LineTest";
  }
};

/* ------------------------------------------------------------------------------- */

class ChargingPattern : public Pattern {
public:
  int lastStateOfCharge = 0;
  int animateFromSOC = 0;
  unsigned long lastValueChange;

  ChargingPattern() : lastValueChange(millis()) {}
  void update() {
    ctx.leds.fill_solid(CRGB::Black);
    HexaShells shells;
    auto outerShell = shells.shells.back();

    const int ringAnimateTime = 1000;
    const int minHue = 0;
    const int maxHue = 0x66;
    const PixelIndex firstIdx = 14; // start near usb port
    int SOC = batteryData.stateOfCharge;

    // animate any jumps in reported battery value
    if (SOC != lastStateOfCharge) {
      lastValueChange = millis();
      animateFromSOC = lastStateOfCharge;
      lastStateOfCharge = SOC;
    }
    
    long animationMillis = millis() - lastValueChange;
    int displaySOC = (animationMillis > ringAnimateTime)
                      ? SOC
                      : (animateFromSOC + (SOC - animateFromSOC) * ease8InOutQuad(0xFF*animationMillis/ringAnimateTime) / 0xFF);
    int displayLength = displaySOC * outerShell.size() / 100;
    int maxLength = SOC * outerShell.size() / 100;
    
    uint8_t hue = maxHue * SOC / 100 - minHue;
    CRGB color = CHSV(hue, 0xFF, 0xAF);

    for (int i = 0; i < displayLength; ++i) {
      ctx.leds[outerShell[(i + firstIdx) % outerShell.size()].value()] = color.scale8(0x50 + 0x9F*i / displayLength);
    }
    if (animationMillis > ringAnimateTime) {
      if (displayLength < outerShell.size()) {
        ctx.leds[outerShell[(displayLength + firstIdx) % outerShell.size()].value()] = color.scale8(beatsin8(30));
      }
    }
  }
  const char *description() {
    return "ChargingPattern";
  }
};

class PowerOffAnimation : public Pattern {
public:
  void update() {
    // FIXME: animation is basic - improve
    const int collapseTime = 600;
    const int dotTime = 600;
    ctx.leds.fill_solid(CRGB::Black);
    if (runTime() < collapseTime) {
      auto centerNode = hexGrid.nodes[LED_COUNT/2];
      
      float brightspot = kMeridian/2 - kMeridian/2 * runTime() / (float)collapseTime;
      for (int i = 0; i < 6; ++i) {
        int distance = 1;
        HexGrid<PixelIndex>::HexNode *node = centerNode->neighbors[i];
        do {
          ctx.leds[node->data()] = CHSV(0, 0xFF, 0xFF - 0xFF * abs(distance-brightspot)/5);
          node = node->neighbors[i];
          distance++;
        } while (node->isDataNode());
      }
    } else if (runTime() < collapseTime + dotTime) {
      ctx.leds[LED_COUNT/2] = CHSV(0, 0xFF, 0xFF - 0xFF*(runTime()-collapseTime)/dotTime);
    } else {
      stop();
    }
  }
  const char *description() {
    return "PowerOff";
  }

};

#endif
