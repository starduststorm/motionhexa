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
    constexpr int mult = 1000; // smooth everything with integer math
    auto agmt = MotionManager::motionFrame.agmt;
    vector32 acc = vector32(agmt.acc.axes.x, agmt.acc.axes.y, agmt.acc.axes.z);
    smoothAcc = (9 * smoothAcc + acc) / 10;

    constexpr int kInverseRootThree = mult*1/sqrt(3);
    Axial offcenter = center;
    int q = offcenter.q() + smoothAcc.y + kInverseRootThree * smoothAcc.x / mult;
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
    const int accScale = (MotionManager::manager().enableDMP ? 1000 : 2000); // coolcool cool coooool
    const int gyrScale = (MotionManager::manager().enableDMP ? 200 : 2000); // coolcoolcool
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
  TriBounce() : BouncyPixels(3, 70, 0xFF, 2) {
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
  PixelDust() : BouncyPixels(60, 70, 0xF4) {
  }
  const char *description() {
    return "PixelDust";
  }
};

class PixelSand : public BouncyPixels {
public:
  PixelSand() : BouncyPixels(60, 70, 0xC0) {
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

// Note: for this and any other "smooth motion" physics patterns, i could just do cartesian floating point physics on a single particle with walls and it'd probably be fine and even smoother.
class LargeBouncyBall : public Pattern, PaletteRotation<CRGBPalette256> {
  HexGrid<PixelIndex> insetHexGrid;
public:
  const int inset = 2;
  PixelPhysics<LED_COUNT> physics;
  LargeBouncyBall() : insetHexGrid(kMeridian), physics(insetHexGrid, 1, 70, 0xF0, 1) {
    minBrightness = 15;
    insetHexGrid.insetEdgeNodesBy(inset, axial);
    physics.particles[0]->index = kHexaCenterIndex;
  }

  vector32 gyrAccum32;
  virtual void update() {
    ctx.leds.fill_solid(CRGB::Black);

    const int gyrScale = (MotionManager::manager().enableDMP ? 200 : 2000); // coolcoolcool
    auto agmt = MotionManager::motionFrame.agmt;
    gyrAccum32 += vector16(agmt.gyr.axes.x, agmt.gyr.axes.y, agmt.gyr.axes.z);
    vector16 gyrAccum = gyrAccum32 / gyrScale;

    physics.update([](PixelIndex index) {
      return accelerationAtPixelIndex(index, MotionManager::motionFrame.agmt);
    });
    int i = 0;
    for (PixelPhysics<LED_COUNT>::Particle *p : physics.particles) {
      Axial ballAx = axial.axialFromPixelIndex(p->index);

      constexpr int kInverseRootThree = 1/sqrt(3);
      fAxial offcenter = ballAx;
      float q = offcenter.q() + p->pos.x/255./2;
      float r = offcenter.r() - p->pos.y/255./2 - kInverseRootThree * p->pos.x/255./2;
      offcenter.setQR(q,r);

      uint8_t hue = constrain((abs(p->velocity.x) + abs(p->velocity.y))/2 + (abs(p->velocity.x) + abs(p->velocity.y))/50, 0, 224);
      for (PixelIndex px = 0; px < LED_COUNT; ++px) {
        Axial ax = axial.axialFromPixelIndex(px);
        
        float distance = max(max(abs(offcenter.q() - ax.q()), abs(offcenter.r() - ax.r())), abs(offcenter.s() - ax.s()));
        uint8_t brightness = constrain(0xFF - 0xFF * distance / (inset+1), 0, 0xFF);
        
        CRGB c = CHSV(max(0,hue-6*(int)distance), 0xFF, 0xFF);
        c = c.scale8(brightness);
        ctx.leds[px] = c;
      }
    }
  }

  virtual const char *description() {
    return "LargeBouncyBall";
  }
};

/* ------------------------------------------------------------------------------- */

class TriangleSpin : public Pattern, PaletteRotation<CRGBPalette256> {
public:
  TriangleSpin() {
    secondsPerPalette = 20;
  };
  void update() {
    ctx.leds.fill_solid(CRGB::Black);
    float yaw = MotionManager::motionFrame.euler.yaw*PI/180;
    uint16_t yawBytes = max(0, min(0x1FF, (MotionManager::motionFrame.euler.yaw+180) * 0x1FF/360));

    float r = kMeridian-6;
    unsigned timeOffset = millis() / 50;

    vectorT<float> pt1 = {r * cosf(yaw + 0),      r * sinf(yaw + 0)};
    vectorT<float> pt2 = {r * cosf(yaw + 2*PI/3), r * sinf(yaw + 2*PI/3)};
    vectorT<float> pt3 = {r * cosf(yaw + 4*PI/3), r * sinf(yaw + 4*PI/3)};
    fAxial ax1 = axial.rectToHex(pt1, 1.0);
    fAxial ax2 = axial.rectToHex(pt2, 1.0);
    fAxial ax3 = axial.rectToHex(pt3, 1.0);

    hexline(ctx, ax1, ax2, [this, yawBytes, timeOffset] (uint8_t progress) {
      return getMirroredPaletteColor(timeOffset + yawBytes + 0 + progress);
    });
    hexline(ctx, ax2, ax3, [this, yawBytes, timeOffset] (uint8_t progress) {
      return getMirroredPaletteColor(timeOffset + yawBytes + 1*0x1FF/3 + progress);
    });
    hexline(ctx, ax3, ax1, [this, yawBytes, timeOffset] (uint8_t progress) {
      return getMirroredPaletteColor(timeOffset + yawBytes + 2*0x1FF/3 + progress);
    });
  }

  const char *description() {
    return "TriangleSpin";
  }
};

/* ------------------------------------------------------------------------------- */

class PridefulSpinnyThing : public Pattern {
public:
  CRGBPalette256 palettes[6] = {
    Trans_Flag_gp,
    Pride_Flag_gp,
    Genderqueer_Flag_gp,
    Bi_Flag_gp,
    Ace_Flag_gp,
    Lesbian_Flag_gp
  };
  PridefulSpinnyThing() {
    dSpin *= random8(2)?-1:1;
  }
  float avgZ=0;
  int lastSeenAtHighAngle = 0;
  float spinTheta = 0;
  float dSpin = 1/500.;
  void update() {
    
    ICM_20948_AGMT_t agmt = MotionManager::motionFrame.agmt;

    float theta = M_PI+atan2(agmt.acc.axes.y, agmt.acc.axes.x);
    int flag = 6*(theta+M_PI/12) / (2*M_PI);
    flag = mod_wrap(flag,6);
    
    const int maxHexRadius = (kMeridian-3);
    const int minHexRadius = -3;
    avgZ = (10*avgZ+agmt.acc.axes.z)/11.f;
    float lineRadius = kMeridian - (kMeridian+3) * abs(avgZ) / 9000.;
    float hexRadius = maxHexRadius * abs(avgZ) / 9000. + minHexRadius;

    if (lineRadius > kMeridian/2) {
      lastSeenAtHighAngle = flag;
    }

    ctx.leds.fadeToBlackBy(5 + (hexRadius>0?hexRadius:0));

    float scaledGyr = (agmt.gyr.axes.z / 6666) / 66666.f;
    spinTheta += frameTime() * dSpin;
  
    if (lineRadius > 0) {
      vectorT<float> pt1 = {lineRadius*cosf(spinTheta), lineRadius*sinf(spinTheta)};
      vectorT<float> pt2 = {lineRadius*-cosf(spinTheta), lineRadius*-sinf(spinTheta)};
      fAxial ax1 = axial.rectToHex(pt1, 1.0);
      fAxial ax2 = axial.rectToHex(pt2, 1.0);

      hexline(ctx, ax1, ax2, [this, flag] (uint8_t progress) {
        return ColorFromPalette(palettes[flag], progress);
      });
    }

    if (hexRadius > 0) {
      dSpin += frameTime()*(scaledGyr * (hexRadius/maxHexRadius));
      dSpin = constrain(dSpin, -0.1, 0.1);
      for (int i = 0; i < 6; ++i) {
        float ptTheta = i * 2*+M_PI/6;
        float ptTheta2 = (i+1) * 2*+M_PI/6;
        vectorT<float> pt1 = {hexRadius*cosf(ptTheta+spinTheta), hexRadius*sinf(ptTheta+spinTheta)};
        vectorT<float> pt2 = {hexRadius*cosf(ptTheta2+spinTheta), hexRadius*sinf(ptTheta2+spinTheta)};
        fAxial ax1 = axial.rectToHex(pt1, 1.0);
        fAxial ax2 = axial.rectToHex(pt2, 1.0);
        
        hexline(ctx, ax1, ax2, [this, i] (uint8_t progress) {
          return PaletteRotation<CRGBPalette256>::getMirroredPaletteColor(palettes[lastSeenAtHighAngle], progress/3 + 0xFF*i/3);
        });
      }
    }
  }
  const char *description() {
    return "PridefulSpinnyThing";
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
    int SOC = min(100, 100 * batteryData.stateOfCharge / kFullCharge);

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

class PowerOnOffAnimation : public Pattern {
  const int maxPosition = kMeridian/2;
  float position; // distance from origin 
public:
  bool animatingPowerOn = true;
  PowerOnOffAnimation(bool poweringOn) : animatingPowerOn(poweringOn), position(poweringOn?0:maxPosition) {
    setPoweringOn(poweringOn);
  }

  void setPoweringOn(bool poweringOn) {
    animatingPowerOn = poweringOn;
  }

  float progress() {
    return (animatingPowerOn ? position / maxPosition : (maxPosition - position) / maxPosition);
  }

  void update() {
    uint8_t centerPixelRed = ctx.leds[LED_COUNT/2].red;
    ctx.leds.fill_solid(CRGB::Black);
    
    const int duration = 1000;

    position += (animatingPowerOn ? 1 : -1) * (int)frameTime() * maxPosition / (float)duration;
    if (position < 0) {
      const int powerOffDonePos = -5;
      if (position < powerOffDonePos) {
        stop();
      } else {
        // final dot
        ctx.leds[LED_COUNT/2] = CHSV(0, 0xFF, 0xFF - 0xFF*(position/powerOffDonePos));
      }
    } else if (position > maxPosition) {
      stop();
    } else {
      const int waveSize = 5;
      const float expand = 1.8; // factor to expand the animation from the logical position
      float animationPosition = position * expand - maxPosition*(expand-1)/2;
      for (int q = 0; q <= maxPosition; ++q) {
        float distance = fabs(q - animationPosition);
        Axial ax(q,0);
        for (int i = 0; i < 6; ++i) {
          auto pxOpt = axial.indexAtAxial(ax);
          if (pxOpt) {
            PixelIndex px = pxOpt.value();
            CRGB c = CHSV(0, 0xFF, 0xFF - 0xFF * distance/waveSize);
            if (px == LED_COUNT/2 && !animatingPowerOn) {
              // hack to keep the final dot at a consistent brightness at the end
              c.red = max(c.red, centerPixelRed); 
            }
            ctx.leds[px] = c;
          }
          // rotate to next spoke
          ax = Axial(-ax.r(), -ax.s());
        }
      }
    }
  }
  const char *description() {
    return (animatingPowerOn ? "PowerOn" : "PowerOff");
  }
};

#endif
