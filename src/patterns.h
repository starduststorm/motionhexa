#ifndef PATTERN_H
#define PATTERN_H

#include <vector>
#include <functional>
#include <optional>

#include <FastLED.h>

#include <util.h>
#include <paletting.h>
#include <patterning.h>

#include "ledgraph.h"
#include "drawing.h"
#include "MotionManager.h"
#include "hexaphysics.h"
#include "particles.h"
#include <phaser.h>

struct HexaShells {
  vector<vector<std::optional<PixelIndex> > > shells;

  HexaShells(PixelIndex center, int maxShellCount = 0) {
    // generate a series of hexashells centered around the given pixel
    Axial ax = axial.axialFromPixelIndex(center);
    int centerQ = ax.q();
    int centerR = ax.r();

    int shellCount = min((maxShellCount==0 ? kMeridian : maxShellCount), (kMeridian+1) / 2 + abs(centerQ) + abs(centerR));

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

class PulseHexaSmooth : public Pattern, AmplitudeReceiver, PaletteRotation<CRGBPalette256> {
public:
  AxialT<int32_t> center;
  PulseHexaSmooth() : AmplitudeReceiver(audioInput) {
    maxColorJump = 30;
    secondsPerPalette = 9;
  }

  vector32 smoothAcc;

  void update() {
    constexpr int mult = 1000; // smooth everything with integer math
    auto agmt = MotionManager::motionFrame.agmt;

    vector32 acc = vector32(agmt.acc.axes.x, agmt.acc.axes.y, agmt.acc.axes.z) * 5;
    const int smoooooth = 10;
    smoothAcc = (smoooooth * smoothAcc + acc) / (smoooooth+1);

    constexpr int kInverseRootThree = mult*1/sqrt(3);
    AxialT<int32_t> offcenter = center;
    int q = offcenter.q() + smoothAcc.x + kInverseRootThree * smoothAcc.y / mult;
    int r = offcenter.r() - smoothAcc.y;
    offcenter.setQR(q,r);

    int amplitude = amplitudeFrame();

    for (PixelIndex px = 0; px < LED_COUNT; ++px) {
      AxialT<int32_t> ax(axial.axialFromPixelIndex(px));
      ax *= mult;
      
      const int kAccScale = 20000;
      const int kAmpScale = 600;
      const int kLocScale = 2000000;
      // full static glitch with no fade
      // int glitchIt = (smoothAcc.z>0 ? (ax.q()*ax.r()*ax.s()) * smoothAcc.z/500000. : 0);
      // smooth-transition glitch that also reacts to sound
      int glitchIt = (smoothAcc.z<0 ? (ax.q()*ax.r()*ax.s())/kLocScale * (1 + amplitude/kAmpScale) * smoothAcc.z/kAccScale: 0);
      int distance = max(max(abs(offcenter.q() - ax.q()), abs(offcenter.r() - ax.r())), abs(offcenter.s() - ax.s())) + glitchIt;
      
      uint8_t brightness = beatsin8(60, 0, 255, 0, -beatsin16(2, 250, 350)*distance/(kMeridian/2)/mult);
      CRGB c = this->getMirroredPaletteColor(millis()/100 + distance*15/mult + beatsin8(3, 0, kMeridian));
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
    vector32 gyrAccum = gyrAccum32 / gyrScale;
    vector32 accAccum = accAccum32 / accScale;
    // logf("gyr = (%i, %i, %i), gyrAccum = (%i, %i, %i), accel = (%i, %i, %i), accelAccum = (%i, %i, %i)", 
    //         agmt.gyr.axes.x/gyrScale, agmt.gyr.axes.y/gyrScale, agmt.gyr.axes.z/gyrScale,
    //         gyrAccum.x, gyrAccum.y, gyrAccum.z,
    //         agmt.acc.axes.x/accScale, agmt.acc.axes.y/accScale, agmt.acc.axes.z/accScale,
    //         accAccum.x, accAccum.y, accAccum.z);
    
    int index = 0;
    int shellCount = hexaShells.shells.size();
    for (int s = 0 ; s < hexaShells.shells.size(); ++s) {
      uint8_t shellSize = hexaShells.shells[s].size();
      
      const int32_t bandIndex = gyrAccum.x*2; // TODO: tune this so it's roughly one half index change every complete flip
      const int32_t bandRotate = accAccum.x;
      const int32_t bandTwist = accAccum.y;//gyrAccum.z*2;
      const int32_t bandThing = 0;//accAccum.x;
      const int bandCounts[] = {0, 1, 2, 3, 6, 9}; // i like this somewhat better than arbitrary band counts
      int32_t bands = bandCounts[((int32_t)(bandIndex+INT16_MAX) / (1<<12)) % ARRAY_SIZE(bandCounts)];
      int32_t withinBand = (int32_t)(bandIndex+INT16_MAX-(1<<11)) % (1<<12);
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
        int32_t gyrRotate = (gyrAccum.z/2) % 0x200;
        int32_t radialH =  0x200 * si / shellSize;
        int32_t twistFactor = (s * gyrAccum.y/8 + s * millis()/500) % 0x200;
        int32_t shellH = 0x200 * s/shellCount * beatsin16(3, 0, 0x200, 0, gyrAccum.x) / 0x200;
        int32_t evolve = (millis()/100)%0x200;
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

// special case the single ball physics since we can do nice floating point math for a single particle
class LargeBouncyBall : public Pattern {
  struct Ball {
    vectorf pos;
    vectorf velocity;
    Ball() : pos(0,0), velocity(0,0) {};
    Ball(vectorf pos, vectorf velocity) : pos(pos), velocity(velocity) {};
  };
public:
  Ball p;
  unsigned long boomStart = 0;

  void stellate(float radius, float bright) {
    for (PixelIndex px = 0; px < LED_COUNT; ++px) {
      Axial ax = axial.axialFromPixelIndex(px);
      int aq = abs(ax.q()), ar = abs(ax.r()), as = abs(ax.s());
      float stellatedDist = (max(max(aq, ar), as) + min(min(aq, ar), as) * 2) / 2;
      if (stellatedDist <= radius) {
        uint8_t b = bright * (1.0f - stellatedDist / max(radius, 0.01f)) * 255;
        ctx.leds[px] = CRGB(b, b, b);
      }
    }
  }

  void sideHit(Ball &p, int w, uint8_t hue, unsigned long elapsed) {
    assert(hexaSide(w).size() == 10,"hexa side size");
    uint8_t hitSpeed = constrain(2000 * p.velocity.length()*elapsed - 100, 0, 0xFF);
    for (PixelIndex px : hexaSide(w)) {
      ctx.leds[px] = CHSV(hue+0xFF/2, 0xFF, hitSpeed);
    }
  }

  uint8_t sideCollision(Ball &p, unsigned long elapsed) {
    static const float sideR = kMeridian/2.f - 2;
    const linef urLine(kSqrtThree,   1, -sideR*kSqrtThree);
    const linef uLine (0,            1, -sideR*kSqrtThree/2);
    const linef ulLine(-kSqrtThree,  1, -sideR*kSqrtThree);
    const linef dlLine(-kSqrtThree, -1, -sideR*kSqrtThree);
    const linef dLine (0,           -1, -sideR*kSqrtThree/2);
    const linef drLine(kSqrtThree,  -1, -sideR*kSqrtThree);

    // u,ur,dr,d,dl,ul order, matches clockwise from px 0 hexaSide order
    const linef lines[] = {uLine, urLine, drLine, dLine, dlLine, ulLine};
    const float elasticity = 0.95f + constrain(p.velocity.length()*elapsed/6 - 0.019, 0, 0.09f);

    uint8_t sidesHit = 0;
    // Iterate to handle corner collision
    for (int it = 0; it < 3; ++it) {
      bool collided = false;
      for (int w = 0; w < 6; ++w) {
        const auto &wallLine = lines[w];
        float dist = wallLine.A * p.pos.x + wallLine.B * p.pos.y + wallLine.C;
        if (dist > 0) {
          float nLenSq = wallLine.A * wallLine.A + wallLine.B * wallLine.B;

          // Reflect position back inside (mirror across wall)
          p.pos.x -= 2 * wallLine.A * dist / nLenSq;
          p.pos.y -= 2 * wallLine.B * dist / nLenSq;

          // Reflect velocity only if moving outward
          float vDotN = p.velocity.x * wallLine.A + p.velocity.y * wallLine.B;
          if (vDotN > 0) {
            p.velocity.x -= 2 * wallLine.A * vDotN / nLenSq * elasticity;
            p.velocity.y -= 2 * wallLine.B * vDotN / nLenSq * elasticity;
          }
          collided = true;
          sidesHit |= 1 << w;
          break; // re-check all
        }
      }
      if (!collided) {
        break;
      }
    }
    return sidesHit;
  }

  unsigned long lastUpdate = 0;
  virtual void update() {
    ctx.leds.fadeToBlackBy(12);

    int32_t elapsed = (lastUpdate > 0 ? millis() - lastUpdate : 1);
    lastUpdate = millis();

    if (boomStart != 0) {
      unsigned long boomRuntime = millis() - boomStart;
      auto p = Phaser()
        .anim(250, [this](Phase ph) {
          ctx.leds.fill_solid(CRGB::Black);
        })
        .anim(300, [this](Phase ph) {
          float p = ph.progress();
          float expand = (1.0f - p) * (1.0f - p);
          stellate(24.0f * expand, 1.0f);
        })
        .complete([this](Phase) {
          boomStart = 0;
        });
      p.run(min(boomRuntime, p.duration()));

      if (boomRuntime < p.duration()) {
        return;
      };
    }

    // update ball position from velocity and elapsed time
    std::optional<PixelIndex> pxopt = axial.indexAtRect(p.pos);
    if (!pxopt.has_value()) {
      logf("You win! Ball at pos (%f, %f) is out of bounds!", p.pos.x, p.pos.y);
      ctx.leds.fill_solid(CRGB::Black);
      boomStart = millis();

      p.pos.x = 0;
      p.pos.y = 0;
      p.velocity.x = 0;
      p.velocity.y = 0;
      return;
    }
    
    PixelIndex px = pxopt.value();
    auto agmt = MotionManager::motionFrame.agmt;
    vectorf accelVector = accelerationAtPixelIndex(px, agmt);

    const float accelPreScale = 3600; // tuned
    vectorf scaledAccel = accelVector * elapsed / accelPreScale / MotionManager::accelToGScale;
    p.velocity.x += scaledAccel.x;
    p.velocity.y += scaledAccel.y;

    // Coriolis: deflects ball path during rotation
    const float coriolisScale = 1.0f;
    const float coriolisK = coriolisScale / (MotionManager::gyrToRadScale * 1000.0f);
    float coriolisF = agmt.gyr.axes.z * elapsed * coriolisK;
    p.velocity.x +=  coriolisF * p.velocity.y;
    p.velocity.y += -coriolisF * p.velocity.x;

    // Friction: linear approximation of exp(-k*dt)
    const float frictionCoeff = 0.9f;
    const float frictionK = frictionCoeff / 1000.0f;
    float dampFactor = max(0.0f, 1.0f - elapsed * frictionK);
    p.velocity.x *= dampFactor;
    p.velocity.y *= dampFactor;

    p.pos += p.velocity * elapsed;
    uint8_t sidesHit = sideCollision(p, elapsed);

    uint8_t hue = constrain(3000 * p.velocity.length() - 30, 0, 224);
    
    // 16-mult integer optimizations
    int16_t bx16 = (int16_t)(p.pos.x * 16);
    int16_t by16 = (int16_t)(p.pos.y * 16);
    for (PixelIndex px = 0; px < LED_COUNT; ++px) {
      vectorf r = axial.rectFromPixelIndex(px);
      // Hex-shaped integer distance, pulling out the sqrt(3) ~= 111/64
      uint16_t adx = abs(bx16 - (int16_t)(r.x * 16));
      uint16_t ady = abs(by16 - (int16_t)(r.y * 16));
      uint32_t hexD = max(ady * 2, (adx * 111 >> 6) + ady);
      uint16_t size = 50; // diameter 2*sqrt(3)*16 ~= 55
      if (hexD >= size) { 
        continue;
      }
      uint8_t brightness = 255 - (uint8_t)(hexD * 255/size);
      int hueShift = hexD * size >> 8;

      CRGB c = CHSV(max(0, hue - hueShift), 0xFF, 0xFF);
      c = c.scale8(brightness);
      ctx.point(px, c, blendBrighten);
    }
    for (int i = 0; i < 6; ++i) {
      if (sidesHit & (1 << i)) {
        sideHit(p, i, hue, elapsed);
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

  // Rotate vector v by quaternion q: v' = v + 2w*(q×v) + 2*(q×(q×v))
  vectorf quatRotate(const Quaternion &q, vectorf v) {
    // t = 2 * cross(q.xyz, v)
    float tx = 2.0f * (q.y * v.z - q.z * v.y);
    float ty = 2.0f * (q.z * v.x - q.x * v.z);
    float tz = 2.0f * (q.x * v.y - q.y * v.x);
    // v' = v + w*t + cross(q.xyz, t)
    return vectorf(
      v.x + q.w * tx + (q.y * tz - q.z * ty),
      v.y + q.w * ty + (q.z * tx - q.x * tz),
      v.z + q.w * tz + (q.x * ty - q.y * tx)
    );
  }

  void update() {
    ctx.leds.fill_solid(CRGB::Black);

    // we're actually counterrotating a tetrahedron mmkay
    Quaternion q = MotionManager::motionFrame.quat;
    q.z = -q.z;

    float r = kMeridian/2-1;
    unsigned timeOffset = millis() / 50;

    // Regular tetrahedron with vertex pointing 'up' when flat
    constexpr float sq2_3 = 0.9428090f;  // 2*sqrt(2)/3
    constexpr float sq6_3 = 0.8164966f;  // sqrt(6)/3
    constexpr float third = 1.0f / 3.0f;
    const vectorf baseVerts[4] = {
      {0,          0,      -1},      // v0: apex
      {sq2_3,      0,       third},  // v1: base front
      {-sq2_3/2,   sq6_3,   third},  // v2: base left
      {-sq2_3/2,  -sq6_3,   third},  // v3: base right
    };

    // Rotate and scale vertices
    vectorf verts[4];
    for (int i = 0; i < 4; i++) {
      verts[i] = quatRotate(q, baseVerts[i]) * r;
    }

    constexpr uint8_t edges[6][2] = {
      {0,1}, {0,2}, {0,3}, {1,2}, {1,3}, {2,3}
    };

    uint16_t yawBytes = max(0, min(0x1FF, (int)((fabsf(q.w) + fabsf(q.x) + fabsf(q.y) + fabsf(q.z)) * 0x1FF/4)));

    for (int e = 0; e < 6; e++) {
      vectorf &a = verts[edges[e][0]];
      vectorf &b = verts[edges[e][1]];

      // tweak brightness based on average z of endpoints
      float avgZ = (a.z + b.z) / (2.0f * r);  // normalized to [-1, 1]
      uint8_t brightness = 100 + (uint8_t)(155 * (avgZ + 1.0f) / 2.0f); // 100..255

      vectorf pa(a.x, a.y, 0);
      vectorf pb(b.x, b.y, 0);
      fAxial ax1 = axial.rectToHex(pa, 1.0);
      fAxial ax2 = axial.rectToHex(pb, 1.0);

      uint16_t edgeOffset = e * 0x1FF / 6;
      hexline(ctx, ax1, ax2, [this, yawBytes, timeOffset, edgeOffset, brightness] (uint8_t progress) {
        CRGB c = getMirroredPaletteColor(timeOffset + yawBytes + edgeOffset + progress);
        c.nscale8(brightness);
        return c;
      });
    }
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
    
    const int maxHexRadius = (kMeridian/2-2);
    const int minHexRadius = -3;
    const float maxZ = 9000.;
    avgZ = min(maxZ, (10*avgZ+agmt.acc.axes.z)/11.f);
    const float maxLineRadius = kMeridian/2+2;
    float lineRadius = maxLineRadius - (maxLineRadius+2) * abs(avgZ) / maxZ;
    float hexRadius = minHexRadius + (maxHexRadius-minHexRadius) * abs(avgZ) / maxZ;

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
        
        // When we rotate the drawn hexagon at correct angles, there is an aliasing effect where all 6 lines move to the next pixel at the same time
        // causing a visible flicker. shifting each vertex slightly spreads out the next-pixel jumps across different frames and reduces the flicker.
        float vertexBump = 0.01*i;

        vectorT<float> pt1 = {hexRadius*cosf(ptTheta+spinTheta+vertexBump), hexRadius*sinf(ptTheta+spinTheta+vertexBump)};
        vectorT<float> pt2 = {hexRadius*cosf(ptTheta2+spinTheta+vertexBump), hexRadius*sinf(ptTheta2+spinTheta+vertexBump)};
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

class SoundPattern : public Pattern, public FFTReceiver {
public:
  unsigned long lastLevelThreshChange{0};
  int minFFTLevelThreshold{3};
  int fftLevelThreshold{minFFTLevelThreshold};
  int autoGainAdjustmentInterval{600};

  SoundPattern() : FFTReceiver(fftProcessing) {
    // stop main loop from lowering framerate when we have nothing to draw, since that results in visibly-delayed response to sounds
    fc.takeFPSAssertion(); 
  }
  ~SoundPattern() {
    fc.releaseFPSAssertion();
  }
  void autoGainUpdate() {
    FFTFrame frame = fftProcessing.getDataFrame();
    unsigned long mils = millis();

    int maxFrameValue = 0;
    int32_t sumFrameValue = 0;
    for (int i = 0 ; i < frame.size; ++i) {
      if (frame.smoothSpectrum[i] > maxFrameValue) {
        maxFrameValue = frame.smoothSpectrum[i];
      }
      sumFrameValue += frame.smoothSpectrum[i];
    }
    int avgFrameValue = sumFrameValue/frame.size;

    int litCount{0};
    for (int i = 0 ; i < LED_COUNT; ++i) {
      litCount += ctx.leds[i] ? 1 : 0;
    }
    /* latch-ditch auto gain:
     * slowly adjust threshold for drawing if to approach the average levels
     * quickly move threshold for drawing if we're over- or under-drawing
     * temporarily adjust thresholds at a fast interval at the start of pattern running to find a baseline
    */
   bool overDrawing = litCount > 95*LED_COUNT/100;
   bool underDrawing = litCount < 2*LED_COUNT/10;
   int adjustmentInterval = (runTime() > 3000 ? autoGainAdjustmentInterval : autoGainAdjustmentInterval/6);
   if ((overDrawing || fftLevelThreshold < avgFrameValue) && mils - lastLevelThreshChange > adjustmentInterval) {
      fftLevelThreshold++;
      if (overDrawing) {
        fftLevelThreshold += max(0, (maxFrameValue - fftLevelThreshold) / 20);
      }
      // logf("SoundPattern litCount = %i, frame value avg=%i,max=%i, fftLevelThreshold up to %i", litCount, avgFrameValue, maxFrameValue, fftLevelThreshold);
      lastLevelThreshChange = mils;
    } else if (fftLevelThreshold > minFFTLevelThreshold && (underDrawing || fftLevelThreshold > avgFrameValue) && mils - lastLevelThreshChange > adjustmentInterval) {
      fftLevelThreshold--;
      if (underDrawing) {
        fftLevelThreshold = max(minFFTLevelThreshold, fftLevelThreshold + min(0, (maxFrameValue - fftLevelThreshold) / 10));
      }
      // logf("SoundPattern litCount = %i, frame value avg=%i,max=%i, fftLevelThreshold down to %i", litCount, avgFrameValue, maxFrameValue, fftLevelThreshold);
      lastLevelThreshChange = mils;
    }
  }
};

class SoundDroplets : public SoundPattern, public PaletteRotation<CRGBPalette256> {
  HexaShells shells;
  CRGB cs[LED_COUNT] = {0}; // scratch space
  unsigned long lastFlow = 0;
  unsigned long lastLevelThreshChange;
public:
  int dropletSize;

  SoundDroplets(int size) : dropletSize(size) {
    minBrightness = 20;
  }

  void flowDroplets(int i, int i2) {
    // This sub-pixel flow algorithm leaves a lot of r,g,&b residue pixels during fadedown
    const int kFlow = 5;//%
    const int kEff = 80;//%
    const int minLoss = 1;

    // calculate flows from og leds, set in scratch
    CRGB led1 = ctx.leds[i];
    CRGB led2 = ctx.leds[i2];
    for (uint8_t sp = 0; sp < 3; ++sp) { // each subpixel
      uint8_t *refSp = NULL;
      uint8_t *srcSp = NULL;
      uint8_t *dstSp = NULL;
      if (led1[sp] < led2[sp]) {
        refSp = &led2[sp];
        srcSp = &cs[i2][sp];
        dstSp = &cs[i][sp];
      } else if (led1[sp] > led2[sp] ) {
        refSp = &led1[sp];
        srcSp = &cs[i][sp];
        dstSp = &cs[i2][sp];
      }
      if (srcSp && dstSp) {
        uint8_t flow = min(*srcSp, min(*refSp * kFlow/100, 0xFF - *dstSp));
        *dstSp += flow * kEff/100;
        *srcSp = max(0, *srcSp - max(minLoss, flow));
      }
    }
  }

  void makeDroplet(PixelIndex px, int size, uint8_t phase, uint8_t brightness, uint8_t gradientDropoff=0x7F) {
    CRGB color = getPaletteColor(phase, brightness);
    if (size > 0) {
      ctx.leds[px] = color;
    }
    if (size > 1) {
      HexaShells droplet(px, size);
      for (int s = 1; s < size; ++s) {
        brightness = scale8(brightness, gradientDropoff);
        color = getPaletteColor(phase + 20*s, brightness);
        for (int si = 0; si < droplet.shells[s].size(); ++si) {
          auto d = droplet.shells[s][si];
          if (d.has_value()) {
            ctx.leds[d.value()] = color;
          }
        }
      }
    }
  }

  void update() {
    unsigned long mils = millis();
    FFTFrame frame = spectrumFrame();

    for (int s = 0 ; s < min(frame.size, shells.shells.size()); ++s) {
      int32_t level = frame.spectrum[s] - fftLevelThreshold;
      if (level > 0) {
        int shellNum = (s + millis()/1000 + random8()%2) % shells.shells.size();
        int indexInShell = random16()%shells.shells[shellNum].size();
        
        paletteRotate(MotionManager::motionFrame.agmt.gyr.axes.z/1000);

        auto pxOpt = shells.shells[shellNum][indexInShell];
        if (!pxOpt.has_value()) continue;
        PixelIndex px = pxOpt.value();
        uint8_t phase = s*15+millis()/100;
        uint8_t brightness = min(0xFF, level*20);
        makeDroplet(px, dropletSize,phase, brightness);
      }
    }

    int pixelsLit = 0;
    const unsigned int flowInterval = 30;
    if (mils - lastFlow > flowInterval) {
      for (int i = 0; i < LED_COUNT; ++i) {
        cs[i] = ctx.leds[i];
      }
      for (int i = 0; i < LED_COUNT; ++i) {
        Axial ax = axial.axialFromPixelIndex(i);
        std::optional<PixelIndex> other;
        other = axial.indexAtAxial(ax + Axial(1,0));
        if (other.has_value()) flowDroplets(i, other.value());
        other = axial.indexAtAxial(ax + Axial(-1,1));
        if (other.has_value()) flowDroplets(i, other.value());
        other = axial.indexAtAxial(ax + Axial(0,1));
        if (other.has_value()) flowDroplets(i, other.value());
      }
      for (int i = 0; i < LED_COUNT; ++i) {
        ctx.leds[i] = cs[i];
      }
      lastFlow  = mils;
    }
    autoGainUpdate();
  }
  const char *description() {
    return "SoundDroplets";
  }
};

class SparkleDroplets : public SoundDroplets {
public:
  SparkleDroplets() : SoundDroplets(1) { }
  const char *description() {
    return "SparkleDroplets";
  }
};

class BlobDroplets : public SoundDroplets {
public:
  BlobDroplets() : SoundDroplets(2) { }
  const char *description() {
    return "BlobDroplets";
  }
};

class SoundBits : public SoundPattern, public PaletteRotation<CRGBPalette256> {
  HexaShells shells;
public:
  ParticleSim<LED_COUNT> particles;

  int bitLoudZoom = 70;

  SoundBits() : particles(ledgraph, ctx, 0, 0, 1200, {clockwise, counterclockwise}) {
    minBrightness = 20;
    particles.setFadeUpDistance(1);
    particles.handleUpdateParticle = [this](Particle &bit, uint8_t index) {
      if (bit.age() > bit.lifespan/2) {
        bit.brightness = min(0xFF, max(0, (int)(0xFF - 0xAF * (bit.age()-bit.lifespan/2) / (bit.lifespan-bit.lifespan/2))));
      }
      if (bit.speed > bitLoudZoom - bitLoudZoom * bit.age() / bit.lifespan) {
        bit.speed-=2;
      }
    };
  }

  vector32 gyrAccum32;
  
  void update() {
    unsigned long mils = millis();
    
    auto agmt = MotionManager::motionFrame.agmt;
    gyrAccum32 += vector16(agmt.gyr.axes.x/100, agmt.gyr.axes.y/100, agmt.gyr.axes.z/100); // drop low order noisy data
    
    paletteRotate(MotionManager::motionFrame.agmt.gyr.axes.z/1000);

    FFTFrame frame = spectrumFrame();
    for (int s = 0 ; s < min(frame.size, shells.shells.size()); ++s) {
      int32_t level = frame.spectrum[s] - fftLevelThreshold;
      if (level > 0 && particles.particles.size() < 255) {
        int shellNum = (s + millis()/1000 + random8()%2 + gyrAccum32.x/200) % shells.shells.size();
        int indexInShell = random16()%shells.shells[shellNum].size();
        
        unsigned maxlifespan = 300;
        Particle &p = particles.addParticle();
        p.px = shells.shells[shellNum][indexInShell].value();
        p.lifespan = max(1, min(maxlifespan, maxlifespan * level/30));
        uint8_t phase = s*15+millis()/100;
        uint8_t brightness = min(0xFF, level*10);
        p.color = getPaletteColor(phase, brightness);
        p.speed = min(bitLoudZoom, 3*level);
      }
      autoGainUpdate();
    }
    particles.update();
  }

  const char *description() {
    return "SoundBits";
  }
};

/* ------------------------------------------------------------------------------- */

class ChargingPattern : public Pattern {
public:
  int lastStateOfCharge = 0;
  int animateFromSOC = 0;
  unsigned long lastValueChange;

  ChargingPattern() : lastValueChange(millis()) {
    btlogf("[t=%lu] ChargingPattern start: batteryInitialized=%i soc=%u%% flags=%X detected=%i",
           millis(), powerState.batteryInitialized, batteryData.stateOfCharge, batteryData.flags,
           batteryData.batteryDetected());
  }
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
            if (px == LED_COUNT/2 && (!animatingPowerOn || progress() < 0.2)) {
              // hack to keep the final dot at a consistent brightness at the end, as well as after resuming a canceled power-on animation
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

class BlinkIdentifyPattern : public Pattern {
  const int blinkTime = 900;
  HexaShells hexaShells;
  void update() {
    unsigned long rt = runTime();
    ctx.leds.fadeToBlackBy(20);
    int shell = hexaShells.shells.size() * triwave8(0xFF * rt / (blinkTime/3)) / 0xFF;
    shell = min(hexaShells.shells.size(), shell);
    for (int i = 0; i < hexaShells.shells[shell].size(); ++i) {
      ctx.leds[hexaShells.shells[shell][i].value()] = CRGB::Blue;
    }
    if (rt >= blinkTime) {
      stop();
    }
  }
  const char *description() {
    return "BlinkIdentifyPattern";
  }
};

#endif
