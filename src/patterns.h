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

// Pattern override for brightness - reset by main each frame
int16_t patternBrightnessOverride = -1;

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
  BaselineStepper smoothStepper;

  void update() {
    constexpr int mult = 1000; // smooth everything with integer math
    const MotionFrame &motion = MotionManager::motionFrame;

    vector32 acc = vector32(motion.acc.x, motion.acc.y, motion.acc.z) * 5;
    const int smoooooth = 10;
    for (int k = smoothStepper.steps(170); k > 0; --k) {
      smoothAcc = (smoooooth * smoothAcc + acc) / (smoooooth+1);
    }

    constexpr int kInverseRootThree = mult*1/sqrt(3);
    AxialT<int32_t> offcenter = center;
    int q = offcenter.q() + smoothAcc.x + kInverseRootThree * smoothAcc.y / mult;
    int r = offcenter.r() - smoothAcc.y;
    offcenter.setQR(q,r);

    int amplitude = amplitudeFrame();

    // frame-constant terms
    const unsigned long mils = millis();
    CRGBPalette256 &palette = getPalette(); // also advances palette rotation once per frame instead of per pixel
    const int32_t pulseBeat = beatsin16(2, 250, 350);
    const uint16_t paletteEvolve = beatsin8(3, 0, kMeridian);

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

      uint8_t brightness = beatsin8(60, 0, 255, 0, -pulseBeat*distance/(kMeridian/2)/mult);
      CRGB c = PaletteRotation<CRGBPalette256>::getMirroredPaletteColor(palette, mils/100 + distance*15/mult + paletteEvolve);
      c = c.scale8(brightness);
      ctx.leds[px] = c;
    }
  }

  const char *description() {
    return "PulseHexaSmooth";
  }
};


// Framerate-invariant wrapped output motion integrator
struct MotionIntegrator {
  const int32_t scale, period, baselineFPS;
  int32_t value = 0; // [0, period)
  int32_t carry = 0; // sub-unit remainder in 1/(1000*scale) units, [0, 1000*scale)
  MotionIntegrator(int32_t scale, int32_t period, int32_t baselineFPS) : scale(scale), period(period), baselineFPS(baselineFPS) {}
  void step(int32_t sample, int32_t frameMS) {
    // frameMS*baselineFPS/1000 baseline frames elapsed, sample/scale per baseline frame. |sample| <= 32767, frameMS <= 100: fits int32.
    const int32_t unit = 1000 * scale;
    carry += sample * frameMS * baselineFPS;
    int32_t whole = carry / unit;
    if (carry < 0 && carry % unit != 0) --whole; // floor, so carry stays non-negative
    carry -= whole * unit;
    value = mod_wrap(value + whole, period);
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
    maxColorJump = 30;
  }

  static constexpr int32_t kBaselineFPS = 90;
  static constexpr int32_t accScale = 1000;
  static constexpr int32_t gyrScale = 200;

  // One integrator per motion term, each wrapped at a common multiple of the moduli update() derives from it:
  //   gyr.x: bandIndex = 2x walks 6 bands of 4096 (period 12288 in x) and phases a beatsin16 (65536)
  //   gyr.y: s*y/8 mod 0x200         gyr.z: z/2 mod 0x200         acc.x: x/4 mod 256
  //   acc.y: bands*y/shellSize mod 256 for every shell size, so 256 * lcm(shell sizes)
  MotionIntegrator gyrX{gyrScale, 3 * 65536, kBaselineFPS};
  MotionIntegrator gyrY{gyrScale, 8 * 0x200, kBaselineFPS};
  MotionIntegrator gyrZ{gyrScale, 2 * 0x200, kBaselineFPS};
  MotionIntegrator accX{accScale, 4 * 256, kBaselineFPS};
  MotionIntegrator accY{accScale, twistPeriod(hexaShells), kBaselineFPS};

  static int32_t twistPeriod(const HexaShells &shells) {
    int64_t l = 1;
    for (auto &shell : shells.shells) {
      int64_t n = shell.size(), a = l, b = n;
      while (b) { int64_t t = a % b; a = b; b = t; }
      l = l / a * n;
    }
    return (int32_t)min<int64_t>(256 * l, INT32_MAX / 2); // 3870720 for the 10-shell hexa
  }

  void update() {
    const MotionFrame &motion = MotionManager::motionFrame;
    // clamp long stalls to not jump the animation
    int32_t frameMS = constrain((int32_t)frameTime(), 0, 100);
    gyrX.step(motion.gyr.x, frameMS);
    gyrY.step(motion.gyr.y, frameMS);
    gyrZ.step(motion.gyr.z, frameMS);
    accX.step(motion.acc.x, frameMS);
    accY.step(motion.acc.y, frameMS);

    int shellCount = hexaShells.shells.size();

    // frame-constant terms
    const unsigned long mils = millis();
    const unsigned long rt = runTime();
    CRGBPalette256 &palette = getPalette(); // also advances palette rotation once per frame instead of per pixel

    const int32_t bandIndex = gyrX.value * 2 + INT16_MAX;
    const int32_t bandRotate = accX.value;
    const int32_t bandTwist = accY.period - accY.value;
    const int bandCounts[] = {0, 1, 2, 3, 6, 9}; // i like this somewhat better than arbitrary band counts
    int32_t bands = bandCounts[(bandIndex / (1<<12)) % ARRAY_SIZE(bandCounts)];
    int32_t withinBand = (bandIndex - (1<<11)) % (1<<12);
    uint8_t bandFadeIn = 0xFF - cos8(0xFF*withinBand / (1<<12));

    const int32_t gyrRotate = gyrZ.value / 2; // getMirroredPaletteColor wraps at 0x200
    const int32_t evolve = (mils/100) % 0x200;
    const int32_t evolveTwist = (mils/500) % 0x200; // reduced before the per-shell multiply so s*mils can't overflow
    const int32_t shellHBeat = beatsin16(3, 0, 0x200, 0, gyrX.value);

    for (int s = 0 ; s < shellCount; ++s) {
      auto &shell = hexaShells.shells[s];
      uint8_t shellSize = shell.size();

      // fade in at start
      const long fadeinDuration = 1000;
      uint8_t shellBrightness = 0xFF;
      if (rt < fadeinDuration) {
        long fadeOverlap = shellCount/2;
        long shellFadeTime = fadeinDuration/(shellCount + fadeOverlap);
        shellBrightness = (rt > s * shellFadeTime ? min(0xFF, 0xFF * (rt - s*shellFadeTime) / (fadeOverlap * shellFadeTime)) : 0);
      }

      int32_t twistFactor = (s * gyrY.value / 8 + s * evolveTwist) % 0x200;
      int32_t shellH = 0x200 * s/shellCount * shellHBeat / 0x200;

      for (int si = 0; si < shellSize; ++si) {
        auto pxOpt = shell[si];
        if (!pxOpt.has_value()) continue;
        PixelIndex px = pxOpt.value();

        uint8_t brightness = lerp8by8(sin8(-bandRotate/4 + bands*(0xFF*si + bandTwist) / shellSize - 0xFF*s/shellCount), 0xFF, bandFadeIn);

        brightness = scale8(brightness, brightness);
        int32_t radialH =  0x200 * si / shellSize;
        CRGB c = PaletteRotation<CRGBPalette256>::getMirroredPaletteColor(palette, gyrRotate + radialH + twistFactor + shellH + evolve);

        c.nscale8(brightness);
        if (shellBrightness != 0xFF) {
          c.nscale8(shellBrightness);
        }
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
    ctx.fadeToBlackBy16(18 * 245 * 256 / 1000);
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
    ctx.fadeToBlackBy16(5 * 245 * 256 / 1000);
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
      return accelerationAtPixelIndex(index, MotionManager::motionFrame);
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
//
// The ball lives in the hexa's own (accelerating, rotating) frame, so everything the hexa does reaches it as a pseudo-force.
// Those are applied at true physical scale (inertiaScale 1: one pixel pitch of hexa travel is one pixel of ball travel), which
// is what gives it mass: shove or spin the hexa and the ball stays put in the room while the walls come to it; flick it into
// a wall and it leaves with the hexa's velocity. Gravity is scaled separately and much weaker, to keep tilt playable.
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
  // hit brightness tuned as per-frame displacement
  static constexpr float kTunedFrameMS = 5.5f;

  // 1g in px/ms^2 at true scale: one rect unit is one pixel pitch
  static constexpr float kGToPxPerMsSq = 9.80665f / (pixelSpacing * 1e-3f) * 1e-6f;
  // fractions of physical scale
  static constexpr float gravityScale = 0.11f; // tilt; the old tuning (1g = 1/3600 px/ms^2)
  static constexpr float inertiaScale = 0.5f;  // linear acceleration, centrifugal, Euler, Coriolis
  // ball-bearing friction: barely any drag, plus a rolling resistance that lets it come to rest on a near-level hexa
  static constexpr float viscousPerSecond = 0.45f;
  static constexpr float rollingResistance = 6e-6f; // px/ms^2; holds the ball still inside ~1.2deg of tilt at gravityScale
  // Walls dampen until a critical collision speed, when they start boosting
  static constexpr float wallBonusStartSpeed = 0.096f;
  static constexpr float wallBonusFullSpeed = 0.6f;

  // even more supercritical
  static constexpr unsigned long kSupercriticalMS = 500;
  static constexpr uint8_t kSupercriticalBrightness = 60;
  static constexpr float kSupercriticalSpeed = wallBonusFullSpeed*1.2;
  static constexpr unsigned long kSupercriticalFadeMS = 120; // tail of the stellation over which the brightness ramps back down
  unsigned long superStart = 0;
  uint8_t superBaseBrightness = 0; // brightness when the ball escaped; the ramp starts here
  uint8_t superBrightness = 0;     // brightness through the boom

  GravityTracker gravityTracker;
  float lastGyrZ = 0; // rad/ms

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

  void sideHit(Ball &p, int w, uint8_t hue) {
    assert(hexaSide(w).size() == 10,"hexa side size");
    uint8_t hitSpeed = constrain(2000 * p.velocity.length()*kTunedFrameMS - 100, 0, 0xFF);
    for (PixelIndex px : hexaSide(w)) {
      ctx.leds[px] = CHSV(hue+0xFF/2, 0xFF, hitSpeed);
    }
  }

  // Walls in rect units, A*x + B*y + C > 0 is outside. u,ur,dr,d,dl,ul order, matches clockwise from px 0 hexaSide order
  static constexpr float kSideR = kMeridian/2.f - 2;
  static constexpr float kInradius = kSideR * kSqrtThree / 2; // every wall's normalized distance from center
  static const linef *walls() {
    static const linef lines[] = {
      linef(0,            1, -kSideR*kSqrtThree/2), // u
      linef(kSqrtThree,   1, -kSideR*kSqrtThree),   // ur
      linef(kSqrtThree,  -1, -kSideR*kSqrtThree),   // dr
      linef(0,           -1, -kSideR*kSqrtThree/2), // d
      linef(-kSqrtThree, -1, -kSideR*kSqrtThree),   // dl
      linef(-kSqrtThree,  1, -kSideR*kSqrtThree),   // ul
    };
    return lines;
  }

  // Pull an escaped ball back along its ray to the center until it sits just inside the walls. A ball that went
  // non-finite goes back to the center instead.
  void projectInside(Ball &b) {
    if (!isfinite(b.pos.x) || !isfinite(b.pos.y) || !isfinite(b.velocity.x) || !isfinite(b.velocity.y)) {
      b.pos = vectorf(0, 0);
      b.velocity = vectorf(0, 0);
      return;
    }
    float reach = 0; // hexagonal norm of the position: how far out along the most-violated wall normal
    for (int w = 0; w < 6; ++w) {
      const linef &l = walls()[w];
      reach = max(reach, (l.A * b.pos.x + l.B * b.pos.y) / sqrtf(l.A * l.A + l.B * l.B));
    }
    const float limit = kInradius * 0.9f;
    if (reach > limit) {
      b.pos *= limit / reach;
    }
  }

  uint8_t sideCollision(Ball &p) {
    const linef *lines = walls();
    const float wallBonus = constrain((p.velocity.length() - wallBonusStartSpeed) / (wallBonusFullSpeed - wallBonusStartSpeed), 0.0f, 1.0f);
    const float elasticity = 0.94f + 0.082f * wallBonus;

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

  unsigned long lastUpdateMicros = 0;
  virtual void update() {
    ctx.fadeToBlackBy16(12 * 180 * 256 / 1000);

    // float ms from micros() (frames are ~1.4ms)
    unsigned long nowMicros = micros();
    const bool firstUpdate = (lastUpdateMicros == 0);
    float elapsed = (firstUpdate ? 1.0f : (nowMicros - lastUpdateMicros) * 1e-3f);
    lastUpdateMicros = nowMicros;
    const MotionFrame &motion = MotionManager::motionFrame;
    const bool resync = (firstUpdate || elapsed > 50 || boomStart != 0);
    if (resync) {
      // stalled, or not simulating: the gravity estimate has missed rotation, start it over
      elapsed = min(elapsed, 1.0f);
      gravityTracker.reset();
    }
    gravityTracker.update(motion, elapsed * 1e-3f);
    const float gyrZ = motion.gyr.z / (MotionManager::gyrToRadScale * 1000.0f); // rad/ms
    const float gyrZDelta = (resync ? 0 : gyrZ - lastGyrZ);
    lastGyrZ = gyrZ;

    if (boomStart != 0) {
      unsigned long boomRuntime = millis() - boomStart;
      patternBrightnessOverride = superBrightness; // the flash gets the supercritical brightness too
      auto p = Phaser()
        .anim(250, [this](Phase ph) {
          ctx.leds.fill_solid(CRGB::Black);
        })
        .anim(300, [this](Phase ph) {
          float p = ph.progress();
          float expand = (1.0f - p) * (1.0f - p);
          stellate(24.0f * expand, 1.0f);
          // ramp down brightness
          unsigned long remaining = ph.duration - ph.elapsed;
          if (remaining < kSupercriticalFadeMS) {
            patternBrightnessOverride = superBaseBrightness + (int)(superBrightness - superBaseBrightness) * (int)remaining / (int)kSupercriticalFadeMS;
          }
        })
        .complete([this](Phase) {
          boomStart = 0;
        });
      p.run(min(boomRuntime, p.duration()));

      if (boomRuntime < p.duration()) {
        return;
      };
    }

    // ball escaped. start boom!
    if (!axial.indexAtRect(p.pos).has_value()) {
      if (superStart == 0) {
        logf("You win! Ball at pos (%f, %f) escaped at %f px/ms", p.pos.x, p.pos.y, p.velocity.length());
        superStart = millis();
        superBaseBrightness = FastLED.getBrightness();
        superBrightness = superBaseBrightness;
      }
      projectInside(p);
    }
    if (superStart != 0) {
      unsigned long superRuntime = millis() - superStart;
      if (superRuntime >= kSupercriticalMS) {
        ctx.leds.fill_solid(CRGB::Black);
        boomStart = millis();
        superStart = 0;
        p.pos = vectorf(0, 0);
        p.velocity = vectorf(0, 0);
        patternBrightnessOverride = superBrightness;
        return;
      }
      const uint8_t target = max(superBaseBrightness, kSupercriticalBrightness);
      superBrightness = superBaseBrightness + (uint8_t)((target - superBaseBrightness) * superRuntime / kSupercriticalMS);
      patternBrightnessOverride = superBrightness;
      // cap the runaway speed so a frame's travel stays well inside the hexa and the wall reflection can do its job
      float speed = p.velocity.length();
      if (speed > kSupercriticalSpeed) {
        p.velocity *= kSupercriticalSpeed / speed;
      }
    }

    
    // Acceleration of the ball relative to the hexa, rect coordinates, px/ms^2. Motion-frame x/y read directly as "the direction
    // things fall" in pixel geometry (see MotionFrame), for the pseudo-force of a shove as much as for gravity.
    const vectorf &gravity = gravityTracker.gravity;
    vectorf linear = gravityTracker.linear(motion);
    float ax = kGToPxPerMsSq * (gravityScale * gravity.x + inertiaScale * linear.x);
    float ay = kGToPxPerMsSq * (gravityScale * gravity.y + inertiaScale * linear.y);

    // The accelerometer reports the hexa's motion at its own position; the ball is somewhere else on a rotating body.
    // r is ball from sensor, in pixels. Centrifugal: w^2 * r.
    static const float sensorX = kHexaMotionPlacement.position.x / (pixelSpacing * 1000.0f);
    static const float sensorY = kHexaMotionPlacement.position.y / (pixelSpacing * 1000.0f);
    float rx = p.pos.x - sensorX, ry = p.pos.y - sensorY;
    ax += inertiaScale * gyrZ * gyrZ * rx;
    ay += inertiaScale * gyrZ * gyrZ * ry;

    // Coriolis, -2w x v: turns the velocity through -2w*dt without changing its length. Half before the other forces and half
    // after; doing it all at one end is a first-order error that shows up as the ball wandering when a shove meets a spin.
    const float theta = inertiaScale * gyrZ * elapsed;
    const float cosT = 1.0f - theta * theta / 2, sinT = theta;
    auto coriolisHalfTurn = [&]() {
      float vx = p.velocity.x, vy = p.velocity.y;
      p.velocity.x =  cosT * vx + sinT * vy;
      p.velocity.y = -sinT * vx + cosT * vy;
    };
    coriolisHalfTurn();

    p.velocity.x += ax * elapsed;
    p.velocity.y += ay * elapsed;

    // Euler: spin the hexa up and the ball stays behind. -(dw/dt) x r integrates over the frame to -dw x r, so the change in
    // rate is applied directly rather than differentiating the gyro.
    p.velocity.x += inertiaScale * gyrZDelta *  ry;
    p.velocity.y += inertiaScale * gyrZDelta * -rx;

    coriolisHalfTurn();

    // Friction: viscous as a linear approximation of exp(-k*dt), then rolling resistance as a constant deceleration that
    // stops the ball rather than reversing it
    float dampFactor = max(0.0f, 1.0f - elapsed * viscousPerSecond / 1000.0f);
    float speed = p.velocity.length();
    float rolledSpeed = max(0.0f, speed * dampFactor - rollingResistance * elapsed);
    float speedFactor = (speed > 0 ? rolledSpeed / speed : 0.0f);
    p.velocity.x *= speedFactor;
    p.velocity.y *= speedFactor;

    p.pos += p.velocity * elapsed;
    uint8_t sidesHit = sideCollision(p);
    if (superStart != 0) {
      projectInside(p); // the reflection gives up after three walls; must not escape again
    }

    uint8_t hue = constrain(1000 * p.velocity.length() - 30, 0, 224);
    
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
        sideHit(p, i, hue);
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
      hexline(ctx, ax1, ax2, false, [this, yawBytes, timeOffset, edgeOffset, brightness] (uint8_t progress) {
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
  BaselineStepper smoothStepper;
  void update() {

    const MotionFrame &motion = MotionManager::motionFrame;

    float theta = M_PI+atan2(motion.acc.y, motion.acc.x);
    int flag = 6*(theta+M_PI/12) / (2*M_PI);
    flag = mod_wrap(flag,6);

    const int maxHexRadius = (kMeridian/2-2);
    const int minHexRadius = -3;
    const float maxZ = 9000.;
    for (int k = smoothStepper.steps(195); k > 0; --k) {
      avgZ = min(maxZ, (10*avgZ+motion.acc.z)/11.f);
    }
    const float maxLineRadius = kMeridian/2+2;
    float lineRadius = maxLineRadius - (maxLineRadius+2) * abs(avgZ) / maxZ;
    float hexRadius = minHexRadius + (maxHexRadius-minHexRadius) * abs(avgZ) / maxZ;

    if (lineRadius > kMeridian/2) {
      lastSeenAtHighAngle = flag;
    }

    ctx.fadeToBlackBy16((5 + (hexRadius>0?(int)hexRadius:0)) * 400 * 256 / 1000);

    float scaledGyr = (motion.gyr.z / 6666) / 100000.f;
    spinTheta += frameTime() * dSpin;
  
    if (lineRadius > 0) {
      vectorT<float> pt1 = {lineRadius*cosf(spinTheta), lineRadius*sinf(spinTheta)};
      vectorT<float> pt2 = {lineRadius*-cosf(spinTheta), lineRadius*-sinf(spinTheta)};
      fAxial ax1 = axial.rectToHex(pt1, 1.0);
      fAxial ax2 = axial.rectToHex(pt2, 1.0);

      hexline(ctx, ax1, ax2, true, [this, flag] (uint8_t progress) {
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
        
        hexline(ctx, ax1, ax2, true, [this, i] (uint8_t progress) {
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

class SoundDroplets : public SoundPattern, public PaletteRotation<CRGBPalette256> {
  HexaShells shells;
  CRGB cs[LED_COUNT] = {0}; // scratch space
  unsigned long lastFlow = 0;
  unsigned long lastLevelThreshChange;
public:
  int dropletSize;

  SoundDroplets(int size) : SoundPattern(fftProcessing), dropletSize(size) {
    minBrightness = 20;
    // stop main loop from lowering framerate when we have nothing to draw, since that results in visibly-delayed response to sounds
    fc.takeFPSAssertion();
  }

  ~SoundDroplets() {
    fc.releaseFPSAssertion();
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

  unsigned long lastSpawnCheck = 0;

  void update() {
    unsigned long mils = millis();
    FFTFrame frame = spectrumFrame();

    if (mils - lastSpawnCheck >= 8) {
      lastSpawnCheck = mils;
      for (int s = 0 ; s < min(frame.size, shells.shells.size()); ++s) {
        int32_t level = frame.spectrum[s] - fftLevelThreshold;
        if (level > 0) {
          int shellNum = (s + millis()/1000 + random8()%2) % shells.shells.size();
          int indexInShell = random16()%shells.shells[shellNum].size();

          paletteRotate(MotionManager::motionFrame.gyr.z/1000);

          auto pxOpt = shells.shells[shellNum][indexInShell];
          if (!pxOpt.has_value()) continue;
          PixelIndex px = pxOpt.value();
          uint8_t phase = s*15+millis()/100;
          uint8_t brightness = min(0xFF, level*20);
          makeDroplet(px, dropletSize,phase, brightness);
        }
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

  SoundBits() : SoundPattern(fftProcessing), particles(ledgraph, ctx, 0, 0, 1200, {clockwise, counterclockwise}) {
    minBrightness = 20;
    particles.setFadeUpDistance(1);
    particles.handleUpdateParticle = [this](Particle &bit, uint8_t index) {
      if (bit.age() > bit.lifespan/2) {
        bit.brightness = min(0xFF, max(0, (int)(0xFF - 0xAF * (bit.age()-bit.lifespan/2) / (bit.lifespan-bit.lifespan/2))));
      }
      // slow the bit down toward a threshold that falls to zero over its life
      int threshold = bitLoudZoom - bitLoudZoom * (int)bit.age() / (int)bit.lifespan;
      if (bit.speed > threshold) {
        bit.speed = max(0, (int)bit.speed - speedDecaySteps);
      }
    };
    // stop main loop from lowering framerate when we have nothing to draw, since that results in visibly-delayed response to sounds
    fc.takeFPSAssertion();
  }
  ~SoundBits() {
    fc.releaseFPSAssertion();
  }

  static constexpr int32_t kBaselineFPS = 90;
  MotionIntegrator gyrShell{200, (int32_t)shells.shells.size(), kBaselineFPS};
  unsigned long lastSpawnCheck = 0;
  int speedDecaySteps = 0;
  BaselineStepper decayStepper;

  void update() {
    unsigned long mils = millis();

    // framerate-invariant integration, see MotionIntegrator. frameMS 0 means a sub-ms frame.
    int32_t frameMS = constrain((int32_t)frameTime(), 0, 100);
    speedDecaySteps = decayStepper.steps(125);

    const MotionFrame &motion = MotionManager::motionFrame;
    gyrShell.step(motion.gyr.x/100, frameMS); // drop low order noisy data

    FFTFrame frame = spectrumFrame();
    if (mils - lastSpawnCheck >= 10) {
      lastSpawnCheck = mils;
      paletteRotate(motion.gyr.z/1000);

      for (int s = 0 ; s < min(frame.size, shells.shells.size()); ++s) {
        int32_t level = frame.spectrum[s] - fftLevelThreshold;
        if (level > 0 && particles.particles.size() < 255) {
          int shellNum = (s + mils/1000 + random8()%2 + gyrShell.value) % shells.shells.size();
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
  HexaShells shells;
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
    const auto &outerShell = shells.shells.back();

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
    ctx.fadeToBlackBy16(20 * 245 * 256 / 1000);
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

/* ------------------------------------------------------------------------------- */

class CompassPattern : public Pattern {
  static constexpr float kDotRadius = 6.0f; // hex cells from center
  float dotAngle = 0; // rad, pixel frame; integrates -gyro z (see update) so the dots counter-rotate against the device

  void drawRingDots(CRGB color) {
    for (int i = 0; i < 6; ++i) {
      float a = dotAngle + i * (float)M_PI / 3;
      hexdot(ctx, vectorf(kDotRadius * cosf(a), kDotRadius * sinf(a), 0), true, color);
    }
  }

public:
  void update() {
    ctx.leds.fill_solid(CRGB::Black);
    const MotionFrame &motion = MotionManager::motionFrame;

    if (!Compass::headingValid()) {
      // Counter-rotate: a device turning CCW about +z (positive gyro z in the motion frame; the pixel frame is that
      // rotated 180deg about z, so the sense is the same) makes a world-fixed feature turn CW across the panel.
      float dt = constrain((int32_t)frameTime(), 0, 100) * 1e-3f; // clamp stalls so a hitch can't spin the ring
      dotAngle -= motion.gyr.z / MotionManager::gyrToRadScale * dt;
      dotAngle = fmodf(dotAngle, 2 * (float)M_PI);
      drawRingDots(CRGB::White);
      return;
    }

    // Compass angles are in the motion frame; the pixel geometry frame (rectToHex) is that rotated 180deg about z.
    float a = Compass::northAngleRad() + PI;
    const float r = kMeridian / 2;
    vectorf tip((r-2) * cosf(a), (r-2) * sinf(a), 0);
    vectorf back((r-3) * cosf(a-PI), (r-3) * sinf(a-PI), 0);

    fAxial backHex = axial.rectToHex(back, 1.0);
    fAxial tipHex = axial.rectToHex(tip, 1.0);
    hexline(ctx, backHex, tipHex, true, CRGB::White);

    // Two barbs pointing back from the tip
    const float barbLen = 4.0f;
    const float barbSpread = 15.0f * M_PI / 180.0f;
    for (float s : {+3.0f, -3.0f}) {
      float ba = a + M_PI + s * barbSpread;
      vectorf barb(tip.x + barbLen * cosf(ba), tip.y + barbLen * sinf(ba), 0);
      hexline(ctx, tipHex, axial.rectToHex(barb, 1.0), true, CRGB::White);
    }
  }

  const char *description() {
    return "CompassPattern";
  }
};

#endif
