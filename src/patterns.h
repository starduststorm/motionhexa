#ifndef PATTERN_H
#define PATTERN_H

#include <FastLED.h>
#include <vector>
#include <functional>
#include <optional>

#include "util.h"
#include "palettes.h"
#include "ledgraph.h"
#include "drawing.h"
#include "MotionManager.h"

class Pattern {
private:  
  long startTime = -1;
  long stopTime = -1;
  long lastUpdateTime = -1;
public:
  DrawingContext ctx;
  virtual ~Pattern() { }

  void start() {
    logf("Starting %s", description());
    startTime = millis();
    stopTime = -1;
    setup();
  }

  void loop() {
    update();
    lastUpdateTime = millis();
  }

  virtual bool wantsToIdleStop() {
    return true;
  }

  virtual bool wantsToRun() {
    // for idle patterns that require microphone input and may opt not to run if there is no sound
    return true;
  }

  virtual void setup() { }

  void stop() {
    logf("Stopping %s", description());
    startTime = -1;
  }

  virtual void update() { }
  
  virtual const char *description() = 0;

public:
  bool isRunning() {
    return startTime != -1;
  }

  unsigned long runTime() {
    return startTime == -1 ? 0 : millis() - startTime;
  }

  unsigned long frameTime() {
    return (lastUpdateTime == -1 ? 0 : millis() - lastUpdateTime);
  }
};

/* ------------------------------------------------------------------------------------------------------ */

// a lil patternlet that can be instantiated to run bits
class BitsFiller {
public:
  typedef enum : uint8_t { random, priority, split } FlowRule;
  typedef enum : uint8_t { maintainPopulation, manualSpawn } SpawnRule;
 
  struct Bit {
    friend BitsFiller;
  private:
    unsigned long birthmilli;
    bool firstFrame = true;
  public:
    uint8_t colorIndex; // storage only
    
    PixelIndex px;
    EdgeTypesQuad directions;
    
    unsigned long lifespan;

    CRGB color;
    uint8_t brightness = 0xFF;

    Bit(int px, EdgeTypesQuad directions, unsigned long lifespan) 
      : px(px), directions(directions), lifespan(lifespan) {
      reset();
    }

    void reset() {
      birthmilli = millis();
      color = CHSV(random8(), 0xFF, 0xFF);
    }

    unsigned long age() {
      return min(millis() - birthmilli, lifespan);
    }
protected:
    unsigned long exactAge() {
      return millis() - birthmilli;
    }
  };

private:

  DrawingContext &ctx;

  unsigned long lastTick = 0;
  unsigned long lastMove = 0;
  unsigned long lastBitSpawn = 0;

  uint8_t spawnLocation() {
    if (spawnPixels) {
      return spawnPixels->at(random8()%spawnPixels->size());
    }
    return random16()%LED_COUNT;
  }

  Bit &makeBit(Bit *fromBit=NULL) {
    // the bit directions at the BitsFiller level may contain multiple options, choose one at random for this bit
    EdgeTypesQuad directionsForBit = {0};

    for (int n = 0; n < 4; ++n) {
      uint8_t bit[EdgeTypesCount] = {0};
      uint8_t bitcount = 0;
      for (int i = 0; i < EdgeTypesCount; ++i) {
        uint8_t byte = bitDirections.quad >> (n * EdgeTypesCount);
        if (byte & 1 << i) {
          bit[bitcount++] = i;
        }
      }
      if (bitcount) {
        directionsForBit.quad |= 1 << (bit[random8()%bitcount] + n * EdgeTypesCount);
      }
    }

    if (fromBit) {
      bits.emplace_back(*fromBit);
    } else {
      bits.emplace_back(spawnLocation(), directionsForBit, lifespan);
    }
    return bits.back();
  }

  void killBit(uint8_t bitIndex) {
    handleKillBit(bits[bitIndex]);
    bits.erase(bits.begin() + bitIndex);
  }

  void splitBit(Bit &bit, PixelIndex toIndex) {
    Bit &split = makeBit(&bit);
    split.px = toIndex;
  }

  bool isIndexAllowed(PixelIndex index) {
    if (allowedPixels) {
      return allowedPixels->end() != allowedPixels->find(index);
    }
    return true;
  }

  vector<Edge> edgeCandidates(PixelIndex index, EdgeTypesQuad bitDirections) {
    vector<Edge> nextEdges;
    switch (flowRule) {
      case priority: {
        auto adj = ledgraph.adjacencies(index, bitDirections);
        for (auto edge : adj) {
          // logf("edgeCandidates: index %i has %i adjacencies matching directions %i (%i,%i)", 
              // index, adj.size(), (int)bitDirections.pair, bitDirections.edgeTypes.first, bitDirections.edgeTypes.second);
          // loglf("checking if adj index %i is allowed for edge from %i types %i...", (int)edge.to, (int)edge.from, (int)edge.types);
          if (isIndexAllowed(edge.to)) {
            nextEdges.push_back(edge);
            break;
          }
        }
        break;
      }
      case random:
      case split: {
        auto adj = ledgraph.adjacencies(index, bitDirections);
        for (auto a : adj) {
          if (isIndexAllowed(a.to)) {
            nextEdges.push_back(a);
          }
        }
        if (flowRule == split) {
          if (nextEdges.size() == 1) {
            // flow normally if we're not actually splitting
            nextEdges.push_back(nextEdges.front());
          } else {
            // split along all allowed split directions, or none if none are allowed
            for (Edge nextEdge : nextEdges) {
              if (splitDirections & nextEdge.types) {
                nextEdges.push_back(nextEdge);
              }
            }
          }
        } else if (nextEdges.size() > 0) {
          // FIXME: EdgeType::random behavior doesn't work right with the way fadeUp is implemented
          nextEdges.push_back(nextEdges.at(random8()%nextEdges.size()));
        }
        break;
      }
    }
    // TODO: does not handle duplicates in the case of the same vertex being reachable via multiple edges
    assert(nextEdges.size() <= 4, "no pixel in this design has more than 4 adjacencies but index %i had %u", index, nextEdges.size());
    return nextEdges;
  }

  bool flowBit(uint8_t bitIndex) {
    vector<Edge> nextEdges = edgeCandidates(bits[bitIndex].px, bits[bitIndex].directions);
    if (nextEdges.size() == 0) {
      // leaf behavior
      logf("no path for bit");
      killBit(bitIndex);
      return false;
    } else {
      bits[bitIndex].px = nextEdges.front().to;
      for (unsigned i = 1; i < nextEdges.size(); ++i) {
        splitBit(bits[bitIndex], nextEdges[i].to);
      }
    }
    return true;
  }

public:
  void dumpBits() {
    logf("--------");
    logf("There are %i bits", bits.size());
    for (unsigned b = 0; b < bits.size(); ++b) {
      Bit &bit = bits[b];
      logf("Bit %i: px=%i, birthmilli=%lu, colorIndex=%u", b, bit.px, bit.birthmilli, bit.colorIndex);
      Serial.print("  Directions: 0b");
      for (int i = 2*EdgeTypesCount - 1; i >= 0; --i) {
        Serial.print(bit.directions.quad & (1 << i));
      }
      Serial.println();
    }
    logf("--------");
  }

  vector<Bit> bits;
  uint8_t maxSpawnBits;
  uint8_t maxBitsPerSecond = 0; // limit how fast new bits are spawned, 0 = no limit
  uint8_t speed; // in pixels/second
  EdgeTypesQuad bitDirections;

  unsigned long lifespan = 0; // in milliseconds, forever if 0

  FlowRule flowRule = random;
  SpawnRule spawnRule = maintainPopulation;
  uint8_t fadeUpDistance = 0; // fade up n pixels ahead of bit motion
  EdgeTypes splitDirections = EdgeType::all; // if flowRule is split, which directions are allowed to split
  
  const vector<PixelIndex> *spawnPixels = NULL; // list of pixels to automatically spawn bits on
  const set<PixelIndex> *allowedPixels = NULL; // set of pixels that bits are allowed to travel to

  function<void(Bit &)> handleNewBit = [](Bit &bit){};
  function<void(Bit &)> handleUpdateBit = [](Bit &bit){};
  function<void(Bit &)> handleKillBit = [](Bit &bit){};

  BitsFiller(DrawingContext &ctx, uint8_t maxSpawnBits, uint8_t speed, unsigned long lifespan, vector<EdgeTypes> bitDirections)
    : ctx(ctx), maxSpawnBits(maxSpawnBits), speed(speed), lifespan(lifespan) {
      this->bitDirections = MakeEdgeTypesQuad(bitDirections);
    bits.reserve(maxSpawnBits);
  };

  void fadeUpForBit(Bit &bit, PixelIndex px, int distanceRemaining, unsigned long lastMove) {
    vector<Edge> nextEdges = edgeCandidates(px, bit.directions);

    unsigned long mils = millis();
    unsigned long fadeUpDuration = 1000 * fadeUpDistance / speed;
    for (Edge edge : nextEdges) {
      unsigned long fadeTimeSoFar = mils - lastMove + distanceRemaining * 1000/speed;
      uint8_t progress = 0xFF * fadeTimeSoFar / fadeUpDuration;

      CRGB existing = ctx.leds[edge.to];
      CRGB blended = blend(existing, bit.color, dim8_raw(progress));
      blended.nscale8(bit.brightness);
      ctx.leds[edge.to] = blended;
      
      if (distanceRemaining > 0) {
        fadeUpForBit(bit, edge.to, distanceRemaining-1, lastMove);
      }
    }
  }

  int fadeDown = 4; // fadeToBlackBy units per millisecond
  void update() {
    unsigned long mils = millis();

    ctx.leds.fadeToBlackBy(fadeDown * (mils - lastTick));
    
    if (spawnRule == maintainPopulation) {
      for (unsigned b = bits.size(); b < maxSpawnBits; ++b) {
        if (maxBitsPerSecond != 0 && mils - lastBitSpawn < 1000 / maxBitsPerSecond) {
          continue;
        }
        addBit();
        lastBitSpawn = mils;
      }
    }

    if (mils - lastMove > 1000/speed) {
      for (int i = bits.size() - 1; i >= 0; --i) {
        if (bits[i].firstFrame) {
          // don't flow bits on the first frame. this allows pattern code to make their own bits that are displayed before being flowed
          continue;
        }
        bool bitAlive = flowBit(i);
        if (bitAlive && bits[i].lifespan != 0 && bits[i].exactAge() > bits[i].lifespan) {
          killBit(i);
        }
      }
      if (mils - lastMove > 2000/speed) {
        lastMove = mils;
      } else {
        // This helps avoid time drift, which for some reason can make one device run consistently faster than another
        lastMove += 1000/speed;
      }
    }
    for (Bit &bit : bits) {
      handleUpdateBit(bit);
    }

    for (Bit &bit : bits) {
      CRGB color = bit.color;
      color.nscale8(bit.brightness);
      ctx.leds[bit.px] = color;
    }
    
    if (fadeUpDistance > 0) {
      for (Bit &bit : bits) {
        if (bit.firstFrame) continue;
        // don't show full fade-up distance right when bit is created
        int bitFadeUpDistance = min((unsigned long)fadeUpDistance, speed * bit.age() / 1000);
        if (bitFadeUpDistance > 0) {
          // TODO: can fade-up take into account color advancement?
          fadeUpForBit(bit, bit.px, bitFadeUpDistance - 1, lastMove);
        }
      }
    }

    lastTick = mils;

    for (Bit &bit : bits) {
      bit.firstFrame = false;
    }
  };

  Bit &addBit() {
    Bit &newbit = makeBit();
    handleNewBit(newbit);
    return newbit;
  }

  void removeAllBits() {
    bits.clear();
  }
};

/* ------------------------------------------------------------------------------- */

class TestPattern : public Pattern, PaletteRotation<CRGBPalette256> {
public:
  vector<set<int> > shells = vector<set<int> >();
  TestPattern() {
    maxColorJump = 7;
    secondsPerPalette = 20;

    uint8_t lit[LED_COUNT] = {0};
    shells.emplace_back();
    shells[0].insert(kHexaCenterIndex);
    int topShell = 0;
    for (int s = 0; s < kMeridian/2; ++s) {
      bool litOne = false;
      auto thetopshell = shells[topShell]; // WTF
      for (int px : thetopshell) {
        auto adj = ledgraph.adjacencies(px);
        int thesize = adj.size();
        for (Edge edge : adj) {
          if (!lit[edge.to]) {
            if (shells.size() - 1 == topShell) {
              shells.emplace_back();
            }
            shells[topShell+1].insert(edge.to);
            lit[edge.to] = 1;
            litOne = true;
          }
        }
      }
      topShell++;
      if (!litOne) {
        break;
      }
    }
  }

  void update() {
    for (int s = 0 ; s < shells.size(); ++s) {
      for (int px : shells[s]) {
        uint8_t brightness = beatsin8(60, 0, 255, 0, -beatsin16(2, 250, 350)*s/shells.size());
        brightness = scale8(brightness, brightness);
        // ctx.leds[px] = CHSV(millis()/20+s*10, 0xFF, brightness);
        CRGB c = this->getMirroredPaletteColor(millis()/100 + s*15);
        c = c.scale8(brightness);
        ctx.leds[px] = c;
      }
    }
  }

  const char *description() {
    return "TestPattern";
  }
};

/* ------------------------------------------------------------------------------- */

struct vector16 {
  int16_t x,y;
  vector16() : x(0), y(0) {}
  vector16(int16_t x, int16_t y) : x(x), y(y) {}
  int32_t dot(const vector16 &other) {
    return x*other.x + y*other.y;
  }
  vector16 operator-() {
    return vector16(-x, -y);
  }
  vector16 operator+(const vector16 &other) {
    return vector16(x+other.x, y+other.y);
  }
  vector16 operator-(const vector16 &other) {
    return vector16(x-other.x, y-other.y);
  }
  vector16 operator/(const int16_t divisor) {
    return vector16(x/divisor, y/divisor);
  }
  vector16 &operator=(const vector16 &other) {
    x = other.x;
    y = other.y;
    return *this;
  }
  vector16 &operator+=(const vector16 &other) {
    x += other.x;
    y += other.y;
    return *this;
  }
  vector16 &operator-=(const vector16 &other) {
    x -= other.x;
    y -= other.y;
    return *this;
  }
  vector16 scale8(uint8_t scaleBy) {
    int16_t x = scale16by8(abs(x), scaleBy) * (x < 0 ? -1 : 1);
    int16_t y = scale16by8(abs(x), scaleBy) * (x < 0 ? -1 : 1);
    return vector16(x, y);
  }
};

// Note:
// Unfortunately this pixel layout is such that each pixel has neighboring pixels on the left, right, and diagonals (point-down)
// while the edges of the whole design are on the top, bottom, and diagonals (side down)
// so when doing collision checks, I need to check against two different kinds of hexagons:
// 1. the inner space and 2. the wall space, rotated pi/6 off each other.
// this is the reason for the 8-sided hexagon below.
enum class HexagonBounding : uint8_t {
  interior     = 0,
  right        = 1 << 0, // 1
  topright     = 1 << 1, // 2
  top          = 1 << 2, // 4
  topleft      = 1 << 3, // 8
  left         = 1 << 4, // 16
  bottomleft   = 1 << 5, // 32
  bottom       = 1 << 6, // 64
  bottomright  = 1 << 7, // 128
};
inline HexagonBounding operator|(HexagonBounding lhs, HexagonBounding rhs) {
  using T = std::underlying_type_t <HexagonBounding>;
  return static_cast<HexagonBounding>(static_cast<T>(lhs) | static_cast<T>(rhs));
}
inline HexagonBounding operator&(HexagonBounding lhs, HexagonBounding rhs) {
  using T = std::underlying_type_t <HexagonBounding>;
  return static_cast<HexagonBounding>(static_cast<T>(lhs) & static_cast<T>(rhs));
}
inline HexagonBounding& operator|=(HexagonBounding &lhs, HexagonBounding rhs) {
  lhs = lhs | rhs;
  return lhs;
}

template<unsigned int SIZE>
class PixelPhysics {
public:
  struct Particle {
    PixelIndex index;
    vector16 pos;      // pos within hexagonal inner-particle dof space
    vector16 velocity; // velocity in range (-255, 255)
    Particle() : index(0) {};
    Particle(PixelIndex index, vector16 pos, vector16 velocity) : index(index), pos(pos), velocity(velocity) {};
  };
  vector<Particle *> particles;
  Particle *particleMap[SIZE] = {0}; // map from physical led index to particle
  uint8_t accelScaling;
  uint8_t elasticity;
public:
  PixelPhysics(ItemGeometry<SIZE> &geometry, PixelIndex particleCount, uint8_t accelScaling, uint8_t elasticity) : accelScaling(accelScaling), elasticity(elasticity) {
    particles.reserve(particleCount);
    for (int i = 0; i < particleCount; ++i) {
      PixelIndex index;
      do {
        index = random16()%SIZE;
      } while (particleMap[index] != NULL);
      Particle *p = new Particle();
      particles.emplace_back(p);
      particles[i]->index = index;
      particleMap[index] = p;
    }
  }

  ~PixelPhysics() {
    logf("~PixelPhysics");
    for (Particle *p : particles) {
      delete p;
    }
    logf("~PixelPhysics done");
  }

private:
  struct line16 {
    int16_t x1, y1, x2, y2;
    line16(int16_t x1, int16_t y1, int16_t x2, int16_t y2) : x1(x1), y1(y1), x2(x2), y2(y2) { }
    vector16 normal(bool clockwise=true) {
      int16_t dy = y2-y1, dx = x2-x1;
      return clockwise ? vector16(-dy, dx) : vector16(dy, -dx);
    }
    int32_t A() {
      return y2-y1;
    }
    int32_t B() {
      return x1-x2;
    }
    int32_t C() {
      return y1 * (x2 - x1) - (y2 - y1) * x1;
    }
  };

  optional<line16> wallLineForBound(HexagonBounding bound) {
    // side-down bound
    // int16_t vertexDistance=256, sideDistance=222, yintercept=444, x1=128; // center-to-point 256
    int16_t vertexDistance=222, sideDistance=192, yintercept=384, x1=111; // center-to-point 222
    // disregard running into two walls (corner) at the same time since it's rare (have to land on a line)
    // and in all likelihood we'll get it in the next pass
    // clockwise for correct normal orientation
    if ((bound & HexagonBounding::topright) != HexagonBounding::interior)    return line16( vertexDistance,  0,  x1,  sideDistance);
    if ((bound & HexagonBounding::top) != HexagonBounding::interior)         return line16( x1,  sideDistance,  -x1,  sideDistance);
    if ((bound & HexagonBounding::topleft) != HexagonBounding::interior)     return line16(-x1,  sideDistance,  -vertexDistance,  0);
    if ((bound & HexagonBounding::bottomleft) != HexagonBounding::interior)  return line16(-vertexDistance,  0, -x1, -sideDistance);
    if ((bound & HexagonBounding::bottom) != HexagonBounding::interior)      return line16(-x1, -sideDistance,   x1, -sideDistance);
    if ((bound & HexagonBounding::bottomright) != HexagonBounding::interior) return line16( x1, -sideDistance,   vertexDistance,  0);
    return nullopt;
  }

  HexagonBounding wallHexagonBounding(vector16 p) {
    // int16_t sideDistance=222, yintercept=444; // center to point 256
    int16_t sideDistance=192, yintercept=384; // center to point 222
    HexagonBounding bounds = HexagonBounding::interior;
    bool abovePosDivider = point_above_line(p,  222,128, 0); // divides plane with positive slope through origin
    bool aboveNegDivider = point_above_line(p, -222,128, 0); // divides plane with negative slope through origin
    if (p.y < -sideDistance/* && !abovePosDivider && !aboveNegDivider*/) bounds |= HexagonBounding::bottom;
    if (p.y >  sideDistance/* && aboveNegDivider && abovePosDivider*/) bounds |= HexagonBounding::top;
    if ( point_above_line(p, -222,128,  yintercept)/* && p.y>=0 && !abovePosDivider*/) bounds |= HexagonBounding::topright;
    if (!point_above_line(p,  222,128, -yintercept)/* && p.y<=0 &&  aboveNegDivider*/) bounds |= HexagonBounding::bottomright;
    if (!point_above_line(p, -222,128, -yintercept)/* && p.y<=0 &&  abovePosDivider*/) bounds |= HexagonBounding::bottomleft;
    if ( point_above_line(p,  222,128,  yintercept)/* && p.y>=0 && !aboveNegDivider*/) bounds |= HexagonBounding::topleft;
    return bounds;
  }

  HexagonBounding wallsAdjacentToPixel(PixelIndex index) {
    HexagonBounding bounds = HexagonBounding::interior;
    if (!hexGrid.nodes[index].named.ul && !hexGrid.nodes[index].named.ur) {
      bounds |= HexagonBounding::top;
    }
    if (!hexGrid.nodes[index].named.l && !hexGrid.nodes[index].named.ul) {
      bounds |= HexagonBounding::topleft;
    }
    if (!hexGrid.nodes[index].named.l && !hexGrid.nodes[index].named.dl) {
      bounds |= HexagonBounding::bottomleft;
    }
    if (!hexGrid.nodes[index].named.dl && !hexGrid.nodes[index].named.dr) {
      bounds |= HexagonBounding::bottom;
    }
    if (!hexGrid.nodes[index].named.dr && !hexGrid.nodes[index].named.r) {
      bounds |= HexagonBounding::bottomright;
    }
    if (!hexGrid.nodes[index].named.r && !hexGrid.nodes[index].named.ur) {
      bounds |= HexagonBounding::topright;
    }
    return bounds;
  }

  optional<line16> innerSpaceLineForBound(HexagonBounding bounding) {
    // point-down bound, center-to-point 256
    switch (bounding) {
      // clockwise
      case HexagonBounding::right:       return line16( 222, -128,  222,  128);
      case HexagonBounding::topright:    return line16( 222,  128,  0,    256);
      case HexagonBounding::topleft:     return line16( 0,    256, -222,  128);
      case HexagonBounding::left:        return line16(-222,  128, -222, -128);
      case HexagonBounding::bottomleft:  return line16(-222, -128,  0,   -256);
      case HexagonBounding::bottomright: return line16( 0,   -256,  222, -128);
      case HexagonBounding::interior:
      default: 
        return nullopt;
    }
  }

  vector16 unitMotionAcrossBound(HexagonBounding bound) {
    // point-down bound
    switch (bound) {
      case HexagonBounding::right:       return vector16( 255,  0);
      case HexagonBounding::topright:    return vector16( 111,  192); // 256*sqrt(3)/2 * (cos(pi/3), sin(pi/3))
      case HexagonBounding::topleft:     return vector16(-111,  192);
      case HexagonBounding::left:        return vector16(-255,  0);
      case HexagonBounding::bottomleft:  return vector16(-111, -192);
      case HexagonBounding::bottomright: return vector16( 111, -192);
      case HexagonBounding::interior:
      default:
        return vector16(0, 0);
    }
  }

  inline bool point_above_line(vector16 p, int16_t dy, int16_t dx, int16_t b) {
    return p.y > (dy*p.x + b*dx) / dx;
  }

  HexagonBounding innerSpaceHexagonBounding(vector16 p) {
    // check if given point is in its point-down hexagon-shaped inner particle space
    HexagonBounding bounds = HexagonBounding::interior;
    bool abovePosDivider = point_above_line(p,  128,222, 0); // divides plane with positive slope through origin
    bool aboveNegDivider = point_above_line(p, -128,222, 0); // divides plane with negative slope through origin
    if (p.x < -222 && abovePosDivider && !aboveNegDivider) bounds |= HexagonBounding::left;
    if (p.x >  222 && aboveNegDivider && !abovePosDivider) bounds |= HexagonBounding::right;
    if ( point_above_line(p, -128,222,  255) && p.x>=0 &&  abovePosDivider) bounds |= HexagonBounding::topright;
    if (!point_above_line(p,  128,222, -255) && p.x>=0 && !aboveNegDivider) bounds |= HexagonBounding::bottomright;
    if (!point_above_line(p, -128,222, -255) && p.x<=0 && !abovePosDivider) bounds |= HexagonBounding::bottomleft;
    if ( point_above_line(p,  128,222,  255) && p.x<=0 &&  aboveNegDivider) bounds |= HexagonBounding::topleft;
    return bounds;
  }

  optional<PixelIndex> dstForMotion(Particle &p, HexagonBounding bounding) {
    switch (bounding) {
      case HexagonBounding::right:       return hexGrid[p.index].named.r;
      case HexagonBounding::topright:    return hexGrid[p.index].named.ur;
      case HexagonBounding::topleft:     return hexGrid[p.index].named.ul;
      case HexagonBounding::left:        return hexGrid[p.index].named.l;
      case HexagonBounding::bottomleft:  return hexGrid[p.index].named.dl;
      case HexagonBounding::bottomright: return hexGrid[p.index].named.dr;
      case HexagonBounding::interior:
      default:
        return nullopt;
    }
  }

  void updateParticleAtBound(Particle &p, HexagonBounding checkBound) {
    assert(checkBound != HexagonBounding::interior, "updateParticleAtBound should not get interior");
    HexagonBounding particleContainment = innerSpaceHexagonBounding(p.pos);
    if ((particleContainment & checkBound) != HexagonBounding::interior) {
      // logf("  Particle crossed motion checkBound %i", checkBound);
      optional<PixelIndex> dst = dstForMotion(p, checkBound);
      PixelIndex srcPixel = p.index;
      if (dst.has_value()) {
        // particle moving/colliding
        PixelIndex dstPixel = dst.value();
        // logf("  particle at index %i check dst index %i", srcPixel, dstPixel);
        if (particleMap[dstPixel]) {
          Particle &p2 = *(particleMap[dstPixel]);
          // collision
          // logf("  particle at pixel %i v=(%i,%i) collision with pixel %i v=(%i,%i)", srcPixel, p.velocity.x, p.velocity.y, dstPixel, p2.velocity.x, p2.velocity.y);
          // roll back motion because otherwise p1 may have already skipped past p2
          p.pos -= p.velocity;
          p2.pos -= p2.velocity;
          // convert p1 into p2's coordinate space
          vector16 pos1 = p.pos - unitMotionAcrossBound(checkBound);
          // logf("  pre-collision points in same coordinate space: p1=(%i, %i), p2=(%i, %i)", pos1.x, pos1.y, p2.pos.x, p2.pos.y);
          vector16 dp = vector16(p2.pos.x - pos1.x, p2.pos.y - pos1.y);          
          vector16 dv = p2.velocity - p.velocity;
          // logf("  dp=(%i,%i), dv=(%i,%i)", dp.x, dp.y, dv.x, dv.y);
          int dpDotDv = dp.dot(dv);
          int dpDotDp = dp.dot(dp);
          // logf("  dpDotDv=%i, dpDotDp=%i", dpDotDv, dpDotDp);
          // assert(dpDotDp != 0, "points should not overlap");
          if (dpDotDp != 0) {
            vector16 dv1 = vector16(dp.x*(2 * dpDotDv) / dpDotDp, dp.y*(2 * dpDotDv) / dpDotDp);
            vector16 dv2 = vector16(-dp.x*(2 * -dpDotDv) / dpDotDp, -dp.y*(2 * -dpDotDv) / dpDotDp);
            dv1 = dv1.scale8(elasticity);
            dv2 = dv2.scale8(elasticity);

            // logf("  pre-collision velocities p1=(%i, %i), p2=(%i, %i)", p.velocity.x, p.velocity.y, p2.velocity.x, p2.velocity.y);
            p.velocity += dv1;// = vector16(p.velocity.x + ddx1, p.velocity.y + ddy1);
            p2.velocity += dv2;// = vector16(p2.velocity.x - dp.x*(2 * -dpDotDv) / dpDotDp, p2.velocity.y - dp.y*(2 * -dpDotDv) / dpDotDp);
            // logf("  post-collision velocities p1=(%i, %i), p2=(%i, %i)", p.velocity.x, p.velocity.y, p2.velocity.x, p2.velocity.y);
            
            // roll forward motion?
            p.pos += p.velocity;
            p2.pos += p2.velocity;
          }
        } else {
          // move
          // logf("  particle at index %i move to %i", srcPixel, dstPixel);
          particleMap[srcPixel] = NULL;
          particleMap[dstPixel] = &p;
          p.index = dstPixel;
          p.pos -= unitMotionAcrossBound(checkBound);
        }
      } else {
        // FIXME: this only gets checked if we were about to move to an adjacent pixel, which results in idle bouncing behavior
        // logf("  Particle intersected with wall via checkBound %i", checkBound);
        auto wallsAdjacent = wallsAdjacentToPixel(p.index);
        // logf("  wallsAdjacent result %i", wallsAdjacent);
        assert(wallsAdjacent != HexagonBounding::interior, "no neighbor but not at wall?");
        if (wallsAdjacent != HexagonBounding::interior) {
          auto wallBound = wallHexagonBounding(p.pos);
          wallBound = wallBound & wallsAdjacent;
          if (wallBound != HexagonBounding::interior) {
            line16 line = wallLineForBound(wallBound).value();
            // roll back the particle movement since it crossed a line
            p.pos -= p.velocity;

            // logf("  pre-wall pos (%i, %i), velocity (%i, %i)", p.pos.x, p.pos.y, p.velocity.x, p.velocity.y);

            // v` = v−2*(v⋅n)/(n⋅n)⋅n
            auto normal = line.normal();
            int32_t VDotN = p.velocity.dot(normal);
            int32_t NDotN = normal.dot(normal);

            // get line as Ax + By + C = 0
            int32_t A = line.A();
            int32_t B = line.B();
            int32_t C = line.C();

            // logf("normal = (%i,%i), VDotN = %i, NDotN = %i", normal.x, normal.y, VDotN, NDotN);
            // logf("%i*x+%i*y+%i=0", A, B, C);

            // Find the parameter t where the particle trajectory intersects the line:
            int32_t t_p = -(A * p.pos.x + B * p.pos.y + C);
            int32_t t_q = A * p.velocity.x + B * p.velocity.y;
            // logf("t = %i/%i", t_p, t_q);
            // Check for parallel movement
            // assert(t_q != 0, "t_q should not be 0 if we're doing collision");
            if (t_q != 0) {
                // Compute intersection point
                int16_t x_int = p.pos.x + p.velocity.x * t_p/t_q;
                int16_t y_int = p.pos.y + p.velocity.y * t_p/t_q;
                // logf("intersection = (%i, %i)", x_int, y_int);

                // Reflect the velocity
                p.velocity.x = p.velocity.x - 2 * normal.x * VDotN/NDotN;
                p.velocity.y = p.velocity.y - 2 * normal.y * VDotN/NDotN;

                // Update particle position after the collision
                const int16_t ensureContained = 2;
                p.pos.x = x_int + p.velocity.x * ensureContained * (t_q-t_p)/t_q;
                p.pos.y = y_int + p.velocity.y * ensureContained * (t_q-t_p)/t_q;
                
                // elasticity
                p.velocity.x = scale16by8(abs(p.velocity.x), elasticity) * (p.velocity.x < 0 ? -1 : 1);
                p.velocity.y = scale16by8(abs(p.velocity.y), elasticity) * (p.velocity.y < 0 ? -1 : 1);
            } else {
              logf("No ricochet, is velocity 0?");
            }
            // logf("  post-wall pos (%i, %i), velocity (%i, %i)", p.pos.x, p.pos.y, p.velocity.x, p.velocity.y);
          }
        }
      }
    } else {
      // logf("particle did not cross checkBound %i", checkBound);
    }
    // sanity constraints
    p.pos.x = constrain(p.pos.x, -0xFF, 0xFF);
    p.pos.y = constrain(p.pos.y, -0xFF, 0xFF);
    p.velocity.x = constrain(p.velocity.x, -0xFF, 0xFF);
    p.velocity.y = constrain(p.velocity.y, -0xFF, 0xFF);
  }

public:
  void setPosition(int particleIndex, PixelIndex position) {
  }

  void addParticle() {
  }

  void removeParticle() {
  }

  void clear() {
  }

  inline int16_t scaleAccel(int16_t val) {
    return min((int)0xFF, (int)max((int)-0xFF, (int)(accelScaling * val / 10000)));
  }

  void update(ICM_20948_AGMT_t agmt) {
    // x across hexa (negative when button side down)
    // y vertical on hexa, (negative lipo usb down)
    // z through hexa, (negative leds up)
    // logf("physics accel = %i, %i, %i", agmt.acc.axes.x, agmt.acc.axes.y, agmt.acc.axes.z);
    vector16 accelVector(-scaleAccel(agmt.acc.axes.x), scaleAccel(agmt.acc.axes.y));
    // logf("scaled accel = %i, %i", accelVector.x, accelVector.y);
    assert(abs(accelVector.x) <= 0xFF, "accelVector.x == %i", accelVector.x);
    assert(abs(accelVector.y) <= 0xFF, "accelVector.y == %i", accelVector.y);

    vector<Particle *> lastParticles = particles;
    for (int p = 0; p < particles.size(); ++p) {
      // logf("update motion, consider particle %i", p);
      particles[p]->velocity += accelVector;
      particles[p]->velocity.x = constrain(particles[p]->velocity.x, -0xFF, 0xFF);
      particles[p]->velocity.y = constrain(particles[p]->velocity.y, -0xFF, 0xFF);

      particles[p]->pos += particles[p]->velocity;
      // logf("  p%i now has pos (%i, %i), velocity (%i, %i)", p, particles[p]->pos.x, particles[p]->pos.y, particles[p]->velocity.x, particles[p]->velocity.y);
    }
    for (int p = 0; p < particles.size(); ++p) {
      // logf("check collision, consider particle %i", p);
      updateParticleAtBound(*particles[p], HexagonBounding::right);
      updateParticleAtBound(*particles[p], HexagonBounding::topright);
      updateParticleAtBound(*particles[p], HexagonBounding::topleft);
      updateParticleAtBound(*particles[p], HexagonBounding::left);
      updateParticleAtBound(*particles[p], HexagonBounding::bottomleft);
      updateParticleAtBound(*particles[p], HexagonBounding::bottomright);
    }
  }
};

class BouncyPixels : public Pattern, PaletteRotation<CRGBPalette256> {
public:
  const PixelIndex pixelCount = 50;
  PixelPhysics<LED_COUNT> physics;
  BouncyPixels() : physics(ledgeometry, pixelCount, 0x06, 0xAF) {
    this->prepareTrackedColors(pixelCount);
    minBrightness = 50;
  }
  ~BouncyPixels() {
    this->releaseTrackedColors();
  }

  void update() {
    auto agmt = MotionManager::manager().agmt;
    // logf("physics accel = %i, %i, %i", agmt.acc.axes.x, agmt.acc.axes.y, agmt.acc.axes.z);
    ctx.leds.fill_solid(CRGB::Black);
    physics.update(MotionManager::manager().agmt);
    int i = 0;
    for (PixelPhysics<LED_COUNT>::Particle *p : physics.particles) {
      ctx.leds[p->index] = getTrackedColor(i++);
    }
  }

  const char *description() {
    return "BouncyPixels";
  }
};

#endif
