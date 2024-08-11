#ifndef HEXAPHYSICS_H
#define HEXAPHYSICS_H

#include <vector>
#include <optional>
#include <FastLED.h>

#include "util.h"

using namespace std;

#define DEBUG_PHYSICS 0
#if DEBUG_PHYSICS
#define plogf(format, ...) logf(format, ## __VA_ARGS__)
#else
#define plogf(format, ...)
#endif

typedef uint16_t PixelIndex;

template<typename T>
struct vectorT {
  T x,y;
  vectorT() : x(0), y(0) {}
  vectorT(T x, T y) : x(x), y(y) {}
  int32_t dot(const vectorT<T> &other) {
    return x*other.x + y*other.y;
  }
  virtual const vectorT<T> operator-() {
    return vectorT<T>(-x, -y);
  }
  virtual const vectorT<T> operator+(const vectorT<T> &other) {
    return vectorT<T>(x+other.x, y+other.y);
  }
  virtual const vectorT<T> operator-(const vectorT<T> &other) {
    return vectorT<T>(x-other.x, y-other.y);
  }
  virtual const vectorT<T> operator*(const T multiplicand) {
    return vectorT<T>(x*multiplicand, y*multiplicand);
  }
  virtual const vectorT<T> operator/(const T divisor) {
    return vectorT<T>(x/divisor, y/divisor);
  }
  virtual vectorT<T> &operator=(const vectorT<T> &other) {
    x = other.x;
    y = other.y;
    return *this;
  }
  virtual vectorT<T> &operator+=(const vectorT<T> &other) {
    x += other.x;
    y += other.y;
    return *this;
  }
  virtual vectorT<T> &operator-=(const vectorT<T> &other) {
    x -= other.x;
    y -= other.y;
    return *this;
  }
  vectorT<T> scale8(uint8_t scaleBy) {
    int16_t sx = scale16by8(abs(x), scaleBy) * (x < 0 ? -1 : 1);
    int16_t sy = scale16by8(abs(y), scaleBy) * (y < 0 ? -1 : 1);
    return vectorT<T>(sx, sy);
  }
};

typedef vectorT<int8_t> vector8;
typedef vectorT<int16_t> vector16;
typedef vectorT<int32_t> vector32;

struct line16 {
  int16_t x1, y1, x2, y2;
  line16(int16_t x1, int16_t y1, int16_t x2, int16_t y2) : x1(x1), y1(y1), x2(x2), y2(y2) { }
  vector16 normal(bool clockwise=true) {
    // this vector is not unit length
    int16_t dy = y2-y1, dx = x2-x1;
    return clockwise ? vector16(-dy, dx) : vector16(dy, -dx);
  }
  line16 mergeMidpoints(line16 &oth) {
    return line16((x1+oth.x1)/2, (y1+oth.y1)/2, (x2+oth.x2)/2, (y2+oth.y2)/2);
  }
  // coefficients of line equation Ax + By + C = 0
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
struct UMPoint : vector32 {
  // point operating on integral micrometers
  UMPoint() : vector32() {};
  UMPoint(int32_t x, int32_t y) : vector32(x,y) {};
  UMPoint(const vector32 &v) : UMPoint(v.x, v.y) {}
  UMPoint(const vector32 &&v) noexcept : UMPoint(v.x, v.y) {}
  static UMPoint fromMM(float x, float y) {
    return UMPoint(1000*x, 1000*y);
  }
};

template<typename T>
class HexGrid {
public:
  class HexNode {
    inline void initNeighbors() {
      for (int n = 0; n < 6; ++n) {
        neighbors[n] = nullptr;
      }
    }
    optional<T> _value = nullopt;
    optional<line16> _edgeLine = nullopt;
    public:
    union {
      struct {
        HexNode *ul, *ur, *r, *dr, *dl, *l;
      } named;
      HexNode *neighbors[6];
    };

    HexNode(T val) : _value(val) { 
      initNeighbors();
    }
    HexNode(line16 edgeLine) : _edgeLine(edgeLine) {
      initNeighbors();
    }
    HexNode(const HexGrid<T>::HexNode& oth) {
      _value = oth.value;
      _edgeLine = oth.edgeLine;
      for (int j = 0; j < 6; ++j) {
        neighbors[j] = oth.neighbors[j];
      }
    }
    HexNode(HexGrid<T>::HexNode&& oth) noexcept :
        _value(move(oth.value)),
        _edgeLine(move(oth.edgeLine)) {
          for (int j = 0; j < 6; ++j) {
            neighbors[j] = oth.neighbors[j];
            oth.neighbors[j] = nullptr;
          }
    }

    T data() {
      return _value.value();
    }
    line16 edgeLine(){
      return _edgeLine.value();
    }
    bool isDataNode() {
      return _value.has_value();
    }
    bool isEdgeNode() {
      return _edgeLine.has_value();
    }
    uint8_t neighborCount() {
      int n = 0;
      for (int j = 0; j < 6; ++j) {
        if (neighbors[j]) { n++; }
      }
      return n;
    }
  };
private:
  const T meridian;
  T _valueCount, _totalCount;
  const float spacing;
  
  inline void setPosition(T index, UMPoint pt) {
    logf("setPosition %i = (%i, %i)", index, pt.x, pt.y);
    positions[index] = pt;
  }

  void initConnections(const T kMeridian) {
    assert(kMeridian%2 == 1, "Meridian must be an odd number");
    const T kSidelen = (kMeridian+1) >> 1;
    _valueCount = kMeridian + (kMeridian-kSidelen) * (kSidelen + kMeridian-1); // meridian + 2*(sum of rows from meridian to side)
    const T kMeridianWithEdges = kMeridian+2;
    const T kSidelenWithEdges = (kMeridianWithEdges+1) >> 1;
    _totalCount = kMeridianWithEdges + (kMeridianWithEdges-kSidelenWithEdges) * (kSidelenWithEdges + kMeridianWithEdges-1); // meridian + 2*(sum of rows from meridian to side)

    positions.reserve(_valueCount);
    for (int i = 0 ; i < _valueCount; ++i) {
      positions.emplace_back();
    }

    nodes.reserve(_totalCount);
    for (int i = 0; i < _valueCount; ++i) {
      nodes.emplace_back(new HexNode(i));
      assert(nodes.back()->isDataNode(),"just created node has value");
      assert(nodes.back()->data() == i,"just created node has value");
    }

    int row = 0;
    int rowCounts[kMeridian] = {0};
    for (int r = 0; r<kMeridian; ++r) {
      rowCounts[r] = kSidelen + (r<kMeridian/2 ? r : kMeridian-r-1);
    }
    int rowStarts[kMeridian] = {0};
    for (int r = 0; r<kMeridian; ++r) {
      rowStarts[r] = (r>0 ? rowStarts[r-1] + rowCounts[r-1] : 0);
    }
    for (int i = 0; i < _valueCount; ++i) {
      if (row+1 < kMeridian && i >= rowStarts[row+1]) {
        row++;
      }
      bool topSide = rowCounts[row] < rowCounts[row+1];
      int indexInRow = i - rowStarts[row];
      int rightToLeft = row % 2;
      
      // Compute pixel physical position
      // integral positions given in micrometers relative to center pixel at (0,0)
      if (spacing != 0) {
        const float colSpacing = sin(2*PI/6)*spacing; // 3.3774990747593105 when spacing == 3.9
        int centerRow = kSidelen-1;
        float y = colSpacing * (row - centerRow);
        float x = (rightToLeft ? -1 : 1) * spacing * (indexInRow - rowCounts[row]/2) + (rightToLeft ? 0 : spacing/2);
        // logf("spacing: i=%i, row=%i, indexInRow=%i, rowCounts[row]=%i, rightToLeft=%i, x,y=(%f,%f)", i, row, indexInRow, rowCounts[row], rightToLeft, x, y);
        setPosition(i, UMPoint::fromMM(x,y));
      }

      // Find Neighbors
      if (rightToLeft) {
        if (i > rowStarts[row]) {
          nodes[i]->named.r = nodes[i-1];
          nodes[i-1]->named.l = nodes[i];
        }
      } else {
        if (i < rowStarts[row] + rowCounts[row] - 1) {
          nodes[i]->named.r = nodes[i+1];
          nodes[i+1]->named.l = nodes[i];
        }
      }
      if (row+1 < kMeridian) {
        int indexInRow = i - rowStarts[row];
        bool pastNearHexSide = i > rowStarts[row];
        bool beforeFarHexSide = i < rowStarts[row] + rowCounts[row]-1;
        int topSideCorrection = (topSide ? -1 : 0);
        if (topSide || (!rightToLeft && pastNearHexSide) || (rightToLeft && beforeFarHexSide)) {
          int rightToLeftCorrection = (rightToLeft ? -1 : 0);
          int dl = (rowStarts[row+1] + rowCounts[row+1]) - indexInRow + topSideCorrection + rightToLeftCorrection;
          nodes[i]->named.dl = nodes[dl];
          nodes[dl]->named.ur = nodes[i];
        }
        if (topSide || (!rightToLeft && beforeFarHexSide) || (rightToLeft && pastNearHexSide)) {
          int rightToLeftCorrection = (rightToLeft ? 0 : -1);
          int dr = (rowStarts[row+1] + rowCounts[row+1]) - indexInRow + topSideCorrection + rightToLeftCorrection;
          nodes[i]->named.dr = nodes[dr];
          nodes[dr]->named.ul = nodes[i];
        }
      }
    }

    // "edge" nodes represent the border of the hexagon which pixels will bounce off of
    // generate edge nodes only after value nodes are generated
    // this way the value node indices are in zig-zag order, unaffected by "edge" nodes

    int16_t vertexDistance=222, sideDistance=192, yintercept=384, x1=111; // center-to-point 222
    // clockwise for correct normal orientation
    line16 urLine(vertexDistance,   0,              x1,              sideDistance);
    line16 uLine ( x1,              sideDistance,  -x1,              sideDistance);
    line16 ulLine(-x1,              sideDistance,  -vertexDistance,  0);
    line16 dlLine(-vertexDistance,  0,             -x1,             -sideDistance);
    line16 dLine (-x1,             -sideDistance,   x1,             -sideDistance);
    line16 drLine( x1,             -sideDistance,   vertexDistance,  0);
    line16 urCornerLine = uLine .mergeMidpoints(urLine);
    line16 rCornerLine  = urLine.mergeMidpoints(drLine);
    line16 drCornerLine = drLine.mergeMidpoints(dLine);
    line16 dlCornerLine = dLine .mergeMidpoints(dlLine);
    line16 lCornerLine  = dlLine.mergeMidpoints(ulLine);
    line16 ulCornerLine = ulLine.mergeMidpoints(uLine);

    vector<HexNode *> edges;
    edges.reserve(edgeCount());
    for (auto nodep : nodes) {
      auto &node = *nodep;
      if (!node.named.l && !node.named.ul && !node.named.ur) {
        // top left corner (first node we see)
        node.named.l = edges.emplace_back(new HexNode(ulLine));
        node.named.ul = edges.emplace_back(new HexNode(ulCornerLine));
        node.named.ur = edges.emplace_back(new HexNode(uLine));
      } else if (!node.named.ul && !node.named.ur) {
        // top side
        node.named.ul = node.named.l->named.ur;
        // regular top side or corner?
        node.named.ur = edges.emplace_back(new HexNode(node.named.r ? uLine : urCornerLine));
         if (!node.named.r) {
          // top right corner
          node.named.r = edges.emplace_back(new HexNode(urLine));
         }
      } else if (!node.named.ur && !node.named.r) {
        // top right side
        node.named.ur = node.named.ul->named.r;
        node.named.r = edges.emplace_back(new HexNode(node.named.dr ? urLine : rCornerLine));
        if (!node.named.dr) {
          // right corner
          node.named.dr = edges.emplace_back(new HexNode(drLine));
          assert(node.named.dr && node.named.dr->isEdgeNode(), "node.named.dr");
        }
      } else if (!node.named.ul && !node.named.l) {
        // top left side
        node.named.ul = node.named.ur->named.l;
        node.named.l = edges.emplace_back(new HexNode(node.named.dl ? ulLine : lCornerLine));
        if (!node.named.dl) {
          // left corner
          node.named.dl = edges.emplace_back(new HexNode(dlLine));
        }
      } else if (!node.named.dl && !node.named.l) {
        // bottom left side
        node.named.l = node.named.ul->named.dl;
        node.named.dl = edges.emplace_back(new HexNode(node.named.dr ? dlLine : dlCornerLine));
        if (!node.named.dr) {
          // bottom left corner
          node.named.dr = edges.emplace_back(new HexNode(dLine));
        }
      } else if (!node.named.dr && !node.named.r) {
        // bottom right side
        node.named.r = node.named.ur->named.dr;
        node.named.dr = edges.emplace_back(new HexNode(node.named.dl ? drLine : drCornerLine));
        if (!node.named.dl) {
          // bottom right corner
          node.named.dl = node.named.l->named.dr;
        }
      } else if (!node.named.dr && !node.named.dl) {
        // bottom side
        node.named.dl = node.named.l->named.dr;
        node.named.dr = edges.emplace_back(new HexNode(dLine));
      }
    }
    move(edges.begin(), edges.end(), back_inserter(nodes));
    edges.clear();
  }
public:
  inline UMPoint position(T index) {
    assert(spacing != 0, "geometry disabled");
    if (spacing == 0) {
      return UMPoint(0,0);
    }
    return positions[index];
  }
  vector<HexNode *> nodes;
  vector<UMPoint> positions;

  vector<HexNode *> valueNodes() {
    auto values = vector<HexNode *>(nodes.begin(), nodes.begin()+_valueCount);
    assert(values.size() == valueCount(), "value count mismatch");
    return values;
  }
  vector<HexNode *> edgeNodes() {
    auto edges = vector<HexNode *>(nodes.begin()+_valueCount, nodes.end());
    assert(edges.size() == edgeCount(), "edge count mismatch");
    return edges;
  }

  T valueCount() {
    return _valueCount;
  }
  T edgeCount() {
    return _totalCount-_valueCount;
  }
  void init() {
    // FIXME: do in constructor once it's stable
    initConnections(meridian);
  }
  HexGrid(T meridian, float spacing=0) : meridian(meridian), spacing(spacing) {
    // spacing == 0 means disable geometry features
  }
  ~HexGrid() {
    for (int i = nodes.size()-1; i >=0; --i) {
      delete nodes[i];
    }
  }
  HexNode *operator[](uint16_t index) const {
    return nodes[index];
  }
};

// enum for the "inner space" hexagonal pixel bounding box. each particle exists in this space and can exit on any side
enum class HexagonBounding : uint8_t {
  interior     = 0,
  right        = 1 << 0, // 1
  topright     = 1 << 1, // 2
  topleft      = 1 << 2, // 4
  left         = 1 << 3, // 8
  bottomleft   = 1 << 4, // 16
  bottomright  = 1 << 5, // 32
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
  uint8_t elasticityMultiplier; // adds particle-to-particle bounce in case 100% isn't enough ;)
  const HexGrid<PixelIndex> &hexGrid;
public:
  PixelPhysics(const HexGrid<PixelIndex> &hexGrid, PixelIndex particleCount, uint8_t accelScaling, uint8_t elasticity, uint8_t elasticityMultiplier=1) : hexGrid(hexGrid), accelScaling(accelScaling), elasticity(elasticity), elasticityMultiplier(elasticityMultiplier) {
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
    for (Particle *p : particles) {
      delete p;
    }
  }

private:

  const vector16 unitMotionAcrossBound(HexagonBounding bound) {
    // point-down bound
    switch (bound) {
      case HexagonBounding::right:       return vector16( 222,  0)*2;
      case HexagonBounding::topright:    return vector16( 111,  192)*2; // 256*sqrt(3)/2 * (cos(pi/3), sin(pi/3))
      case HexagonBounding::topleft:     return vector16(-111,  192)*2;
      case HexagonBounding::left:        return vector16(-222,  0)*2;
      case HexagonBounding::bottomleft:  return vector16(-111, -192)*2;
      case HexagonBounding::bottomright: return vector16( 111, -192)*2;
      case HexagonBounding::interior:
      default:
        return vector16(0, 0);
    }
  }

  inline bool point_above_line(vector16 p, int16_t dy, int16_t dx, int16_t b) {
    // y = dy/dx + b
    return p.y > (dy*p.x + b*dx) / dx;
  }

  HexagonBounding innerSpaceHexagonBounding(vector16 p) {
    // check if given point is in its point-down hexagon-shaped inner particle space
    HexagonBounding bounds = HexagonBounding::interior;
    bool abovePosDivider = point_above_line(p,  128,222, 0); // divides plane with positive slope through origin
    bool aboveNegDivider = point_above_line(p, -128,222, 0); // divides plane with negative slope through origin
    
    // FIXME: region-side-bounds are commented, need to test/confirm that this is correct

    if (p.x < -222/* && abovePosDivider && !aboveNegDivider*/) bounds |= HexagonBounding::left;
    if (p.x >  222/* && aboveNegDivider && !abovePosDivider*/) bounds |= HexagonBounding::right;
    if ( point_above_line(p, -128,222,  255)/* && p.x>=0 &&  abovePosDivider*/) bounds |= HexagonBounding::topright;
    if (!point_above_line(p,  128,222, -255)/* && p.x>=0 && !aboveNegDivider*/) bounds |= HexagonBounding::bottomright;
    if (!point_above_line(p, -128,222, -255)/* && p.x<=0 && !abovePosDivider*/) bounds |= HexagonBounding::bottomleft;
    if ( point_above_line(p,  128,222,  255)/* && p.x<=0 &&  aboveNegDivider*/) bounds |= HexagonBounding::topleft;
    return bounds;
  }

  HexGrid<PixelIndex>::HexNode *dstForMotion(const Particle &p, HexagonBounding bounding) {
    switch (bounding) {
      case HexagonBounding::right:       return hexGrid[p.index]->named.r;
      case HexagonBounding::topright:    return hexGrid[p.index]->named.ur;
      case HexagonBounding::topleft:     return hexGrid[p.index]->named.ul;
      case HexagonBounding::left:        return hexGrid[p.index]->named.l;
      case HexagonBounding::bottomleft:  return hexGrid[p.index]->named.dl;
      case HexagonBounding::bottomright: return hexGrid[p.index]->named.dr;
      case HexagonBounding::interior:
      default:
        return nullptr;
    }
  }

  void updateParticleAtBound(int label, Particle &p, HexagonBounding checkBound) {
    assert(checkBound != HexagonBounding::interior, "updateParticleAtBound should not get interior");
    HexagonBounding particleContainment = innerSpaceHexagonBounding(p.pos);
    if ((particleContainment & checkBound) != HexagonBounding::interior) {
      plogf("Particle %i pos=(%i,%i) v=(%i,%i) crossed motion checkBound %i", label, p.pos.x, p.pos.y, p.velocity.x, p.velocity.y, checkBound);
      HexGrid<PixelIndex>::HexNode *dst = dstForMotion(p, checkBound);
      PixelIndex srcPixel = p.index;
      if (dst->isDataNode()) {
        // particle moving/colliding
        PixelIndex dstPixel = dst->data();
        plogf("  particle %i at index %i check dst index %i", label, srcPixel, dstPixel);
        if (particleMap[dstPixel]) {
          Particle &p2 = *(particleMap[dstPixel]);
          // collision
          plogf("  particle %i at pixel %i v=(%i,%i) collision with pixel %i v=(%i,%i)", label, srcPixel, p.velocity.x, p.velocity.y, dstPixel, p2.velocity.x, p2.velocity.y);
          // roll back motion because otherwise p1 may have already skipped past p2
          p.pos -= p.velocity;
          p2.pos -= p2.velocity;
          // convert p1 into p2's coordinate space
          const vector16 pos1 = p.pos - unitMotionAcrossBound(checkBound);
          plogf("  pre-collision points in same coordinate space: p1=(%i, %i), p2=(%i, %i)", pos1.x, pos1.y, p2.pos.x, p2.pos.y);
          vector16 dp = p2.pos - pos1;
          vector16 dv = p2.velocity - p.velocity;
          plogf("    dp=(%i,%i), dv=(%i,%i)", dp.x, dp.y, dv.x, dv.y);
          int dpDotDv = dp.dot(dv);
          int dpDotDp = dp.dot(dp);
          plogf("    dpDotDv=%i, dpDotDp=%i", dpDotDv, dpDotDp);
          // assert(dpDotDp != 0, "points should not overlap");
          if (dpDotDp != 0) {
            vector16 dv1 = vector16(elasticityMultiplier*dp.x * dpDotDv / dpDotDp, elasticityMultiplier*dp.y * dpDotDv / dpDotDp);
            vector16 dv2 = vector16(elasticityMultiplier*dp.x * -dpDotDv / dpDotDp, elasticityMultiplier*dp.y * -dpDotDv / dpDotDp);
            plogf("  unscaled dv1=(%i,%i) dv2=(%i,%i)", dv1.x, dv1.y, dv2.x, dv2.y);
            dv1 = dv1.scale8(elasticity);
            dv2 = dv2.scale8(elasticity);
            plogf("    scaled dv1=(%i,%i) dv2=(%i,%i)", dv1.x, dv1.y, dv2.x, dv2.y);

            plogf("  pre-collision  p1=(%i, %i), p2=(%i, %i)", p.velocity.x, p.velocity.y, p2.velocity.x, p2.velocity.y);
            p.velocity += dv1;
            p2.velocity += dv2;
            plogf("  post-collision velocities p1=(%i, %i), p2=(%i, %i)", p.velocity.x, p.velocity.y, p2.velocity.x, p2.velocity.y);
            
            // roll forward motion?
            p.pos += p.velocity;
            p2.pos += p2.velocity;
          }
        } else {
          // move
          plogf("  particle at index %i move to %i", srcPixel, dstPixel);
          particleMap[srcPixel] = NULL;
          particleMap[dstPixel] = &p;
          p.index = dstPixel;
          p.pos -= unitMotionAcrossBound(checkBound);
        }
      } else {
        
        plogf("  Particle %i intersected with wall via checkBound %i", label, checkBound);
        line16 line = dst->edgeLine();
        plogf("    wall line points (%i,%i), (%i,%i)", line.x1, line.y1, line.x2, line.y2);

        // roll back the particle movement since it crossed a line
        plogf("  pre-wall pos (%i, %i), velocity (%i, %i)", p.pos.x, p.pos.y, p.velocity.x, p.velocity.y);
        p.pos -= p.velocity;
        plogf("    rolled back to (%i, %i)", p.pos.x, p.pos.y);

        // v` = v−2*(v⋅n)/(n⋅n)⋅n
        auto normal = line.normal();
        int32_t VDotN = p.velocity.dot(normal);
        int32_t NDotN = normal.dot(normal);

        // get line as Ax + By + C = 0
        int32_t A = line.A();
        int32_t B = line.B();
        int32_t C = line.C();

        plogf("    normal = (%i,%i), VDotN = %i, NDotN = %i", normal.x, normal.y, VDotN, NDotN);
        plogf("      %i*x+%i*y+%i=0", A, B, C);

        // Find the parameter t=p/q where the particle trajectory intersects the line:
        int32_t t_p = abs(-(A * p.pos.x + B * p.pos.y + C));
        int32_t t_q = abs(A * p.velocity.x + B * p.velocity.y);
        plogf("      t = %i/%i", t_p, t_q);
        if (t_p > t_q) {
          // this happens when we do wall collision for a point already outside the wall, since pixel-neighbor hexa shape is rotated compared to from wall shape
          plogf("    t_p > t_q, fixing..");
          t_q = t_p;
        }
        if (t_q != 0) { // Check for parallel movement
            // Compute intersection point
            int16_t x_int = p.pos.x + p.velocity.x * t_p/t_q;
            int16_t y_int = p.pos.y + p.velocity.y * t_p/t_q;
            plogf("    intersection = (%i, %i)", x_int, y_int);

            // Reflect the velocity
            vector16 dv(-2 * normal.x * VDotN/NDotN, -2 * normal.y * VDotN/NDotN);
            plogf("    dv = (%i,%i)", dv.x, dv.y);
            dv = dv.scale8(elasticity);
            plogf("      scaled dv = (%i,%i)", dv.x, dv.y);
            p.velocity += dv;

            // Update particle position after the collision
            p.pos.x = x_int + p.velocity.x * (t_q-t_p)/t_q;
            p.pos.y = y_int + p.velocity.y * (t_q-t_p)/t_q;
        } else {
          plogf("No ricochet, is velocity 0?");
          // roll motion forward again so we don't get stuck
          p.pos += p.velocity;
        }
        plogf("  post-wall pos (%i, %i), velocity (%i, %i)", p.pos.x, p.pos.y, p.velocity.x, p.velocity.y);
      }
    } else {
      // plogf("particle did not cross checkBound %i", checkBound);
    }
    // sanity constraints
    p.pos.x = constrain(p.pos.x, -0xFF, 0xFF);
    p.pos.y = constrain(p.pos.y, -0xFF, 0xFF);
    p.velocity.x = constrain(p.velocity.x, -0xFF, 0xFF);
    p.velocity.y = constrain(p.velocity.y, -0xFF, 0xFF);
  }
public:
  void setPosition(int particleIndex, PixelIndex position) {
    assert(particleMap[position] == NULL, "attempt to move one particle on top of another");
    particleMap[particles[particleIndex]->index] = NULL;
    particleMap[position] = particles[particleIndex];
    particles[particleIndex]->index = position;
  }

  void addParticle(PixelIndex index) {
    assert(particles.size() < SIZE, "added too many particles");
    if (particles.size() < SIZE) {
      while (particleMap[index] != NULL) {
        index = random16()%SIZE;
      } 
      Particle *p = new Particle();
      p->index = index;
      particles.emplace_back(p);
      particleMap[index] = p;
    }
  }

  void removeParticle(unsigned int particleIndex) {
    Particle *p = particles[particleIndex];
    particles.erase(std::next(particles.begin(), particleIndex));
    particleMap[p->index] = NULL;
    delete p;
  }

  void clear() {
    typename vector<PixelPhysics<SIZE>::Particle *>::reverse_iterator it;
    for (it = particles.rbegin(); it < particles.rend(); --it) {
      delete *it;
    }
    particles.clear();
    for (int i = 0; i < SIZE; ++i) {
      particleMap[i] = NULL;
    }
  }

  inline int16_t scaleAccel(int16_t val) {
    return min((int)0xFF, (int)max((int)-0xFF, (int)(accelScaling * val / 10000)));
  }

  void update(std::function<vector16(PixelIndex)> accelForIndex) {
    // x across hexa (negative when button side down)
    // y vertical on hexa, (negative lipo usb down)
    // z through hexa, (negative leds up)
    vector<Particle *> lastParticles = particles;
    for (int p = 0; p < particles.size(); ++p) {
      vector16 accelVector = accelForIndex(particles[p]->index);
      accelVector = vector16(scaleAccel(accelVector.x), scaleAccel(accelVector.y));
      // plogf("PHYSICS UPDATE scaled accel = %i, %i", accelVector.x, accelVector.y);
      assert(abs(accelVector.x) <= 0xFF, "accelVector.x == %i", accelVector.x);
      assert(abs(accelVector.y) <= 0xFF, "accelVector.y == %i", accelVector.y);

      // plogf("update motion, consider particle %i", p);
      particles[p]->velocity += accelVector;
      particles[p]->velocity.x = constrain(particles[p]->velocity.x, -0xFF, 0xFF);
      particles[p]->velocity.y = constrain(particles[p]->velocity.y, -0xFF, 0xFF);

      particles[p]->pos += particles[p]->velocity;
      // plogf("  p%i now has pos (%i, %i), velocity (%i, %i)", p, particles[p]->pos.x, particles[p]->pos.y, particles[p]->velocity.x, particles[p]->velocity.y);
    }
    for (int p = 0; p < particles.size(); ++p) {
      updateParticleAtBound(p, *particles[p], HexagonBounding::right);
      updateParticleAtBound(p, *particles[p], HexagonBounding::topright);
      updateParticleAtBound(p, *particles[p], HexagonBounding::topleft);
      updateParticleAtBound(p, *particles[p], HexagonBounding::left);
      updateParticleAtBound(p, *particles[p], HexagonBounding::bottomleft);
      updateParticleAtBound(p, *particles[p], HexagonBounding::bottomright);
    }
  }

  void update(vector16 accel) {
    update([accel](PixelIndex index) {
      return accel;
    });
  }
};

#endif // HEXAPHYSICS_H
