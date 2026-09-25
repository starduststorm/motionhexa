#ifndef HEXAPHYSICS_H
#define HEXAPHYSICS_H

#include <vector>
#include <optional>
#include <FastLED.h>

#include <util.h>

using namespace std;

#define DEBUG_PHYSICS 0

#if DEBUG_PHYSICS
bool physicsDebugFlag = false;
#define plogf(format, ...) if (physicsDebugFlag) logf(format, ## __VA_ARGS__)
#else
#define plogf(format, ...)
#endif

constexpr float kSqrtThree = 1.73205080757f;
constexpr float kSqrtThreeOverThree = 0.57735026919f;
constexpr float kSqrtThreeOverTwo = 0.86602540378f;

// TODO: tbh this should have just used cube coordinates instead of the connections web with hex nodes - would have been simpler to construct, use, copy, inset, etc.

static const int kMotionDamper = 8; // drop some low order motion bits

class AxialAccess;

template<typename T>
struct vectorT {
  T x,y,z;
  vectorT() : x(0), y(0), z(0) {}
  vectorT(T x, T y) : x(x), y(y), z(0) {}
  vectorT(T x, T y, T z) : x(x), y(y), z(z) {}

  template<typename T2>
  vectorT(const vectorT<T2> &other) : x(other.x), y(other.y), z(other.z) {}
  
  float dot(const vectorT<float> &other) const {
    return x*other.x + y*other.y + z*other.z;
  }

  // vector16 dot should be int32
  template<typename T2>
  int32_t dot(const vectorT<T2> &other) const {
    return x*other.x + y*other.y + z*other.z;
  }
  
  const vectorT<T> operator-() const {
    return vectorT<T>(-x, -y, -z);
  }

  template<typename T2>
  const vectorT<T> operator+(const vectorT<T2> &other) const {
    return vectorT<T>(x+other.x, y+other.y, z+other.z);
  }

  template<typename T2>
  const vectorT<T> operator-(const vectorT<T2> &other) const {
    return vectorT<T>(x-other.x, y-other.y, z-other.z);
  }
  virtual const vectorT<T> operator*(const T multiplier) const {
    return vectorT<T>(x*multiplier, y*multiplier, z*multiplier);
  }
  virtual const vectorT<T> operator/(const T divisor) const {
    return vectorT<T>(x/divisor, y/divisor, z/divisor);
  }
  template <typename T2>
  vectorT<T> &operator=(const vectorT<T2> &other) {
    x = other.x;
    y = other.y;
    z = other.z;
    return *this;
  }
  template <typename T2>
  vectorT<T> &operator+=(const vectorT<T2> &other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }
  vectorT<T> &operator-=(const vectorT<T> &other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }
  vectorT<T> &operator*=(const T multiplier) {
    x *= multiplier;
    y *= multiplier;
    z *= multiplier;
    return *this;
  }
  vectorT<T> &operator/=(const T divisor) {
    x /= divisor;
    y /= divisor;
    z /= divisor;
    return *this;
  }
  vectorT<T> operator>>(const unsigned int shift) const {
    return vectorT<T>(x >> shift, y >> shift, z >> shift);
  }
  bool operator==(const vectorT<T> & oth) const { return x == oth.x && y == oth.y && z == oth.z; }
  vectorT<T> scale8(uint8_t scaleBy) {
    T sx = (x * scaleBy) / 255;
    T sy = (y * scaleBy) / 255;
    T sz = (z * scaleBy) / 255;
    return vectorT<T>(sx, sy, sz);
  }
  float length() {
    return sqrt((float)(x*x + y*y + z*z));
  }
};

// lhs multiplication
template<typename T>
vectorT<T> operator*(int multiplier, const vectorT<T>& vec) {
    return vec * (T)multiplier;
}

typedef vectorT<int8_t> vector8;
typedef vectorT<int16_t> vector16;
typedef vectorT<int32_t> vector32;
typedef vectorT<float> vectorf;

template<typename T>
struct lineT {
  T A, B, C;
  
  lineT(T x1, T y1, T x2, T y2) {
    A = y2-y1;
    B = x1-x2;
    C = y1 * x2 - x1*y2;
  }
  lineT(T A, T B, T C) : A(A), B(B), C(C) { }
  vectorf normal(bool clockwise=true) {
    return vectorf(A,B) / sqrt(A*A+B*B);
  }
  vectorT<T> longnormal(bool clockwise=true) {
    return vectorT(A,B);
  }
  bool operator==(const lineT<T> & oth) const { return A == oth.A && B == oth.B && C == oth.C; }
 };

typedef lineT<int32_t> line32;
typedef lineT<float> linef;

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

// enum for the "inner space" hexagonal pixel bounding box. each particle exists in this space and can exit on any side
enum class HexagonBounding : uint8_t {
  interior     = 0,
  right        = 1 << 0, // 1
  upright      = 1 << 1, // 2
  upleft       = 1 << 2, // 4
  left         = 1 << 3, // 8
  downleft     = 1 << 4, // 16
  downright    = 1 << 5, // 32
  up           = 1 << 6, // 64
  down         = 1 << 7, // 128
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
    optional<line32> _edgeLine = nullopt;
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
    HexNode(line32 edgeLine) : _edgeLine(edgeLine) {
      initNeighbors();
    }
    HexNode(const HexGrid<T>::HexNode& oth) {
      _value = oth._value;
      _edgeLine = oth._edgeLine;
      for (int j = 0; j < 6; ++j) {
        neighbors[j] = oth.neighbors[j];
      }
    }
    HexNode(HexGrid<T>::HexNode&& oth) noexcept :
        _value(move(oth._value)),
        _edgeLine(move(oth._edgeLine)) {
          for (int j = 0; j < 6; ++j) {
            neighbors[j] = oth.neighbors[j];
            oth.neighbors[j] = nullptr;
          }
    }

    T data() {
      return _value.value();
    }
    line32 edgeLine(){
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
    HexNode *dstForMotion(HexagonBounding bounding) {
      switch (bounding) {
        case HexagonBounding::right:       return named.r;
        case HexagonBounding::upright:    return named.ur;
        case HexagonBounding::upleft:     return named.ul;
        case HexagonBounding::left:        return named.l;
        case HexagonBounding::downleft:  return named.dl;
        case HexagonBounding::downright: return named.dr;
        case HexagonBounding::interior:
        default:
          return nullptr;
      }
    }
    bool operator==(const HexNode & oth) const { return _value == oth._value && _edgeLine == oth._edgeLine; }
  };
private:
  T meridian;
  const bool zigzag = true; // pixel wiring order, see initConnections
  T _valueCount, _totalCount;
  const float spacing=0;
  
  inline void setPosition(T index, UMPoint pt) {
    // logf("setPosition %i = (%i, %i)", index, pt.x, pt.y);
    positions[index] = pt;
  }

  void initConnections(const T meridian) {
  assert(meridian%2 == 1, "Meridian must be an odd number");
  const T kSidelen = (meridian+1) >> 1;
    _valueCount = meridian + (meridian-kSidelen) * (kSidelen + meridian-1); // meridian + 2*(sum of rows from meridian to side)
    const T kMeridianWithEdges = meridian+2;
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
    int rowCounts[meridian] = {0};
    for (int r = 0; r<meridian; ++r) {
      rowCounts[r] = kSidelen + (r<meridian/2 ? r : meridian-r-1);
    }
    int rowStarts[meridian] = {0};
    for (int r = 0; r<meridian; ++r) {
      rowStarts[r] = (r>0 ? rowStarts[r-1] + rowCounts[r-1] : 0);
    }
    // Wiring order within a row: zig-zag alternates direction every row (v1-v6 pcbs), row-major runs every row left-to-right (v7+).
    // Row 0 is the top row and starts at the top-left corner in both cases. 
    auto rowRightToLeft = [&](int r) -> bool { return zigzag && (r % 2); };
    auto indexForPos = [&](int r, int pos) -> int { return rowStarts[r] + (rowRightToLeft(r) ? rowCounts[r]-1-pos : pos); };

    for (int i = 0; i < _valueCount; ++i) {
      if (row+1 < meridian && i >= rowStarts[row+1]) {
        row++;
      }
      bool topSide = rowCounts[row] < rowCounts[row+1];
      int indexInRow = i - rowStarts[row];
      int posInRow = (rowRightToLeft(row) ? rowCounts[row]-1-indexInRow : indexInRow); // geometric position, left to right
      
      // Compute pixel physical position
      // integral positions given in micrometers relative to center pixel at (0,0)
      if (spacing != 0) {
        const float colSpacing = sin(2*PI/6)*spacing; // 3.3774990747593105 when spacing == 3.9
        int centerRow = kSidelen-1;
        float y = -colSpacing * (row - centerRow);
        float x = spacing * (posInRow - rowCounts[row]/2) + (rowCounts[row] % 2 == 0 ? spacing/2 : 0);
        // logf("spacing: i=%i, row=%i, posInRow=%i, rowCounts[row]=%i, x,y=(%f,%f)", i, row, posInRow, rowCounts[row], x, y);
        setPosition(i, UMPoint::fromMM(x,y));
      }

      // Find Neighbors
      if (posInRow + 1 < rowCounts[row]) {
        int r = indexForPos(row, posInRow+1);
        nodes[i]->named.r = nodes[r];
        nodes[r]->named.l = nodes[i];
      }
      if (row+1 < meridian) {
        // top half: next row is one longer, so below-left is at the same position and below-right one further.
        // bottom half: next row is one shorter, so below-left is one position back and below-right at the same position.
        int dlPos = (topSide ? posInRow : posInRow-1);
        int drPos = (topSide ? posInRow+1 : posInRow);
        if (dlPos >= 0) {
          int dl = indexForPos(row+1, dlPos);
          nodes[i]->named.dl = nodes[dl];
          nodes[dl]->named.ur = nodes[i];
        }
        if (drPos < rowCounts[row+1]) {
          int dr = indexForPos(row+1, drPos);
          nodes[i]->named.dr = nodes[dr];
          nodes[dr]->named.ul = nodes[i];
        }
      }
    }

    // "edge" nodes represent the border of the hexagon which pixels will bounce off of
    // generate edge nodes only after value nodes are generated
    // this way the value node indices are in wiring order, unaffected by "edge" nodes

    int16_t vertexDistance=222, sideDistance=192, yintercept=384, x1=111; // center-to-point 222
    // clockwise for correct normal orientation
    line32 urLine(vertexDistance,   0,              x1,              sideDistance);
    line32 uLine ( x1,              sideDistance,  -x1,              sideDistance);
    line32 ulLine(-x1,              sideDistance,  -vertexDistance,  0);
    line32 dlLine(-vertexDistance,  0,             -x1,             -sideDistance);
    line32 dLine (-x1,             -sideDistance,   x1,             -sideDistance);
    line32 drLine( x1,             -sideDistance,   vertexDistance,  0);
    
    // -C for correct normal?
    // FIXME: I don't see any difference when flipping the normals, so something may be off about corner line collision
    line32 lCornerLine (-1,    0,   -294);
    line32 rCornerLine ( 1,    0,   -294);
    line32 urCornerLine( 128,  222, -75426);
    line32 drCornerLine( 128, -222, -75426);
    line32 dlCornerLine(-128, -222, -75426);
    line32 ulCornerLine(-128,  222, -75426);

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
  inline UMPoint position(T index) const {
    assert(spacing != 0, "geometry disabled");
    if (spacing == 0) {
      return UMPoint(0,0);
    }
    return positions[index];
  }
  float pitchMM() const { return spacing; }
  vector<HexNode *> nodes;
  vector<UMPoint> positions;

  T valueCount() const {
    return _valueCount;
  }
  T edgeCount() const {
    return _totalCount-_valueCount;
  }
  HexGrid(T meridian, float spacing=0, bool zigzag=true) : meridian(meridian), zigzag(zigzag), spacing(spacing) {
    // spacing == 0 means disable geometry features
    initConnections(meridian);
  }
  ~HexGrid() {
    for (int i = nodes.size()-1; i >=0; --i) {
      delete nodes[i];
    }
  }
  HexNode *operator[](uint16_t index) const {
    return nodes[index];
  }
  void insetEdgeNodesBy(unsigned inset, AxialAccess &axial); // implemented in ledgraph.h yayyy
};

struct PixelPhysicsTuning {
  float gravityScale = 0.08f;       // tilt: the gravity part of the accelerometer reading
  float inertiaScale = 0.5f;        // the hexa's own motion: linear acceleration, centrifugal, Euler and Coriolis pseudo-forces
  float viscousPerSecond = 0.0f;    // drag: fraction of velocity lost per second (linearized per step)
  uint8_t elasticity = 0xF4;        // bounce off walls and other particles: restitution = 2*elasticity/255 - 1, so 0x80 is perfectly inelastic
  uint8_t elasticityMultiplier = 1; // adds particle-to-particle bounce in case 100% isn't enough ;)
  // Impacts slower than this (relative normal speed, mm/s) are perfectly inelastic, so a pile comes to rest instead of jostling
  // forever on the velocity gravity feeds it every frame. Top speed is ~280 mm/s.
  float restSpeedMMps = 35.0f;
  // Coulomb friction in contacts: the tangential impulse is at most this times the normal impulse. Sets the angle of repose,
  // atan(friction): without it a pile has none, its wedged particles keep shoving their supports apart and it never settles.
  float friction = 0.15f;
};

// Integer particle simulation on the pixel grid, one particle per pixel at most.
//
// Each particle lives in the pointy-top hexagonal cell of its pixel, in cell units (kUnitsPerPixel per pixel pitch), and is a
// disc one pitch across: particles in adjacent pixels touch when both sit at their cell centers, and are pushed apart (with an
// impulse, restitution and Coulomb friction) whenever they overlap, every step, so a pile transmits force and comes to rest.
// Crossing a cell bound moves a particle into an empty neighboring pixel, and the board's edge lines reflect it. The frame is
// sub-stepped so a particle never moves more than a fraction of a cell between contact checks.
template<unsigned int SIZE>
class PixelPhysics {
public:
  static constexpr int32_t kUnitsPerPixel = 444;   // cell units per pixel pitch, see unitMotionAcrossBound

  static constexpr int32_t kVelocityFrac = 256;    // velocity fixed point: 256 = one cell unit per kMotionDamper ms (the pre-Q8 unit)
  static constexpr int32_t kMaxVelocity = 0xFF * kVelocityFrac; // 32 cell units/ms; per-step travel must stay well inside a cell
  static constexpr int32_t kPosDivisor = kVelocityFrac * 1000 * kMotionDamper; // velocity·µs per cell unit
  static constexpr int32_t kPosLimit = 320;        // sanity bound on cell position, beyond the corner walls at 294
  static constexpr uint32_t kMaxStepUs = 8000;     // sub-step length: 256 cell units at kMaxVelocity, still short of a pitch
  static constexpr uint32_t kMaxFrameUs = 32000;   // a longer stall is a resync, not 32+ms of motion
  static constexpr int32_t kMaxRadialCoeff = 20000; // Q14 velocity per µm; keeps coeff × 55mm inside int32
  static constexpr int kWallNormalShift = 8;       // wall lines are scaled to |(A,B)| = 2^8 so projections onto them are shifts
  static constexpr int32_t kWallNormal = 1 << kWallNormalShift;

  struct Particle {
    PixelIndex index;
    vector16 pos;          // pos within the pixel's hexagonal cell, cell units, nominally within (-255, 255)
    vector32 velocity;     // Q8: kVelocityFrac = one cell unit per kMotionDamper ms, clamped to ±kMaxVelocity
    vector32 posRemainder; // sub-unit remainder of position integration, so slow particles still creep on short steps
    Particle() : index(0), pos(0,0), velocity(0,0), posRemainder(0,0) {};
    Particle(PixelIndex index, vector16 pos, vector32 velocity) : index(index), pos(pos), velocity(velocity), posRemainder(0,0) {};
  };

  // What the hexa felt this frame, in the motion frame (which reads directly as the direction things fall across the pixel
  // geometry, see MotionFrame). Gravity and the body's own acceleration come separately so tilt and shove scale independently.
  struct Motion {
    vectorf gravityG;  // gravity as seen by the body, g
    vectorf linearG;   // the body's own acceleration, g
    float gyroZ = 0;   // rad/s about the LED-face normal, right-handed
  };

  vector<Particle *> particles;
  Particle *particleMap[SIZE] = {0}; // map from physical led index to particle
  PixelPhysicsTuning tuning;
  const HexGrid<PixelIndex> &hexGrid;
  const UMPoint sensorPosition; // accel/gyro position in the pixel geometry, µm: the point whose acceleration the sensor reports
private:
  float velPerMsPerG;  // Q8 velocity gained per ms under 1g at physical scale
  float velPerMsPerUm; // Q8 velocity per (µm × rad/ms): centrifugal and Euler terms per µm of lever arm
  int32_t restSpeedQ8; // tuning.restSpeedMMps in Q8 velocity
  int32_t frictionQ8;  // tuning.friction in Q8

  // The board's edge, per pixel: indices into wallLines of the distinct edge lines among the pixel's neighbors, kNoWall-terminated.
  // A side pixel has one, a corner pixel three, an interior pixel none. The edge runs through the cell at 30deg to the cell's own
  // bounds (the walls are the sides of the board hexagon, the bounds the sides of the pixel's cell), so the wall is tested
  // every step rather than when a bound is crossed.
  static constexpr uint8_t kNoWall = 0xFF;
  static constexpr int kMaxWallsPerPixel = 3;
  vector<line32> wallLines;
  uint8_t pixelWalls[SIZE][kMaxWallsPerPixel];

  void initWalls() {
    for (unsigned i = 0; i < SIZE; ++i) {
      int count = 0;
      for (int w = 0; w < kMaxWallsPerPixel; ++w) {
        pixelWalls[i][w] = kNoWall;
      }
      if (i >= (unsigned)hexGrid.valueCount()) {
        continue;
      }
      for (int n = 0; n < 6; ++n) {
        typename HexGrid<PixelIndex>::HexNode *neighbor = hexGrid[i]->neighbors[n];
        if (!neighbor || !neighbor->isEdgeNode()) {
          continue;
        }
        line32 line = neighbor->edgeLine();
        {
          const float k = kWallNormal / sqrtf((float)line.A * line.A + (float)line.B * line.B);
          line = line32((int32_t)lroundf(line.A * k), (int32_t)lroundf(line.B * k), (int32_t)lroundf(line.C * k));
        }
        uint8_t index = kNoWall;
        for (unsigned l = 0; l < wallLines.size(); ++l) {
          if (wallLines[l] == line) {
            index = l;
            break;
          }
        }
        if (index == kNoWall) {
          index = wallLines.size();
          wallLines.push_back(line);
        }
        bool known = false;
        for (int w = 0; w < count; ++w) {
          known |= (pixelWalls[i][w] == index);
        }
        if (!known) {
          assert(count < kMaxWallsPerPixel, "pixel %u has more than %i walls", i, kMaxWallsPerPixel);
          if (count < kMaxWallsPerPixel) {
            pixelWalls[i][count++] = index;
          }
        }
      }
    }
  }
public:
  PixelPhysics(const HexGrid<PixelIndex> &hexGrid, UMPoint sensorPosition, PixelIndex particleCount, PixelPhysicsTuning tuning)
      : tuning(tuning), hexGrid(hexGrid), sensorPosition(sensorPosition) {
    const float pitchUm = hexGrid.pitchMM() * 1000.0f;
    const float unitsPerUm = kUnitsPerPixel / pitchUm;
    velPerMsPerUm = unitsPerUm * kMotionDamper * kVelocityFrac;
    velPerMsPerG = 9.80665f * velPerMsPerUm; // 1g = 9.80665 µm/ms²
    // one Q8 velocity unit is (1/kVelocityFrac) cell units per kMotionDamper ms
    const float mmPerSecPerQ8 = pitchUm / kUnitsPerPixel / kVelocityFrac / kMotionDamper; // µm/ms == mm/s
    restSpeedQ8 = tuning.restSpeedMMps / mmPerSecPerQ8;
    frictionQ8 = constrain(tuning.friction, 0.0f, 4.0f) * 256;
    initWalls();
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
    // pointy-top cell: neighbors along the row are a pitch apart, the diagonal rows a pitch at 60deg
    switch (bound) {
      case HexagonBounding::right:       return vector16( 222,  0)*2;
      case HexagonBounding::upright:    return vector16( 111,  192)*2; // 256*sqrt(3)/2 * (cos(pi/3), sin(pi/3))
      case HexagonBounding::upleft:     return vector16(-111,  192)*2;
      case HexagonBounding::left:        return vector16(-222,  0)*2;
      case HexagonBounding::downleft:  return vector16(-111, -192)*2;
      case HexagonBounding::downright: return vector16( 111, -192)*2;
      case HexagonBounding::interior:
      default:
        return vector16(0, 0);
    }
  }

  static inline bool point_above_line(vector16 p, int32_t dy, int32_t dx, int32_t b) {
    // y > dy/dx * x + b, dx > 0; cross-multiplied so there is no division in the hot path
    return (int32_t)p.y * dx > dy * p.x + b * dx;
  }

  HexagonBounding innerSpaceHexagonBounding(vector16 p) {
    // check if given point is in its pointy-top hexagon-shaped cell: flat sides at x=±222, sloped sides through (0,±255)
    HexagonBounding bounds = HexagonBounding::interior;
    if (p.x < -222) bounds |= HexagonBounding::left;
    if (p.x >  222) bounds |= HexagonBounding::right;
    if ( point_above_line(p, -128,222,  255)) bounds |= HexagonBounding::upright;
    if (!point_above_line(p,  128,222, -255)) bounds |= HexagonBounding::downright;
    if (!point_above_line(p, -128,222, -255)) bounds |= HexagonBounding::downleft;
    if ( point_above_line(p,  128,222,  255)) bounds |= HexagonBounding::upleft;
    return bounds;
  }

  // |v| within ~6%, no sqrt
  static inline int32_t approxLength(const vector32 &v) {
    const int32_t ax = abs(v.x), ay = abs(v.y);
    return max(ax, ay) + min(ax, ay) / 2;
  }

  static inline void clampVelocity(vector32 &v) {
    v.x = constrain(v.x, -kMaxVelocity, kMaxVelocity);
    v.y = constrain(v.y, -kMaxVelocity, kMaxVelocity);
  }

  static inline void clampPosition(vector16 &pos) {
    pos.x = constrain(pos.x, -kPosLimit, kPosLimit);
    pos.y = constrain(pos.y, -kPosLimit, kPosLimit);
  }

  Particle *neighborParticle(const Particle &p, HexagonBounding dir) {
    typename HexGrid<PixelIndex>::HexNode *dst = hexGrid[p.index]->dstForMotion(dir);
    return (dst && dst->isDataNode()) ? particleMap[dst->data()] : nullptr;
  }

  // The three directions the contact pass visits, with the arithmetic of unitMotionAcrossBound(dir) folded into Q8 constants so
  // the hot path is shifts: proj = (dp.x*cx + dp.y*cy) >> 8 is dp's component along u in cell units, push = overlap*(px,py) >> 8
  // is u*overlap/(2*kUnitsPerPixel).
  struct ContactDir {
    HexagonBounding dir;
    vector16 u;
    int32_t cx, cy;
    int32_t px, py;
  };
  static const ContactDir *contactDirs() {
    static const ContactDir dirs[3] = {
      {HexagonBounding::right,     vector16( 444,    0),  256,    0,  128,    0},
      {HexagonBounding::downright, vector16( 222, -384),  128, -222,   64, -111},
      {HexagonBounding::downleft,  vector16(-222, -384), -128, -222,  -64, -111},
    };
    return dirs;
  }

  // Contact between p and p2 in the adjacent pixel across cd.u (p2's center as seen from p's, |u| = kUnitsPerPixel). Discs a
  // pitch across overlap when their centers are less than a pitch apart along u. An approaching pair exchanges the normal
  // components of velocity (equal masses) with restitution and Coulomb friction, then the overlap is split between the two.
  // The contact normal is u itself rather than the center-to-center line: within 30deg of it, and it keeps a resting pile
  // division-free on the RP2040, where a division costs more than the rest of the contact.
  void contact(Particle &p, Particle &p2, const ContactDir &cd) {
    const vector32 dp = p2.pos - (p.pos - cd.u); // p2 from p, in one coordinate space
    const int32_t proj = (dp.x * cd.cx + dp.y * cd.cy) >> 8;
    const int32_t overlap = kUnitsPerPixel - proj;
    if (overlap <= 0) {
      return;
    }
    const vector32 dv = p2.velocity - p.velocity;
    const int32_t approach = -((dv.x * cd.cx + dv.y * cd.cy) >> 8); // closing speed along û, Q8
    plogf("  contact dp=(%i,%i), dv=(%i,%i) approach=%i overlap=%i", dp.x, dp.y, dv.x, dv.y, approach, overlap);
    // only an approaching pair exchanges momentum
    if (approach > 0) {
      const vector32 dvn((-approach * cd.cx) >> 8, (-approach * cd.cy) >> 8); // the normal part of dv
      // a slow impact sticks: 0x80 leaves both with the average normal velocity. (e is applied as e/256, so 0xFF is 99.6%)
      const int32_t eBase = (approach < restSpeedQ8 && tuning.elasticity > 0x80 ? 0x80 : tuning.elasticity);
      const int32_t e = eBase * tuning.elasticityMultiplier;
      const vector32 dv1((dvn.x * e) >> 8, (dvn.y * e) >> 8);
      p.velocity += dv1;
      p2.velocity -= dv1;
      if (frictionQ8 > 0) {
        // Coulomb friction: transfer relative tangential velocity, up to friction × the normal impulse and at most the half
        // that stops the sliding between equal masses
        const vector32 dvt = dv - dvn;
        const int32_t slide = approxLength(dvt);
        const int32_t grip = (frictionQ8 * ((approach * e) >> 8)) >> 8; // friction × normal impulse, Q8
        if (slide <= 2 * grip) {
          // sticks: the common resting case, no division
          p.velocity.x += dvt.x / 2;
          p.velocity.y += dvt.y / 2;
          p2.velocity.x -= dvt.x / 2;
          p2.velocity.y -= dvt.y / 2;
        } else {
          const int32_t fQ8 = (grip << 8) / slide;
          p.velocity.x += (dvt.x * fQ8) >> 8;
          p.velocity.y += (dvt.y * fQ8) >> 8;
          p2.velocity.x -= (dvt.x * fQ8) >> 8;
          p2.velocity.y -= (dvt.y * fQ8) >> 8;
        }
      }
      clampVelocity(p.velocity);
      clampVelocity(p2.velocity);
      plogf("  post-contact velocities p1=(%i, %i), p2=(%i, %i)", p.velocity.x, p.velocity.y, p2.velocity.x, p2.velocity.y);
    }
    // separate along u, each giving half the overlap
    const vector16 push((overlap * cd.px) >> 8, (overlap * cd.py) >> 8);
    p.pos -= push;
    p2.pos += push;
    clampPosition(p.pos);
    clampPosition(p2.pos);
  }

  // Reflect p off a board edge line in its cell's coordinates. A*x + B*y + C > 0 is beyond the wall (the edge lines wind
  // counterclockwise in initConnections) and initWalls() scaled the line so |(A,B)| = kWallNormal, which makes the projections
  // shifts. Mirrors the position back inside and reflects the outward part of the velocity, with Coulomb friction.
  bool reflectOffWall(Particle &p, const line32 &line) {
    const int32_t A = line.A, B = line.B, C = line.C;
    const int32_t d = A * p.pos.x + B * p.pos.y + C; // kWallNormal × the distance beyond the wall
    if (d <= 0) {
      return false;
    }
    // pos -= 2 n d/(n·n)
    p.pos.x -= (A * d) >> (2 * kWallNormalShift - 1);
    p.pos.y -= (B * d) >> (2 * kWallNormalShift - 1);
    const int32_t vDotN = p.velocity.x * A + p.velocity.y * B;
    if (vDotN > 0) {
      const int32_t vn = vDotN >> kWallNormalShift; // Q8 speed into the wall
      // below 0x80 the reflection would leave the velocity pointing outward and the particle pinned against the mirror; walls are
      // at least perfectly inelastic, and exactly that for a slow impact so a resting particle stays put
      const int32_t e = (vn < restSpeedQ8 || tuning.elasticity < 0x80 ? 0x80 : tuning.elasticity);
      // dvn = 2 (e/256) n̂ (v·n̂)
      const int32_t vne = (vn * e) >> 8;
      const vector32 dvn((A * vne) >> (kWallNormalShift - 1), (B * vne) >> (kWallNormalShift - 1));
      if (frictionQ8 > 0) {
        // Coulomb friction against the wall: up to friction × the normal impulse off the tangential velocity, at most all of it
        const vector32 vt(p.velocity.x - ((A * vn) >> kWallNormalShift), p.velocity.y - ((B * vn) >> kWallNormalShift));
        const int32_t slide = approxLength(vt);
        const int32_t grip = (frictionQ8 * vne) >> 7; // friction × |dvn|
        if (slide <= grip) {
          // sticks: the common resting case, no division
          p.velocity.x -= vt.x;
          p.velocity.y -= vt.y;
        } else {
          const int32_t fQ8 = (grip << 8) / slide;
          p.velocity.x -= (vt.x * fQ8) >> 8;
          p.velocity.y -= (vt.y * fQ8) >> 8;
        }
      }
      p.velocity -= dvn;
    }
    plogf("  post-wall pos (%i, %i), velocity (%i, %i)", p.pos.x, p.pos.y, p.velocity.x, p.velocity.y);
    return true;
  }

  void crossBound(int label, Particle &p, HexagonBounding bound) {
    plogf("Particle %i pos=(%i,%i) v=(%i,%i) crossed bound %i", label, p.pos.x, p.pos.y, p.velocity.x, p.velocity.y, bound);
    HexGrid<PixelIndex>::HexNode *dst = hexGrid[p.index]->dstForMotion(bound);
    if (!dst || !dst->isDataNode()) {
      return; // an edge neighbor: the wall pass in resolveBounds handles it
    }
    PixelIndex srcPixel = p.index;
    PixelIndex dstPixel = dst->data();
    if (particleMap[dstPixel]) {
      return; // occupied: the contact pass keeps the pair apart; the particle stays in its pixel
    }
    plogf("  particle at index %i move to %i", srcPixel, dstPixel);
    particleMap[srcPixel] = NULL;
    particleMap[dstPixel] = &p;
    p.index = dstPixel;
    p.pos -= unitMotionAcrossBound(bound);
    clampPosition(p.pos);
  }

  void resolveBounds(int label, Particle &p) {
    static const HexagonBounding kBounds[6] = {
      HexagonBounding::right, HexagonBounding::upright, HexagonBounding::upleft,
      HexagonBounding::left, HexagonBounding::downleft, HexagonBounding::downright,
    };
    // each bound is considered once per step, against the position as the previous bounds left it
    HexagonBounding containment = innerSpaceHexagonBounding(p.pos);
    for (HexagonBounding bound : kBounds) {
      if (containment == HexagonBounding::interior) {
        break;
      }
      if ((containment & bound) != HexagonBounding::interior) {
        crossBound(label, p, bound);
        containment = innerSpaceHexagonBounding(p.pos);
      }
    }
    // then the board edge, in whichever cell the particle ended up. Two passes so a corner (two or three walls) settles.
    const uint8_t *walls = pixelWalls[p.index];
    if (walls[0] != kNoWall) {
      for (int pass = 0; pass < 2; ++pass) {
        bool hit = false;
        for (int w = 0; w < kMaxWallsPerPixel && walls[w] != kNoWall; ++w) {
          hit |= reflectOffWall(p, wallLines[walls[w]]);
        }
        if (!hit) {
          break;
        }
      }
      clampPosition(p.pos);
      clampVelocity(p.velocity);
    }
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

  unsigned long lastUpdateMicros = 0;
  float lastGyroZ = 0;   // rad/s, for the Euler term
  vectorf dvCarry;       // sub-Q8 remainder of the common acceleration, so a faint tilt still integrates

  void update(const Motion &motion) {
    const unsigned long now = micros();
    uint32_t frameUs = (lastUpdateMicros == 0 ? 1000 : (uint32_t)(now - lastUpdateMicros));
    const bool resync = (lastUpdateMicros == 0 || frameUs > kMaxFrameUs);
    lastUpdateMicros = now;
    frameUs = min(frameUs, kMaxFrameUs);
    const int steps = max(1, (int)((frameUs + kMaxStepUs - 1) / kMaxStepUs));
    const int32_t stepUs = frameUs / steps;
    const float stepMs = stepUs * 1e-3f;

    // Per-frame force constants, float once, then integers per particle.

    // Common acceleration, the same for every particle: the direction things fall, plus the pseudo-force of a shove
    vector32 dvCommon;
    {
      float ax = velPerMsPerG * (tuning.gravityScale * motion.gravityG.x + tuning.inertiaScale * motion.linearG.x) * stepMs * steps + dvCarry.x;
      float ay = velPerMsPerG * (tuning.gravityScale * motion.gravityG.y + tuning.inertiaScale * motion.linearG.y) * stepMs * steps + dvCarry.y;
      dvCommon = vector32((int32_t)(ax / steps), (int32_t)(ay / steps));
      dvCarry = vectorf(ax - dvCommon.x * steps, ay - dvCommon.y * steps);
    }

    // The sensor reports the hexa's acceleration at its own position; a particle is somewhere else on a rotating body, with r
    // its lever arm from the sensor. Centrifugal w²r and Euler -(dw/dt)×r are linear in r, so they reduce to a Q14 coefficient
    // per µm of r. Euler integrates over the frame to -dw×r, so the change in rate is applied directly.
    const float gyroZMs = motion.gyroZ * 1e-3f; // rad/ms
    const float gyroZDeltaMs = (resync ? 0.0f : (motion.gyroZ - lastGyroZ) * 1e-3f) / steps;
    lastGyroZ = motion.gyroZ;
    const int32_t centrifugalQ14 = constrain((int32_t)(tuning.inertiaScale * gyroZMs * gyroZMs * stepMs * velPerMsPerUm * 16384), -kMaxRadialCoeff, kMaxRadialCoeff);
    const int32_t eulerQ14 = constrain((int32_t)(tuning.inertiaScale * gyroZDeltaMs * velPerMsPerUm * 16384), -kMaxRadialCoeff, kMaxRadialCoeff);
    const bool radial = (centrifugalQ14 != 0 || eulerQ14 != 0);

    // Coriolis, -2w×v: turns the velocity through -2w·dt without changing its length. Half a turn before the other forces and
    // half after; all at one end is a first-order error that shows up as drift when a shove meets a spin. Q13 so cos·vx + sin·vy
    // fits int32 at kMaxVelocity.
    const float theta = tuning.inertiaScale * gyroZMs * stepMs;
    int32_t sinQ13, cosQ13;
    if (fabsf(theta) < 0.05f) {
      // small angle, spares the soft-float sinf/cosf on RP2040 in the common case
      sinQ13 = (int32_t)(theta * 8192);
      cosQ13 = 8192 - (int32_t)(theta * theta * 4096);
    } else {
      sinQ13 = (int32_t)(sinf(theta) * 8192);
      cosQ13 = (int32_t)(cosf(theta) * 8192);
    }
    const bool spinning = (sinQ13 != 0);
    auto coriolisHalfTurn = [&](vector32 &v) {
      const int32_t vx = v.x, vy = v.y;
      v.x = ( cosQ13 * vx + sinQ13 * vy) / 8192;
      v.y = (-sinQ13 * vx + cosQ13 * vy) / 8192;
    };

    // Viscous drag as a linear approximation of exp(-k·dt)
    const int32_t dragQ14 = (int32_t)(16384 * max(0.0f, 1.0f - tuning.viscousPerSecond * stepMs * 1e-3f));
    const bool dragging = (dragQ14 < 16384);

    for (int step = 0; step < steps; ++step) {
      for (int i = 0; i < particles.size(); ++i) {
        Particle &p = *particles[i];
        vector32 &v = p.velocity;
        if (spinning) {
          coriolisHalfTurn(v);
        }
        v += dvCommon;
        if (radial) {
          const vector32 r = hexGrid.position(p.index) - sensorPosition; // µm
          v.x += centrifugalQ14 * r.x / 16384 + eulerQ14 * r.y / 16384;
          v.y += centrifugalQ14 * r.y / 16384 - eulerQ14 * r.x / 16384;
        }
        if (spinning) {
          coriolisHalfTurn(v);
        }
        if (dragging) {
          // division rather than shift so both signs decay to zero
          v.x = v.x * dragQ14 / 16384;
          v.y = v.y * dragQ14 / 16384;
        }
        clampVelocity(v);

        // integrate velocity·µs into cell units, carrying the remainder
        vector32 motionNum = v;
        motionNum *= stepUs;
        motionNum += p.posRemainder;
        p.pos += vector16(motionNum.x / kPosDivisor, motionNum.y / kPosDivisor);
        p.posRemainder = vector32(motionNum.x % kPosDivisor, motionNum.y % kPosDivisor);
        clampPosition(p.pos); // a particle pressing into an occupied pixel has nowhere to go
        plogf("  p%i at px %i pos (%i, %i) velocity (%i, %i)", i, p.index, p.pos.x, p.pos.y, v.x, v.y);
      }
      // contacts: three of the six directions visit each adjacent pair exactly once
      const ContactDir *dirs = contactDirs();
      for (int i = 0; i < particles.size(); ++i) {
        Particle &p = *particles[i];
        for (int k = 0; k < 3; ++k) {
          Particle *p2 = neighborParticle(p, dirs[k].dir);
          if (p2) {
            contact(p, *p2, dirs[k]);
          }
        }
      }
      // then moves into empty pixels and the board's edge
      for (int i = 0; i < particles.size(); ++i) {
        resolveBounds(i, *particles[i]);
      }
    }
  }
};

#endif // HEXAPHYSICS_H
