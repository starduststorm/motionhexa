#ifndef LEDGRAPH_H
#define LEDGRAPH_H

using namespace std;

#include <vector>
#include <algorithm>
#include <FastLED.h>
#include <set>
#include <util.h>
#include <optional>

#include <drawing.h>

#define LED_COUNT (271)

#include <mapping.h>
#include "hexaphysics.h"
#include "MotionManager.h"

typedef enum : uint8_t {
  none              = 0,
  clockwise         = 1 << 0,
  counterclockwise  = 1 << 1,
  geometric         = 1 << 2,
  inbound           = 1 << 3,
  outbound          = 1 << 4,
  all               = 0xFF,
} EdgeType;

Graph ledgraph;
constexpr uint16_t kHexaCenterIndex = LED_COUNT/2;
constexpr uint8_t kMeridian = (3 + sqrt(12*LED_COUNT-3))/6 * 2 - 1;
const float pixelSpacing = 3.9;

HexGrid<PixelIndex> hexGrid(kMeridian, pixelSpacing);

// clockwise degrees, for integer math
static int angleForDirection(HexagonBounding dir) {
  switch (dir) {
  case HexagonBounding::right:     return 0; break;
  case HexagonBounding::bottomright: return 1*360/6; break;
  case HexagonBounding::bottomleft:  return 2*360/6; break;
  case HexagonBounding::left:    return 3*360/6; break;
  case HexagonBounding::topleft:   return 4*360/6; break;
  case HexagonBounding::topright:  return 5*360/6; break;
  default:
    assert(false, "angleForDirection");
    return 0; break;
  }
}

static HexagonBounding directionForAngle(int angle) {
  angle = mod_wrap(angle, 360);
  switch (angle) {
    case 0:     return HexagonBounding::right; break;
    case 1*360/6: return HexagonBounding::bottomright; break;
    case 2*360/6: return HexagonBounding::bottomleft; break;
    case 3*360/6: return HexagonBounding::left; break;
    case 4*360/6: return HexagonBounding::topleft; break;
    case 5*360/6: return HexagonBounding::topright; break;
  default:
    assert(false, "directionForAngle(%i)", angle);
    return HexagonBounding::interior; break;
  }
}

vector16 accelerationAtPixelIndex(PixelIndex index, ICM_20948_AGMT_t &agmt) {
  // imu_pos = 8.0506, 22.9692 # 108.0506, 77.0308 relative to 100,100 center

  UMPoint Q = hexGrid.position(index); // in um

  #if HARDWARE_VERSION > 1
  static const UMPoint P = UMPoint::fromMM(100-83.125922, 100-92.920152);
  vector32 accel(agmt.acc.axes.y, agmt.acc.axes.x);
  #else
  static const UMPoint P = UMPoint::fromMM(8.0506, -22.9692);
  vector16 accel(-agmt.acc.axes.x, agmt.acc.axes.y);
  #endif
  // vector16 gyro(agmt.gyr.axes.x, agmt.gyr.axes.y);
  // gyro = gyro / 1000;
  // gyro = gyro.scale8(0x02);
  // logf("accelerationAtPixelIndex(%03i), Q=(%i,%i), accel=(%i,%i), gryo=(%i, %i)", index, Q.x, Q.y, accel.x, accel.y, gyro.x, gyro.y);

  // FIXME: doesn't work, oversimplified

  UMPoint P2Q = Q - P;
  if (index == 0 || index == 270) {
    // logf("index %i P2Q = (%i, %i)", index, P2Q.x, P2Q.y);
  }
  // FIXME: this never worked right
  auto ω_z = 0;//agmt.gyr.axes.z / 15000;
  auto vec = vector16(accel.x + ω_z*ω_z * P2Q.x, accel.y + ω_z*ω_z * P2Q.y);
  // logf("  => (%i, %i)", vec.x, vec.y);
  return vec;
}

struct Axial : vector16 {
  Axial() : vector16(0,0,0) {}
  Axial(int16_t q, int16_t r) : vector16(q,r,-q-r) {}
  int16_t q() { return x; };
  int16_t r() { return y; };
  int16_t s() { return z; };
  void setQR(int16_t q, int16_t r) {
    x = q;
    y = r;
    z = -q - r;
  }
};

template<int SIZE>
class AxialAccess {
  int meridian;
  std::optional<PixelIndex> *rectToPixelMap; // rect index -> pixel index
  int16_t *pixelToRectMap; // pixel index -> rect index
  CRGBArray<SIZE>& leds;
  unsigned int index(int q, int r) { // q,r -> rect index
    return (q+meridian/2) + meridian*(r+meridian/2);
  }
public:
  AxialAccess(CRGBArray<SIZE>& leds, HexGrid<PixelIndex> &hexGrid) : leds(leds) {
    assert(hexGrid.nodes.size() > 0, "hexGrid uninitialized");
    meridian = (3 + sqrt(12*SIZE-3))/6 * 2 - 1;
    rectToPixelMap = (std::optional<PixelIndex> *)malloc(meridian * meridian * sizeof(std::optional<PixelIndex>)); // rectangular storage
    memset(rectToPixelMap, 0, meridian*meridian*sizeof(std::optional<PixelIndex>));
    pixelToRectMap = (int16_t*)malloc(SIZE * sizeof(int16_t)); // mapping from pixel index back to Axial
    memset(pixelToRectMap, 0xFF, SIZE*sizeof(int16_t));

    HexGrid<PixelIndex>::HexNode *leftNode = hexGrid.nodes[0]; // starts at top-left node
    int q = 0, r = -meridian/2;
    int rowStartQ = q;
    for (int row = 0; row < meridian; ++row) {
      auto rowNode = leftNode;
      while (!rowNode->isEdgeNode()) {
        int idx = index(q,r);
        rectToPixelMap[idx] = rowNode->data();
        pixelToRectMap[rowNode->data()] = idx;
        q++;
        rowNode = rowNode->named.r;
      }
      if (leftNode->named.dl->isDataNode()) { // top half of hexa
        leftNode = leftNode->named.dl;
        rowStartQ--;
      } else { // bottom half
        leftNode = leftNode->named.dr;
      }
      r++;
      q = rowStartQ;
    }
  }
  ~AxialAccess() {
    free(rectToPixelMap);
    free(pixelToRectMap);
  }

  Axial axialFromPixelIndex(PixelIndex pxIndex) {
    int rectIndex = pixelToRectMap[pxIndex];
    int q = rectIndex % meridian - meridian/2;
    int r = rectIndex / meridian - meridian/2;
    return Axial(q,r);
  }
  
  std::optional<PixelIndex> indexAtAxial(int q, int r) {
    if (abs(r) > meridian/2 || abs(q) > meridian/2) {
      return nullopt;
    }
    int rectIndex = index(q,r);
    return rectToPixelMap[rectIndex];
  }

  CRGB* at(int q, int r) {
    int idx = index(q,r);
    if (idx >= 0 && idx < SIZE) {
      return &leds[rectToPixelMap[idx]];
    }
    return nullptr;
  }
  CRGB& operator()(int q, int r) {
    int idx = index(q,r);
    assert(idx >= 0 && idx < SIZE, "axial access (%i, %i) out of range 0 <= idx %i < SIZE %i", q, r, idx, SIZE);
    return leds[rectToPixelMap[idx]];
  }
};

void initLEDGraph() {
  assert(hexGrid.valueCount() == LED_COUNT, "led count issue");
  ledgraph = Graph({}, LED_COUNT);
  
  // get clockwise/counterclockwise edges by traversing hexgrid starting at px 0 and circling perimeter
  PixelIndex index = 0;
  int angle = 0;
  set<HexGrid<PixelIndex>::HexNode*> found;
  while (index != kHexaCenterIndex) {
    auto node = hexGrid.nodes[index];
    found.insert(node);
    auto nextNode = node->dstForMotion(directionForAngle(angle));
    if (nextNode->isDataNode()) {
      bool alreadyVisited = find(found.begin(), found.end(), nextNode) != found.end();
      if (!alreadyVisited || angle == 300) {
        // discovering a new pixel in the ring, or completing a ring
        PixelIndex nextIndex = nextNode->data();
        ledgraph.addEdge(Edge(index, nextIndex, EdgeType::clockwise));

        if (!alreadyVisited) {
          // not done with ring
          index = nextIndex;
        }
      }
      if (alreadyVisited) {
        // complete loop, turn
        angle = (angle+360/6) % 360;
      }
    } else {
      // edge node, turn
      angle = (angle+360/6) % 360;
    }
  }
};

#endif
