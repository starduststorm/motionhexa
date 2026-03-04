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
struct fAxial;
struct Axial : vector16 {
  Axial() : vector16(0,0,0) {}
  Axial(int16_t q, int16_t r) : vector16(q,r,-q-r) {}
  Axial(vector16 v) : Axial(v.x,v.y) {}
  Axial(fAxial fax);
  int16_t q() { return x; };
  int16_t r() { return y; };
  int16_t s() { return z; };
  void setQR(int16_t q, int16_t r) {
    x = q;
    y = r;
    z = -q - r;
  }
};

struct fAxial : vectorT<float> {
  fAxial() : vectorT<float>(0,0,0) {}
  fAxial(Axial ax) : vectorT<float>(ax.q(),ax.r(), ax.s()) {}
  fAxial(float q, float r) : vectorT<float>(q,r,-q-r) {}
  float q() { return x; };
  float r() { return y; };
  float s() { return z; };
  void setQR(float q, float r) {
    x = q;
    y = r;
    z = -q - r;
  }
  Axial cubeRound() {
    int qi = round(q());
    int ri = round(r());
    int si = round(s());

    float q_diff = fabs(qi - q());
    float r_diff = fabs(ri - r());
    float s_diff = fabs(si - s());

    if (q_diff > r_diff && q_diff > s_diff) {
      qi = -ri-si;
    } else if (r_diff > s_diff) {
      ri = -qi-si;
    } else {
      si = -qi-ri;
    }
    return Axial(qi, ri);
  }
};

Axial::Axial(fAxial fax) : Axial((int16_t)fax.q(), (int16_t)fax.r()) {}

class AxialAccess {
  int meridian;
  // axial coordinates are stored in a meridian*meridian 2d array
  std::optional<PixelIndex> *storageToPixelMap; // storage index -> pixel index
  int16_t *pixelToStorageMap; // pixel index -> storage index
  unsigned int index(int q, int r) { // q,r -> storage index
    return (q+meridian/2) + meridian*(r+meridian/2);
  }
public:
  AxialAccess(int meridian) : meridian(meridian) {
    int n = meridian / 2;
    int count = 3*n*n + 3*n + 1;
    storageToPixelMap = (std::optional<PixelIndex> *)malloc(meridian * meridian * sizeof(std::optional<PixelIndex>));
    memset(storageToPixelMap, 0, meridian*meridian*sizeof(std::optional<PixelIndex>));
    pixelToStorageMap = (int16_t*)malloc(count * sizeof(int16_t));
    memset(pixelToStorageMap, 0xFF, count*sizeof(int16_t));

    // hexa wiring is zig-zag starting from left-to-right
    PixelIndex px = 0;
    for (int r = -n; r <= n; ++r) {
      int q_start = max(-n, -n - r);
      int q_end   = min( n,  n - r);
      int row = r + n;
      bool rightToLeft = row % 2;
      if (rightToLeft) {
        for (int q = q_end; q >= q_start; --q) {
          int idx = index(q, r);
          storageToPixelMap[idx] = px;
          pixelToStorageMap[px] = idx;
          px++;
        }
      } else {
        for (int q = q_start; q <= q_end; ++q) {
          int idx = index(q, r);
          storageToPixelMap[idx] = px;
          pixelToStorageMap[px] = idx;
          px++;
        }
      }
    }
  }
  ~AxialAccess() {
    free(storageToPixelMap);
    free(pixelToStorageMap);
  }

  Axial axialFromPixelIndex(PixelIndex pxIndex) {
    int rectIndex = pixelToStorageMap[pxIndex];
    int q = rectIndex % meridian - meridian/2;
    int r = rectIndex / meridian - meridian/2;
    return Axial(q,r);
  }
  
  std::optional<PixelIndex> indexAtAxial(int q, int r) {
    if (abs(r) > meridian/2 || abs(q) > meridian/2) {
      return nullopt;
    }
    int rectIndex = index(q,r);
    return storageToPixelMap[rectIndex];
  }
  std::optional<PixelIndex> indexAtAxial(Axial ax) {
    return indexAtAxial(ax.q(), ax.r());
  }

  vectorT<float> hexToRect(fAxial ax, float size = kMeridian) {
    constexpr float sqrtThreeOverTwo = 0.86602540378f;
    constexpr float sqrtThree = 1.73205080757f;
    float x = 3/2.f * ax.q() * size;
    float y = (sqrtThreeOverTwo * ax.q() + sqrtThree * ax.r()) * size;
    return vectorT<float>(x, y);
  }

  fAxial rectToHex(vectorT<float> point, float size = kMeridian) {
    constexpr float sqrtThreeOverThree = 0.57735026919f;
    float x = point.x / size;
    float y = point.y / size;
    float q = 2/3.f * x;
    float r = -1/3.f * x + sqrtThreeOverThree * y;
    return fAxial(q, r);
  }
};

AxialAccess axial(kMeridian);

void initLEDGraph() {
  assert(hexGrid.valueCount() == LED_COUNT, "led count issue");
  ledgraph = Graph({}, LED_COUNT);
  ledgraph.transposeMap = {
                           {clockwise,counterclockwise}, 
                           {counterclockwise,clockwise}, 
                          };
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

/* ---------------------------------------------- */

// hex adaptation of Wu's
// FIXME: line endpoints are draw half-lit, so e.g. a polygon will have its vertexes be half lit, which is not great.
static float fpart(float x) {
  return x - floor(x);
}
static float rfpart(float x) {
  return 1 - fpart(x);
}
static void point(PixelStorage<LED_COUNT> &ctx, int q, int r, CRGB color, float brightness) {
  auto index = axial.indexAtAxial(q, r);
  if (index.has_value()) {
    color.nscale8_video(brightness * 0xFF);
    ctx.point(index.value(), color, blendBrighten);
  }
}
static void hexline(PixelStorage<LED_COUNT> &ctx, float q0, float r0, float q1, float r1, std::function<CRGB(uint8_t)> colorFunc) {
  CRGB color0 = colorFunc(0);
  CRGB color1 = colorFunc(0xFF);
  
  float s0 = -q0-r0;
  float s1 = -q1-r1;
  // determine axis of iteration (steep)
  float qdiff = fabs(q1 - q0);
  float rdiff = fabs(r1 - r0);
  float sdiff = fabs(s1 - s0);
  bool qline = qdiff >= rdiff && qdiff >= sdiff;
  bool rline = rdiff > qdiff && rdiff >= sdiff;
  bool sline = sdiff > qdiff && sdiff > rdiff;

  bool swapped = false;
  // flip draw direction upwards to halve cases
  if ((qline && q0 > q1) || (rline && r0 > r1) || (sline && s0 > s1)) {
    swapped = true;
    std::swap(q0, q1);
    std::swap(r0, r1);
    std::swap(s0, s1);
    std::swap(color0, color1);
  }

  // we consider the three cubic line directions separately but inline them this way
  if (sline) {
    std::swap(q0,s0);
    std::swap(q1,s1);
  } else if (rline) {
    std::swap(q0,r0);
    std::swap(q1,r1);
  }

  float dq = q1 - q0;
  float dr = r1 - r0;
  float ds = s1 - s0;

  float gradient = (dq == 0.0 ? 1.0 : dr/dq);

  // handle first endpoint
  float qend = roundf(q0);
  float rend = r0 + gradient * (qend - q0);
  float qgap = rfpart(q0 + 0.5);
  float qpxl0 = qend; // this will be used in the main loop
  float rpxl0 = floorf(rend);
  if (sline) {
    point(ctx, -qpxl0-rpxl0, rpxl0,   color0, rfpart(rend) * qgap);
    point(ctx, -qpxl0-(rpxl0+1), rpxl0+1, color0,  fpart(rend) * qgap);
  } else if (rline) {
    point(ctx, rpxl0,   qpxl0, color0, rfpart(rend) * qgap);
    point(ctx, rpxl0+1, qpxl0, color0,  fpart(rend) * qgap);
  } else {
    point(ctx, qpxl0, rpxl0  , color0, rfpart(rend) * qgap);
    point(ctx, qpxl0, rpxl0+1, color0,  fpart(rend) * qgap);
  }

  float rinter = rend + gradient; // first r-intersection for the main loop

  // handle second endpoint
  qend = roundf(q1);
  rend = r1 + gradient * (qend - q1);
  qgap = fpart(q1 + 0.5);
  float qpxl1 = qend; //this will be used in the main loop
  float rpxl1 = floorf(rend);
  if (sline) {
    point(ctx, -qpxl1-rpxl1,     rpxl1,   color1, rfpart(rend) * qgap);
    point(ctx, -qpxl1-(rpxl1+1), rpxl1+1, color1,  fpart(rend) * qgap);
  } else if (rline) {
    point(ctx, rpxl1,   qpxl1, color1, rfpart(rend) * qgap);
    point(ctx, rpxl1+1, qpxl1, color1,  fpart(rend) * qgap);
  } else {
    point(ctx, qpxl1, rpxl1,   color1, rfpart(rend) * qgap);
    point(ctx, qpxl1, rpxl1+1, color1,  fpart(rend) * qgap);
  }

  // main loop
  for (float q = qpxl0 + 1; q <= qpxl1 - 1; ++q) {
    uint8_t progress = 0xFF * (uint8_t)(q-(qpxl0 + 1)) / (uint8_t)(qpxl1 - 1-(qpxl0 + 1));
    if (swapped) progress = 0xFF - progress;
    CRGB color = colorFunc(progress);
    if (sline) {
      point(ctx, -q-floorf(rinter), floorf(rinter),   color, rfpart(rinter));
      point(ctx, -q-(floorf(rinter)+1), floorf(rinter)+1, color,  fpart(rinter));
    } else if (rline) {
      point(ctx, floorf(rinter),   q,   color, rfpart(rinter));
      point(ctx, floorf(rinter)+1, q, color,  fpart(rinter));
    } else {
      point(ctx, q, floorf(rinter),   color, rfpart(rinter));
      point(ctx, q, floorf(rinter)+1, color,  fpart(rinter));
    }
    rinter = rinter + gradient;
  }
}

void hexline(PixelStorage<LED_COUNT> &ctx, fAxial p0, fAxial p1, std::function<CRGB(uint8_t)> colorFunc) {
  hexline(ctx, p0.q(), p0.r(), p1.q(), p1.r(), colorFunc);
}
void hexline(PixelStorage<LED_COUNT> &ctx, fAxial p0, fAxial p1, CRGB color) {
  hexline(ctx, p0, p1, [color] (uint8_t progress) {
    return color;
  });
}

// insetEdgeNodesBy is here due to dedependency cycle. Axial should have been the base and hexgrid should have been based on that.
// inset the given hexgrid by inset, maintaining pixel index correctness
template<typename T>
void HexGrid<T>::insetEdgeNodesBy(unsigned inset, AxialAccess &axial) {
  assert(inset < meridian/2, "inset too large");
  Axial centerAxial = axial.axialFromPixelIndex(kHexaCenterIndex);
  for (int i = 0; i < nodes.size(); ++i) {
    HexNode *oldNode = nodes[i];
    if (oldNode->isDataNode()) {
      Axial ax = axial.axialFromPixelIndex(i);
      if (ax.q() == 0 && ax.r() == 0) { continue; }
      int distance = max(max(abs(centerAxial.q() - ax.q()), abs(centerAxial.r() - ax.r())), abs(centerAxial.s() - ax.s()));
      if (distance == meridian/2-inset+1) {
        // found the ring where the new edges will go
        // we'll traverse outward until we find the appropriate edge node to copy
        HexNode *edgeNode = NULL;
        Axial unitDirection(sgn(ax.q()), sgn(ax.r()));
        // neighbor ordering is 0:ul, 1:ur, 2:r, 3:dr, 4:dl, 5:l
        // this is grody
        const int neighborDirections[][3] = {{0/*ul*/, 5/*l*/, 4/*dl*/}, {0/*ul*/, -1, 3/*dr*/}, {1/*ur*/, 2/*r*/, 3/*dr*/}};
        int nd = neighborDirections[unitDirection.q()+1][unitDirection.r()+1];
        HexNode *nextNode = oldNode;
        while (edgeNode == NULL) {
          nextNode = nextNode->neighbors[nd];
          if (nextNode->isEdgeNode()) {
            edgeNode = nextNode;
          }
        }
        // we should now have the appropriate edge node to put in place
        nodes[i] = new HexNode(*edgeNode);
        for (int n = 0; n < 6; ++n) {
          HexNode *neighborNode = oldNode->neighbors[n];
          if (neighborNode->isDataNode()) {
            neighborNode->neighbors[(n+3)%6] = nodes[i];
          }
        }
        delete oldNode;
      }
    }
  }
  meridian -= inset*2;
}

#endif
