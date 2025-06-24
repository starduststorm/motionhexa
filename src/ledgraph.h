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
const uint16_t kHexaCenterIndex = 135;
const uint8_t kMeridian = 19;
const float pixelSpacing = 3.9;

HexGrid<PixelIndex> hexGrid(kMeridian, pixelSpacing);

// clockwise degrees, for integer math
static int angleForDirection(HexagonBounding dir) {
    switch (dir) {
    case HexagonBounding::right:       return 0; break;
    case HexagonBounding::bottomright: return 1*360/6; break;
    case HexagonBounding::bottomleft:  return 2*360/6; break;
    case HexagonBounding::left:        return 3*360/6; break;
    case HexagonBounding::topleft:     return 4*360/6; break;
    case HexagonBounding::topright:    return 5*360/6; break;
    default:
        assert(false, "angleForDirection");
        return 0; break;
    }
}

static HexagonBounding directionForAngle(int angle) {
    angle = mod_wrap(angle, 360);
    switch (angle) {
        case 0:       return HexagonBounding::right; break;
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

void initLEDGraph() {
    hexGrid.init();
    assert(hexGrid.valueCount() == LED_COUNT, "led count issue");
    ledgraph = Graph({}, LED_COUNT);
    for (HexGrid<PixelIndex>::HexNode *node : hexGrid.valueNodes()) {
        if (node->named.r && node->named.r->isDataNode()) {
            ledgraph.addEdge(Edge(node->data(), node->named.r->data(), EdgeType::geometric, 0));
        }
        if (node->named.dl && node->named.dl->isDataNode()) {
            ledgraph.addEdge(Edge(node->data(), node->named.dl->data(), EdgeType::geometric, 0xAB/*170.667*/));
        }
        if (node->named.dr && node->named.dr->isDataNode()) {
            ledgraph.addEdge(Edge(node->data(), node->named.dr->data(), EdgeType::geometric, 0xD5/*213.333*/));
        }
    }
    
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
