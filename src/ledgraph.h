#ifndef LEDGRAPH_H
#define LEDGRAPH_H

#include <vector>
#include <algorithm>
#include <FastLED.h>
#include <set>
#include <util.h>
#include <optional>

#include "drawing.h"
#include "hexaphysics.h"

using namespace std;

typedef uint16_t PixelIndex;

const uint8_t EdgeTypesCount = 8;
typedef uint8_t EdgeTypes;

typedef union {
  struct {
    EdgeTypes first;
    EdgeTypes second;
  } edgeTypes;
  uint16_t pair;
} EdgeTypesPair;

typedef union {
  struct {
    EdgeTypes first;
    EdgeTypes second;
    EdgeTypes third;
    EdgeTypes fourth;
  } edgeTypes;
  uint32_t quad;
} EdgeTypesQuad;



struct Edge {
    typedef enum : uint8_t {
        none             = 0,
        inbound          = 1 << 0,
        outbound         = 1 << 1,
        geometric        = 1 << 2,
        all              = 0xFF,
    } EdgeType;
    
    PixelIndex from, to;
    EdgeTypes types;

    uint8_t angle; // 0 == positive x, then counter-clockwise
    
    Edge(PixelIndex from, PixelIndex to, EdgeType type, uint8_t angle=0) : from(from), to(to), types(type), angle(angle) {};
    Edge(PixelIndex from, PixelIndex to, EdgeTypes types, uint8_t angle=0) : from(from), to(to), types(types), angle(angle)  {};

    Edge transpose() {
        EdgeTypes transposeTypes = none;
        if (types & inbound) transposeTypes |= outbound;
        if (types & outbound) transposeTypes |= inbound;
        if (types & geometric) transposeTypes |= geometric;
        return Edge(to, from, transposeTypes, (transposeTypes&EdgeType::geometric ? angle+0x7F : 0));
    }
};

typedef Edge::EdgeType EdgeType;
typedef uint8_t EdgeTypes;

EdgeTypesPair MakeEdgeTypesPair(EdgeTypes first, EdgeTypes second) {
    EdgeTypesPair pair;
    pair.edgeTypes.first = first;
    pair.edgeTypes.second = second;
    return pair;
}

EdgeTypesQuad MakeEdgeTypesQuad(EdgeTypes first, EdgeTypes second=0, EdgeTypes third=0, EdgeTypes fourth=0) {
    EdgeTypesQuad pair;
    pair.edgeTypes.first = first;
    pair.edgeTypes.second = second;
    pair.edgeTypes.third = third;
    pair.edgeTypes.fourth = fourth;
    return pair;
}

EdgeTypesPair MakeEdgeTypesPair(vector<EdgeTypes> vec) {
    assert(vec.size() <= 2, "only two edge type directions allowed");
    unsigned size = vec.size();
    EdgeTypesPair pair = {0};
    if (size > 0) {
        pair.edgeTypes.first = vec[0];
    }
    if (size > 1) {
        pair.edgeTypes.second = vec[1];
    }
    return pair;
}

EdgeTypesQuad MakeEdgeTypesQuad(vector<EdgeTypes> vec) {
    assert(vec.size() <= 4, "only four edge type directions allowed");
    unsigned size = vec.size();
    EdgeTypesQuad pair = {0};
    if (size > 0) {
        pair.edgeTypes.first = vec[0];
    }
    if (size > 1) {
        pair.edgeTypes.second = vec[1];
    }
    if (size > 2) {
        pair.edgeTypes.third = vec[2];
    }
     if (size > 3) {
        pair.edgeTypes.fourth = vec[3];
    }
    return pair;
}

class Graph {
public:
    vector<vector<Edge> > adjList;
    Graph() { }
    Graph(vector<Edge> const &edges, int count) {
        adjList.resize(count);

        for (auto &edge : edges) {
            addEdge(edge);
        }
    }

    void addEdge(Edge edge, bool bidirectional=true) {
        adjList[edge.from].push_back(edge);
        if (bidirectional) {
            adjList[edge.to].push_back(edge.transpose());
        }
    }

    vector<Edge> adjacencies(PixelIndex vertex) {
        vector<Edge> adjList;
        getAdjacencies(vertex, adjList);
        return adjList;
    }

    vector<Edge> adjacencies(PixelIndex vertex, EdgeTypesPair pair) {
        vector<Edge> adjList;
        getAdjacencies(vertex, pair.edgeTypes.first, adjList);
        getAdjacencies(vertex, pair.edgeTypes.second, adjList);
        return adjList;
    }

    vector<Edge> adjacencies(PixelIndex vertex, EdgeTypesQuad quad) {
        vector<Edge> adjList;
        getAdjacencies(vertex, quad.edgeTypes.first, adjList);
        getAdjacencies(vertex, quad.edgeTypes.second, adjList);
        getAdjacencies(vertex, quad.edgeTypes.third, adjList);
        getAdjacencies(vertex, quad.edgeTypes.fourth, adjList);
        return adjList;
    }

    void getAdjacencies(PixelIndex vertex, std::vector<Edge> &insertInto) {
        vector<Edge> &adj = adjList[vertex];
        for (Edge &edge : adj) {
            insertInto.push_back(edge);
        }
    }

    void getAdjacencies(PixelIndex vertex, EdgeTypes matching, std::vector<Edge> &insertInto) {
        if (matching == 0) {
            return;
        }
        vector<Edge> &adj = adjList[vertex];
        for (Edge &edge : adj) {
            if ((edge.types & matching) == matching) {
                insertInto.push_back(edge);
            }
        }
    }
};

#define LED_COUNT (271)
Graph ledgraph;
const uint16_t kHexaCenterIndex = 135;
const uint8_t kMeridian = 19;
const float pixelSpacing = 3.9;

HexGrid<PixelIndex> hexGrid(kMeridian, pixelSpacing);

void initLEDGraph() {
    hexGrid.init();
    assert(hexGrid.valueCount() == LED_COUNT, "led count issue");
    ledgraph = Graph({}, LED_COUNT);
    for (HexGrid<PixelIndex>::HexNode *node : hexGrid.valueNodes()) {
        if (node->named.r && node->named.r->isDataNode()) {
            ledgraph.addEdge(Edge(node->data(), node->named.r->data(), Edge::geometric, 0));
        }
        if (node->named.dl && node->named.dl->isDataNode()) {
            ledgraph.addEdge(Edge(node->data(), node->named.dl->data(), Edge::geometric, 0xAB/*170.667*/));
        }
        if (node->named.dr && node->named.dr->isDataNode()) {
            ledgraph.addEdge(Edge(node->data(), node->named.dr->data(), Edge::geometric, 0xD5/*213.333*/));
        }
    }
};

// FIXME: can this not have to encode the count this way?
typedef CustomDrawingContext<LED_COUNT, 1, CRGB, CRGBArray<LED_COUNT> > DrawingContext;

void graphTest(DrawingContext &ctx) {
    ctx.leds.fill_solid(CRGB::Black);
    int leader = (millis() / 100) % LED_COUNT;
    
    vector<Edge> edges = ledgraph.adjList[(leader)%LED_COUNT];
    for (Edge edge : edges) {
        ctx.leds[edge.to] = CRGB::Green;
        ctx.leds[edge.from] = CRGB::Blue;
    }
}

#endif
