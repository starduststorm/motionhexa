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

struct Point {
    int x,y;
    Point() : x(0), y(0) {};
    Point(int x, int y) : x(x), y(y) {};
    static Point fromMM(float x, float y) {
        return Point(1000*x, 1000*y);
    }
};

// integral positions given in micrometers relative to center pixel at (0,0)
template<int SIZE>
class ItemGeometry {
    Point positions[SIZE];
    Graph &graph;
public:
    ItemGeometry(Graph g) : graph(g) {};
    inline void setPosition(PixelIndex index, Point pt) {
        positions[index] = pt;
    }
    inline Point position(PixelIndex index) {
        return positions[index];
    }
    vector<Edge> nearbyItems(PixelIndex index) {
        return graph.adjacencies(index);
    }
};
ItemGeometry<LED_COUNT> ledgeometry(ledgraph);

HexGrid<PixelIndex> hexGrid(LED_COUNT);

void initLEDGraph() {
    hexGrid.populate();
    const int kSidelen = lround((3 + sqrt(9 - 12 * (1 - LED_COUNT))) / 6); // 10
    assert(kSidelen == 10, "10");
    const int kMeridian = 2*kSidelen-1;
    ledgraph = Graph({}, LED_COUNT);
    int row = 0;
    int rowCounts[kMeridian] = {0};
    for (int r = 0; r<kMeridian; ++r) {
        rowCounts[r] = kSidelen + (r<kMeridian/2 ? r : kMeridian-r-1);
    }
    int rowStarts[kMeridian] = {0};
    for (int r = 0; r<kMeridian; ++r) {
        rowStarts[r] = (r>0 ? rowStarts[r-1] + rowCounts[r-1] : 0);
    }
    for (int i = 0; i < LED_COUNT; ++i) {
        if (row+1 < kMeridian && i >= rowStarts[row+1]) {
            row++;
        }
        bool topSide = rowCounts[row] < rowCounts[row+1];
        int indexInRow = i - rowStarts[row];
        int rightToLeft = row % 2;
        
        // Compute pixel physical position
        const float pixelSpacing = 3.9;
        const float colSpacing = 3.3774990747593105; // sin(2*PI/6)*pixelSpacing
        int centerRow = kSidelen-1;
        float y = colSpacing * (row - centerRow);
        float x = pixelSpacing * (indexInRow - rowCounts[row]/2) + (rightToLeft ? 0 : pixelSpacing/2);
        ledgeometry.setPosition(i, Point::fromMM(x,y));

        // Find Neighbors
        if (rightToLeft) {
            if (i > rowStarts[row]) {
                ledgraph.addEdge(Edge(i, i-1, Edge::geometric, 0x7F));
                hexGrid.nodes[i].named.r = i-1;
                hexGrid.nodes[i-1].named.l = i;
            }
        } else {
            if (i < rowStarts[row] + rowCounts[row] - 1) {
                ledgraph.addEdge(Edge(i, i+1, Edge::geometric, 0));
                hexGrid.nodes[i].named.r = i+1;
                hexGrid.nodes[i+1].named.l = i;
            }
        }
        if (row+1 < kMeridian) {
            int rightToLeftCorrection = (rightToLeft ? -1 : 0);
            if (topSide || (!rightToLeft && i > rowStarts[row]) || (rightToLeft && i < rowStarts[row] + rowCounts[row]-1)) {
                int dl = (rowStarts[row+1] + rowCounts[row+1]) - (i - rowStarts[row]) + (topSide ? -1 : 0) + rightToLeftCorrection;
                ledgraph.addEdge(Edge(i, dl, Edge::geometric, 0xAB/*170.667*/));
                hexGrid.nodes[i].named.dl = dl;
                hexGrid.nodes[dl].named.ur = i;
            }
            if (topSide || (!rightToLeft && i < rowStarts[row]+rowCounts[row]-1) || (rightToLeft && i > rowStarts[row])) {
                int dr = (rowStarts[row+1] + rowCounts[row+1]) - (i - rowStarts[row]) + (topSide ? -1 : 0) + rightToLeftCorrection + (rightToLeft ? 1 : -1);
                ledgraph.addEdge(Edge(i, dr, Edge::geometric, 0xD5/*213.333*/));
                hexGrid.nodes[i].named.dr = dr;
                hexGrid.nodes[dr].named.ul = i;
            }
        }
    }
}

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
