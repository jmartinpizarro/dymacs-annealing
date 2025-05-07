
#include "graph_t.h"
#include <vector>
#include <queue>
#include <cmath>
#include <limits>

// for avoiding precision errors
static constexpr double EPS = 1e-6;

// for Dijkstra with priority queue
struct NodeDist {
    size_t node;
    double dist;
    bool operator>(const NodeDist& o) const { return dist > o.dist; }
};

// great circle distance
static double great_circle_distance(const vertex_t& A, const vertex_t& B) {

    double lat1 = A.get_latitude(), lon1 = A.get_longitude();
    double lat2 = B.get_latitude(), lon2 = B.get_longitude();

    // heuristic must be int as the weight is an int
    int h = int (acos (sin (lat1) * sin (lat2) + cos (lat1) * 
                cos (lat2) * cos (lon2 - lon1)) * EARTH_RADIUS);

    return h;
}

// Dijkstra. Returns a vector of size N (nodes of the subgraph)
static std::vector<double> dijkstra(const graph_t* g, size_t start) {
    
    size_t N = g->get_nbvertices();
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> dist(N, INF);
    dist[start] = 0.0;

    std::priority_queue<
        NodeDist,
        std::vector<NodeDist>,
        std::greater<NodeDist>
    > pq;
    pq.push({ start, 0.0 });

    while (!pq.empty()) {

        auto [u, du] = pq.top(); 
        pq.pop();
        if (du > dist[u]) continue;

        for (const auto& e : g->get_edges(u)) {
            size_t v = e.get_to();
            double w = static_cast<double>(e.get_weight());
            double alt = du + w;
            if (alt + EPS < dist[v]) {
                dist[v] = alt;
                pq.push({ v, alt });
            }
        }
    }
    return dist;
}
