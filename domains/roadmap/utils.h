#include <sstream>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>

#include "graph_t.h"
#include "parameters.h"

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

    // heuristic must be double due to precission
    // units calculated are x10 lower than the 
    // real cost (maybe using decimeters)
    double h = (acos (sin (lat1) * sin (lat2) + cos (lat1) * 
                cos (lat2) * cos (lon2 - lon1)) * EARTH_RADIUS) * 10;

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

// Given a graph, hashes it in order to use it in a set
inline std::string hash_graph_state(const graph_t& graph) {
  std::ostringstream oss;
  int N = graph.get_nbvertices();
  for (int i = 1; i < N; ++i) {
    vertex_t v = graph.get_vertex(i);
    oss << std::fixed << std::setprecision(5)
        << v.get_latitude() << "," << v.get_longitude() << ";";
  }
  return oss.str();
}
