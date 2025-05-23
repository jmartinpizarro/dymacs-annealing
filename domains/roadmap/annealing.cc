#include <chrono>
#include <cmath>
#include <cstddef>
#include <map>
#include <random>
#include <unordered_map>

#include "annealing.h"
#include "graph_t.h"
#include "state_t.h"
#include "utils.h"

// it is necessary to use some weights to give more importance to
// the distanced moved of a node or the number of violations
static constexpr double W_VIOL = 1.0;
static constexpr double W_DIST = 10.0;

// Computes the number of h(n) violations. Returns the total number of
// violations in the subgraph, and fills 'violations[i]' con el # de veces
// que el nodo i incumple h(i) ≤ g(i).
double objective_function(const graph_t *g, std::map<int, int> *violations) {
  size_t N = g->get_nbvertices();
  violations->clear();

  int total_violations = 0;
  double dist_error = 0.0;

  // for each node considered goal
  for (size_t goal = 0; goal < N; ++goal) {
    auto dist = dijkstra(g, goal);
    vertex_t meta_v = g->get_vertex(goal);

    for (size_t n = 0; n < N; ++n) {
      if (n == goal)
        continue;
      vertex_t vn = g->get_vertex(n);
      double h = great_circle_distance(vn, meta_v);
      double g_real = dist[n];
      double diff = h - g_real;

      // there is a violation
      if (diff > EPS) {
        ++(*violations)[static_cast<int>(n)];
        ++total_violations;
      }

      // add the difference to the error
      dist_error += std::max(0.0, diff);
    }
  }

  // objetivo ponderado
  return W_VIOL * total_violations + W_DIST * dist_error;
}

// Returns the probability of accepting a mutated state as the next state, if
// the recently mutated state is not good enough for being accepted inmediatly
double acceptance_criteria(int new_cost, int old_cost, double t_current) {
  return exp(-(new_cost - old_cost) / t_current);
}

// Uses simulated annealing techniques for fixing a dymacs graph.Returns 0 if
// everything went correctly, -1 otherwise. It creates a new directory with the
// new files for graph processing (again following the dymacs version)
int annealing(state_t &state) {

  int N = state.get_nbvertices();
  if (N == 0)
    return -1;

  // obtain violations of the graph
  std::map<int, int> violations;
  double old_cost = state.evaluate(&violations);
  std::cout << "[annealing] Number of violations in current iteration"
            << violations.size() << std::endl;

  // temperature can be either 1 or the old_cost in order to not iterate
  // tons of time due to a very high temperature and low violations
  double T = std::max(1, static_cast<int>(old_cost));

  // Random engine definition, used for generating random number
  std::mt19937_64 rng(
      std::chrono::steady_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double> uni01(0.0, 1.0);
  std::uniform_int_distribution<int> uniNode(0, static_cast<int>(N) - 1);

  // main SA
  for (int iter = 0; iter < MAX_ITERS && T > 1e-6; ++iter) {

    // choose a node to mutate
    std::vector<int> conflicted;
    conflicted.reserve(violations.size());
    for (auto const &[nid, cnt] : violations) {
      if (cnt > 0)
        conflicted.push_back(nid);
    }
    int node_id = conflicted.empty() ? uniNode(rng)
                                     : conflicted[rng() % conflicted.size()];

    auto [nid, old_v] = state.mutate();
    double new_cost = state.evaluate(&violations);

    // do we accept the new node?
    if (new_cost < old_cost ||
        uni01(rng) < acceptance_criteria(new_cost, old_cost, T)) {
      old_cost = new_cost;
    } else {

      // node was fucked up, restore
      state.modify_vertex(node_id, old_v.get_longitude(), old_v.get_latitude());
    }

    // cool this shit, is burning!
    T *= COOLING_RATE;

    std::cout << "Temperature is " << T << std::endl;

    // no violations, HALT
    if (old_cost == 0)
      break;
  }

  // the graph may still have violations!!
  return 0;
}

// Given the original graph, returns an array of states populated
// with the subgraphs
std::vector<state_t> generateStates(const graph_t &g) {
  std::vector<state_t> states;
  state_t state;
  int c = 0;
  const size_t V = g.get_nbvertices();

  // for each node
  for (size_t u = 0; u < V; ++u) {

    // obtain the edges and weights from the original graph
    // and add them to the new subgraph
    std::vector<edge_t> edgesToAppend = g.get_edges(u);
    vertex_t vertexToAppend = g.get_vertex(u);

    // add the vertex into the _vertices array
    state.modify_vertex(c, vertexToAppend.get_longitude(),
                        vertexToAppend.get_latitude());

    // adds the data to the translation table
    state.add_translation(u, c);

    std::cout << "[generateStates] Vertex added " << c
              << " with a total of vertex in the state: "
              << state.get_nbvertices() << std::endl;

    // add edges
    for (const auto &e : g.get_edges(u)) {
      size_t orig_to = e.get_to();
      auto it = state._mapId.find(orig_to);

      // add if local_to is in the translator table (in the subgraph)
      if (it != state._mapId.end()) {
        size_t local_to = it->second;
        state.add_edge(c, local_to, e.get_weight());
        std::cout << c << " -> " << local_to << " --- w: " << e.get_weight()
                  << std::endl;
      }
    }

    if (state.get_nbvertices() >=
        BATCHES) { // max BATCH surpassed, add and reset
      states.push_back(state);

      // check if there are edges to nodes that
      // are not included in the graph. Remove them.

      state = state_t();
      c = 0;
    }

    c++;

    std::cout << state.get_nbvertices() << std::endl;
    std::cout << "-------" << std::endl;
  }

  // if state size < BATCH size, push
  if (state.get_nbvertices() > 0) {
    states.push_back(state);
  }

  return states;
}
