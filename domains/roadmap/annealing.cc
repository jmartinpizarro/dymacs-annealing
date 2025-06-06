#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <map>
#include <random>

#include "annealing.h"
#include "graph_t.h"
#include "parameters.h"
#include "utils.h"

// it is necessary to use some weights to give more importance to
// the distanced moved of a node or the number of violations
static constexpr double W_VIOL = 10.0;
static constexpr double W_DIST = 1.0;

using namespace std;

// Computes the number of h(n) violations. Returns the total number of
// violations in the subgraph, and fills 'violations[i]' con el # de veces
// que el nodo i incumple h(i) ≤ g(i).
double objective_function(const graph_t *g, std::map<int, int> *violations) {
  size_t N = g->get_nbvertices();
  violations->clear();

  double total_violation_magnitude = 0.0;
  double dist_error = 0.0;

  for (size_t n = 0; n < N; ++n) {
    vertex_t v = g->get_vertex(n);
    vector<edge_t> neighbors = g->get_edges(n);

    for (const auto &neighbor : neighbors) {
      size_t n_neighbor = neighbor.get_to();
      int w_neighbor = neighbor.get_weight();
      vertex_t v_neighbor = g->get_vertex(n_neighbor);

      double h = great_circle_distance(v, v_neighbor);
      double diff = h - w_neighbor;

      if (diff > EPS) {
        (*violations)[static_cast<int>(n)]++;
        (*violations)[static_cast<int>(n_neighbor)]++;
        total_violation_magnitude += diff;
      }

      dist_error += std::max(0.0, diff);
    }
  }

  return W_VIOL * total_violation_magnitude + W_DIST * dist_error;
}

// Returns the probability of accepting a mutated state as the next state, if
// the recently mutated state is not good enough for being accepted inmediatly
double acceptance_criteria(int new_cost, int old_cost, double t_current) {
  return exp(-(new_cost - old_cost) / t_current);
}

// Uses simulated annealing techniques for fixing a dymacs graph.Returns 0 if
// everything went correctly, -1 otherwise. It creates a new directory with the
// new files for graph processing (again following the dymacs version)
int annealing(graph_t &graph) {
  int N = graph.get_nbvertices();
  if (N == 0)
    return -1;

  // Evaluate initial cost and violations
  std::map<int, int> violations;
  double old_cost = graph.evaluate(&violations);
  double best_cost = old_cost;
  auto best_vertices = graph.get_vertices();

  std::cout << "[annealing] Initial cost: " << old_cost
            << " | Initial violations: " << violations.size() << std::endl;

  // Initial temperature
  double T = std::max(100.0, old_cost);
  const double init_T = T;

  // Random generators
  std::mt19937_64 rng(std::chrono::steady_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double> uni01(0.0, 1.0);
  std::uniform_int_distribution<int> uniNode(0, N - 1);

  // Simulated Annealing main loop
  for (int iter = 0; iter < MAX_ITERS && T > 1e-6; ++iter) {
    // Pick a node to mutate
    std::vector<int> conflicted;
    conflicted.reserve(violations.size());
    for (const auto &[nid, cnt] : violations)
      if (cnt > 0)
        conflicted.push_back(nid);

    int node_id = conflicted.empty() ? uniNode(rng)
                                     : conflicted[rng() % conflicted.size()];

    // Mutate and evaluate
    auto [nid, old_v] = graph.mutate(node_id);
    double new_cost = graph.evaluate(&violations);

    bool accepted = false;
    if (new_cost < old_cost ||
        uni01(rng) < acceptance_criteria(new_cost, old_cost, T)) {
      old_cost = new_cost;
      accepted = true;

      if (new_cost < best_cost) {
        best_cost = new_cost;
        best_vertices = graph.get_vertices();
      }
    } else {
      // Restore previous vertex
      int casted_nid = static_cast<int>(nid);
      graph.modify_vertex(casted_nid, old_v.get_longitude(), old_v.get_latitude());
    }

    // Cooling
    T *= COOLING_RATE;

    // Logging every 10 iterations
    if (iter % 10 == 0) {
      std::cout << "[annealing] Iteration " << iter
                << " | Cost: " << old_cost
                << " | Best: " << best_cost
                << " | Violations: " << violations.size()
                << " | Temp: " << T
                << " | " << (accepted ? "Accepted" : "Rejected")
                << std::endl;
    }

    // Early exit
    if (old_cost == 0) {
      std::cout << "[annealing] Perfect solution found at iteration " << iter << "!" << std::endl;
      break;
    }
  }

  // Restore best solution
  graph.set_vertices(best_vertices);
  std::cout << "[annealing] Final best cost: " << best_cost
            << " | Final violations: " << violations.size() << std::endl;

  return 0;
}
