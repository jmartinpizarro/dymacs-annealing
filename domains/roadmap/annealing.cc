#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <random>
#include <unordered_map>

#include "annealing.h"
#include "graph_t.h"
#include "parameters.h"
#include "utils.h"

using namespace std;

// Computes the nomber of violations h(n) > g(n) for each node. Looking to
// minimise the cost. Returns a double as the cost of modifying the new
// node to the graph
double objective_function(const graph_t *g, size_t idx_mod,
                          const vertex_t &old_v,
                          std::unordered_map<int, int> *violations) {
  size_t N = g->get_nbvertices();
  violations->clear(); // clean the shit out of prev iter

  int total_violations = 0;

  // counting iters
  for (size_t u = 0; u < N; ++u) {
    vertex_t vu = g->get_vertex(u);
    for (const auto &e : g->get_edges(u)) {
      size_t v = e.get_to();
      int w = e.get_weight();
      vertex_t vv = g->get_vertex(v);

      double h = great_circle_distance(vu, vv);
      double diff = h - static_cast<double>(w);

      if (diff > EPS) {
        ++(*violations)[static_cast<int>(u)];
        ++total_violations;
      }
    }
  }

  // maybe mse? idk, want to try with this
  double mutation_error = 0.0;
  if (idx_mod < N) {
    vertex_t mod_v = g->get_vertex(idx_mod);
    mutation_error = great_circle_distance(old_v, mod_v);
  }

  // ponderated weight
  return W_VIOL * total_violations + W_DIST * mutation_error;
}

// Returns the probability of accepting a mutated state as the next state, if
// the recently mutated state is not good enough for being accepted inmediatly
double acceptance_criteria(int new_cost, int old_cost, double t_current) {
  return exp(-(new_cost - old_cost) / t_current);
}

// Uses simulated annealing techniques for fixing a dymacs graph. Returns 0 if
// everything went correctly, -1 otherwise. It creates a new directory with the
// new files for graph processing (again following the dymacs version)
int annealing(graph_t &graph) {
  using std::cout;
  using std::endl;
  using std::size_t;

  // mapa id -> number of violations
  std::unordered_map<int, int> violations;

  size_t N = graph.get_nbvertices();
  if (N == 0)
    return -1;

  // calculate initial graph violations
  violations.clear();
  int total_viol = 0;
  for (size_t u = 0; u < N; ++u) {
    vertex_t vu = graph.get_vertex(u);
    for (const auto &e : graph.get_edges(u)) {
      size_t v = e.get_to();
      int w = e.get_weight();
      vertex_t vv = graph.get_vertex(v);

      double h = great_circle_distance(vu, vv);
      double diff = h - static_cast<double>(w);
      if (diff > EPS) {
        ++violations[static_cast<int>(u)];
        ++total_viol;
      }
    }
  }

  // no mutate graph, no diff applied
  double old_cost = W_VIOL * total_viol;

  cout << "[annealing] Number of violations in initial iteration: "
       << violations.size() << endl;

  // init temp, in case there are low violations and tons of nodes
  double T = std::max(1, static_cast<int>(old_cost));

  std::mt19937_64 rng(
      std::chrono::steady_clock::now().time_since_epoch().count());

  // main SA
  for (int iter = 0; iter < MAX_ITERS && T > 1e-6; ++iter) {

    int max_key = -1; // node_id
    int max_value = std::numeric_limits<int>::min();

    for (const auto &pair : violations) {
      if (pair.second > max_value) {
        max_value = pair.second;
        max_key = pair.first;
      }
    }

    // mutate
    auto change = graph.mutate(max_key);
    int node_id = change.first;
    vertex_t old_v = change.second;

    // evaluate:
    double new_cost = graph.evaluate(change, &violations);

    // decide if the modification is accepted or not
    std::uniform_real_distribution<double> uni01(0.0, 1.0);

    if (new_cost < old_cost ||
        uni01(rng) < acceptance_criteria(new_cost, old_cost, T)) {
      // accept mutation
      old_cost = new_cost;
    } else {
      // recover the prev node
      graph.modify_vertex(node_id, old_v.get_longitude(), old_v.get_latitude());
    }

    // this shit is burning, cool it
    T *= COOLING_RATE;

    // no violations, stop
    if (old_cost == 0)
      break;
  }

  // count final_violations
  int violations_counter = 0;
  for (const auto &pair : violations) {
    if (pair.second != 0) {
      ++violations_counter;
    }
  }

  cout << "[annealing output] There are " << violations_counter
       << " nodes still violating in the new graph!" << endl;

  return 0;
}
