#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <random>
#include <unordered_map>
#include <iomanip>
#include <unistd.h>

#include "annealing.h"
#include "graph_t.h"
#include "parameters.h"
#include "utils.h"

using namespace std;

// Computes the nomber of violations h(n) > g(n) for each node. Looking to
// minimise the cost. Returns a double as the cost of modifying the new
// node to the graph
double objective_function(const graph_t *g,
                          std::unordered_map<int, int> *violations) {
  size_t N = g->get_nbvertices();
  violations->clear(); // clean the shit out of prev iter

  double total_violation_magnitude = 0.0;
  double dist_error = 0.0;

  for (size_t n = 1; n < N; ++n) { // idk but there is node 0 = 0.0 0.0
    vertex_t v = g->get_vertex(n);
    vector<edge_t> neighbors = g->get_edges(n);

    for (const auto &neighbor : neighbors) {
      size_t n_neighbor = neighbor.get_to();
      int w_neighbor = neighbor.get_weight();
      vertex_t v_neighbor = g->get_vertex(n_neighbor);

      double h = great_circle_distance(v, v_neighbor);
      double diff = h - w_neighbor;

      if (diff > 0.0) { // diff is positive, then h(n) > g(n)
        cout << "[DEBUG] Violation in the heuristic between the nodes " << 
                n << " - " << n_neighbor << endl;
        (*violations)[static_cast<int>(n)]++;
        total_violation_magnitude += 1;
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

// Uses simulated annealing techniques for fixing a dymacs graph. Returns 0 if
// everything went correctly, -1 otherwise. It creates a new directory with the
// new files for graph processing (again following the dymacs version)
int annealing(graph_t &graph) {
  int N = graph.get_nbvertices();
  if (N == 0)
    return -1;

  // we are looking for to minimise the total cost of the graph, composed of the
  // total number of violations ponderated using the objective function. Thus,
  // we have an initial solution that we want to improve
  unordered_map<int, int> violations; // <nodeId, number of violations>
  double cost = graph.evaluate(&violations);
  cout << cost << endl;

  cout << "[LOGGING] Initial Information data:" << endl
          << "\tTotal of violations found: " << violations.size() << endl
          << "\tInitial cost to minimise: " << cost << endl;

  if (violations.size() == 0) {
    cout << "[LOGGING] Annealing ended. No more violations in the graph." 
            << endl;
    return 0;
  }

  // initial temperature
  double T = max(100000.0, cost); 
    
  // for the acceptance criteria
  std::mt19937_64 rng(std::chrono::steady_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double> uni01(0.0, 1.0);

  // simulated annealing main loop
  for (int iter = 0; iter <= MAX_ITERS && T >= 1e-8; iter++) {

    // pick a random node based on probabilities. The more violations, the more
    // probabilities it will have. However, it is allowed to select nodes with
    // fewer violations 
    vector<int> conflicted;
    for (const auto& [node, vcount] : violations) {
        for (int i = 0; i < vcount; ++i)
            conflicted.push_back(node);
    }

    // random number generator setter
    static random_device rd;
    static mt19937 gen(rd());
    uniform_int_distribution<> dis(0, conflicted.size() - 1);

    int selectedNodeId = conflicted[dis(gen)];

    auto [nid, old_v] = graph.mutate(selectedNodeId);
    double new_cost = graph.evaluate(&violations);

    if (violations.size() == 0) {
      cout << "[LOGGING] New cost is: " << new_cost << endl;
      cout << "[LOGGING] Annealing ended. No more violations in the graph." 
            << endl;
      return 0;
    }

    //sleep(1);
    cout << endl << "Node selected: " << nid << " with a cost of " 
            << new_cost << endl;

    bool accepted = false;
    if (new_cost < cost ||
        uni01(rng) < acceptance_criteria(new_cost, cost, T)) {
      // accept mutation
      cost = new_cost;
      accepted = true;

    } else {
      // Restore previous vertex
      int casted_nid = static_cast<int>(nid);
      graph.modify_vertex(casted_nid, old_v.get_longitude(), old_v.get_latitude());
    }

    // cool temperature
    T *= COOLING_RATE;
  }

  cout << "[LOGGING] Total number of violations in final iteration: " 
          << violations.size() << endl;
  return violations.size();
}
