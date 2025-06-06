// definition of the simmulated annealing algorithm

#ifndef ANNEALING_H
#define ANNEALING_H

#include <unordered_map>
#include "graph_t.h"

constexpr int BATCHES = 10000; // maximum size of any subgraph

// Computes the number of h(n) violations. Returns the total number of 
// violations in the subgraph and the violations that each node has.
double objective_function(const graph_t *g,
                          std::unordered_map<int, int> *violations);

// Returns the probability of accepting a mutated state as the next state, if 
// the recently mutated state is not good enough for being accepted inmediatly
double acceptance_criteria(int new_cost, int old_cost, double t_current);

// Uses simulated annealing techniques for fixing a dymacs graph.Returns 0 if 
// everything went correctly, -1 otherwise. It creates a new directory with the
// new files for graph processing (again following the dymacs version)
int annealing(graph_t& graph);

#endif // ANNEALING_H