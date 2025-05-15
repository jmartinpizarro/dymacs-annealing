// definition of the simmulated annealing algorithm

#ifndef ANNEALING_H
#define ANNEALING_H

#include <map>
#include <vector>

#include "graph_t.h"
#include "state_t.h"

constexpr int BATCHES = 200; // maximum size of any subgraph
constexpr int MAX_ITERS = 10000; // if no improvements after, HALT
constexpr double COOLING_RATE = 0.95;
constexpr double COORDINATES_MAX_VARIATION = 0.05; // bound variation of coord

// Computes the number of h(n) violations. Returns the total number of 
// violations in the subgraph and the violations that each node has.
double objective_function(const graph_t* g, std::map<int, int>* violations);

// Returns the probability of accepting a mutated state as the next state, if 
// the recently mutated state is not good enough for being accepted inmediatly
double acceptance_criteria(int new_cost, int old_cost, double t_current);

// Uses simulated annealing techniques for fixing a dymacs graph.Returns 0 if 
// everything went correctly, -1 otherwise. It creates a new directory with the
// new files for graph processing (again following the dymacs version)
int annealing(state_t& state);

// Given the original graph, returns an array of states populated 
// with the subgraphs
std::vector<state_t> generateStates(const graph_t& g);

#endif // ANNEALING_H