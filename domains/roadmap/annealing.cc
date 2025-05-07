#include <cmath>

#include "annealing.h"
#include "utils.h"

// Computes the number of h(n) violations. Returns the total number of
// violations in the subgraph, and fills 'violations[i]' con el # de veces
// que el nodo i incumple h(i) ≤ g(i).
int objective_function(const graph_t *g, std::vector<int> &violations) {

    // As every subgraph is theorically independent from other subgraphs 
    // (connections between nodes of different graphs are not taken into 
    // account), ids are not necessary here.

    size_t N = g->get_nbvertices();
    violations.assign(N, 0);
    int total_violations = 0;

    for (size_t goal = 0; goal < N; ++goal) {
        
        auto dist = dijkstra(g, goal);

        vertex_t meta_v = g->get_vertex(goal);

        for (size_t n = 0; n < N; ++n) {
            if (n == goal) continue;
            vertex_t vn = g->get_vertex(n);
            double h       = great_circle_distance(vn, meta_v);
            double g_real  = dist[n];
            if (h > g_real + EPS) {
                ++violations[n];
                ++total_violations;
            }
        }
    }

    return total_violations;
}


// Returns the probability of accepting a mutated state as the next state, if
// the recently mutated state is not good enough for being accepted inmediatly
double acceptance_criteria(int new_cost, int old_cost, double t_current) {
  return exp(-(new_cost - old_cost) / t_current);
}

// Uses simulated annealing techniques for fixing a dymacs graph.Returns 0 if 
// everything went correctly, -1 otherwise. It creates a new directory with the
// new files for graph processing (again following the dymacs version)
int annealing(graph_t* g){
    // TODO
    return 0;
}