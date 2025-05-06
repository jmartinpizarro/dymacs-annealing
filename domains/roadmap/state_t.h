// Definition of a state

#ifndef STATE_T
#define STATE_T

#include <map>
#include <vector>

#include "graph_t.h"

class state_t {

    // a state is defined as a subgraph (from the original graph). The algorithm
    // detects all invalid heuristics and using simulated annealing, it expands
    // the node until no better changes to the nodes coordinates can be done

    private:

        graph_t* _original_graph; // pointer to original graph
        size_t _nebedges;
        std::vector<std:vector<edge_t>> _edges;
        std::vector<vertex_t> _vertices; 

    public:

        // default constructors create an empty graph and collection of
        // edges and vertices of the subgraph
        state_t () : 
            _original_graph {NULL},
            _nbedges {0},
            _edges {std::vector<std::vector<edge_t>>()},
            _vertices {std::vector<vertex_t>()}
            {}

        // move and copy are forbidden
        graph_t (const graph_t& other) = delete;
        graph_t (graph_t&& other) = delete;

        // getters
        const size_t get_nbedges () const { return _nbedges; }
        const std::vector<edge_t>& get_edges (size_t vertex) const { return _edges[vertex]; }
        const vertex_t get_vertex (size_t vertex) const { return _vertices[vertex]; }
        const size_t get_nbvertices () const { return _vertices.size (); }
        const std::vector<vertex_t>& get_vertices () const { return _vertices; }

        // methods

        // add a new vertex to the graph. It returns true if the vertex was
        // effectively added and false otherwise (e.g., because it exceeds the
        // overall capacity of the graph)
        bool add_vertex (size_t vertex);

        // add a new edge to the graph. In case the operation is not feasible, it
        // immediately raises an exception. Otherwise, it returns nothing
        void add_edge (size_t from, size_t to, int weight);

        // modifies the coordinates of a vector. It returns true if the vertex
        // correctly modified, false otherwise
        bool modify_vertex (size_t new_vertex, size_t idx);

        // mutates the state To Define how.
        void mutate();

        // evaluates an state in order to detect if it must mutate or not. To Define how.
        double evaluate();

}; // class state_t

#endif // STATE_T