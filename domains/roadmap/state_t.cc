#include "state_t.h"
#include "graph_t.h"

using namespace std;

// add a new vertex to the graph. It returns true if the vertex was effectively
// added and false otherwise (e.g., because it exceeds the overall capacity of
// the graph)
bool state_t::add_vertex (size_t vertex) {

    // first, ensure the vertex can be allocated both in the container of
    // vertices and the adjacency matrix. If not, return false right away
    if (vertex > _vertices.max_size () ||
        vertex > _edges.max_size()) {
        return false;
    }

    // otherwise, ensure there is space enough to store the new vertex both in
    // the container of vertices and also the adjacency matrix.
    if (vertex >= _vertices.size ()) {
        _vertices.resize (1+vertex);
    }

    // When resizing the adjacency matrix, make sure that the vector stored at
    // this location has space enough to store all neighbours. The reason why we
    // proceed this way is because instead of populating the vector pushing
    // back, we need random access to positions which are available in memory
    if (vertex >= _edges.size ()) {
        _edges.resize (1+vertex, vector<edge_t>());
    }

    // and return true
    return true;
}

// add a new edge to the graph. In case the operation is not feasible, it
// immediately raises an exception. Otherwise, it returns nothing
void state_t::add_edge (size_t from, size_t to, int weight) {

    // Make sure the from vertex exists
    if (!add_vertex (from)) {
        throw std::range_error ("[state_t::add_edge] It was not possible to add the starting vertex");
    }

    // even if it is not going to be used by this edge, ensure the to vertex
    // exists as well
    if (!add_vertex (to)) {
        throw std::range_error ("[graph_t::add_edge] It was not possible to add the ending vertex");
    }

    // check whether this edge already exists or not
    vector<edge_t> edges = _edges[from];
    if (find (edges.begin (), edges.end (), edge_t{to, weight}) != edges.end ()) {
        return;
    }

    // next, just simply add a new edge to it
    _edges[from].push_back (edge_t {to, weight});

    // and increment the number of edges
    _nbedges++;
}

bool state_t::modify_vertex(vertex_t new_vertex, size_t idx){

    // ensure that the vertex exists
    if (idx >= _vertices.size()){
        return false;
    }

    // update the new coordinate
    _vertices[idx] = new_vertex;
    return true;
}