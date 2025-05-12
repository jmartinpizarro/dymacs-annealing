#include <utility>
#include <random>

#include "state_t.h"
#include "graph_t.h"
#include "utils.h"
#include "annealing.h"

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

void state_t::modify_vertex(int& node_id, double longitude, double latitude) {

    // first, ensure the vertex can be allocated both in the container of
    // vertices and the adjacency matrix. If not, return false right away
    if (node_id > _vertices.max_size () || node_id > _edges.max_size()) {
        return;
    }

    // otherwise, ensure there is space enough to store the new vertex both in
    // the container of vertices and also the adjacency matrix.
    if (node_id >= _vertices.size ()) {
        _vertices.resize (1+node_id);
    }

    // When resizing the adjacency matrix, make sure that the vector stored at
    // this location has space enough to store all neighbours. The reason why we
    // proceed this way is because instead of populating the vector pushing
    // back, we need random access to positions which are available in memory
    if (node_id >= _edges.size ()) {
        _edges.resize (1+node_id, vector<edge_t>());
    }

    _vertices[node_id] = vertex_t(longitude, latitude);
}

// Mutate: randomly displace one vertex, returns <node_id, old_vertex>
std::pair<size_t,vertex_t> state_t::mutate() {

    // RNG static to preserve seed across calls
    static std::mt19937_64 rng(std::random_device{}());
    std::uniform_int_distribution<size_t> uniNode(0, _vertices.size()-1);
    std::uniform_real_distribution<double> uniVar(
        -COORDINATES_MAX_VARIATION,
        COORDINATES_MAX_VARIATION
    );

    size_t node_id = uniNode(rng);
    vertex_t old_v = _vertices[node_id];

    // Apply small random offset
    double dlon = uniVar(rng);
    double dlat = uniVar(rng);

    _vertices[node_id] = vertex_t(
        old_v.get_longitude() + dlon,
        old_v.get_latitude()  + dlat
    );

    return {node_id, old_v};
}

// evaluates an state in order to detect if mutation has been useful
// or not
double state_t::evaluate(std::map<int,int>* violations) {
    // reconstruct the graph class
    graph_t temp;
    size_t N = _vertices.size();
    for (int i = 0; i < N; ++i) {

        temp.add_vertex(i);
        temp.modify_vertex(i,
            _vertices[i].get_longitude(),
            _vertices[i].get_latitude()
        );
    }

    for (size_t u = 0; u < N; ++u) {

        for (auto const& e : _edges[u]) {
            temp.add_edge(u, e.get_to(), e.get_weight());
        }
    }

    violations->clear();
    int cost = objective_function(&temp, violations);
    return static_cast<double>(cost);
}

