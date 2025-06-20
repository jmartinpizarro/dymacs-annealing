// -*- coding: utf-8 -*-
// graph_t.h
// -----------------------------------------------------------------------------
//
// Started on <jue 12-10-2023 18:03:55.079429321 (1697126635)>
// Carlos Linares López <carlos.linares@uc3m.es>
// Ian Herman <iankherman@gmail.com>   Ian Herman <iankherman@gmail.com>

//
// Definition of weighted directed graphs using adjacency lists.
//

#ifndef _GRAPH_T_H_
#define _GRAPH_T_H_

#include <cstdlib>
#include <map>
#include <unordered_map>
#include <string>
#include <vector>
#include <cstddef>

constexpr int EARTH_RADIUS = 6'350'000;

// Definition of a vertex as a pair of longitude (x-value) and latitude
// (y-value)
struct vertex_t {

    // every vertex contains information about its position in the plane
    // measured in radians
    double _longitude;
    double _latitude;

    // default constructor
    vertex_t () :
        _longitude { 0.0 },
        _latitude  { 0.0 }
        {}

    // explicit constructor
    vertex_t (double longitude, double latitude) :
        _longitude { longitude },
        _latitude  { latitude  }
        {}

    // accessors
    const double get_longitude () const { return _longitude; }
    const double get_latitude () const { return _latitude; }

    // operators
    bool operator== (const vertex_t& other) const {

        // I know equality among doubles is hard ...
        return _longitude == other.get_longitude () &&
            _latitude == other.get_latitude (); }

}; // struct vertex_t

// Definition of an edge as the ending vertex of the edge and the cost weight
struct edge_t {

    // every edge contains information about the destination vertex and the
    // weight cost. Vertices are identified with their index into the private
    // vector used for storing the entire graph. Weights are assumed to be
    // always integer numbers
    size_t _to;
    int _weight;

    // default constructors are forbidden
    edge_t () = delete;

    // explicit constructor
    edge_t (size_t to, int weight) :
        _to     {     to },
        _weight { weight }
        {}

    // accessors
    const size_t get_to () const { return _to; }
    const int get_weight () const { return _weight; }

    // operators
    bool operator==(const edge_t& other) const {
        return _to == other.get_to () &&
            _weight == other.get_weight ();
    }
}; // struct edge_t

// Definition of a graph as a vector of adjacency lists whose elements store
// edges as defined above
class graph_t {

private:

    // INVARIANT: A graph consists of a container of vertices and an adjacency
    // matrix. The definition also stores the total number of edges separately

    // store the number of edges in the graph
    size_t _nbedges;

    // the adjacency list is a vector of vectors of edges
    std::vector<std::vector<edge_t>> _edges;

    // vertices are indexed in a vector by their id
    std::vector<vertex_t> _vertices;

public:

    // default constructors create an empty collection of vertices and edges
    graph_t () :
        _nbedges {0},
        _edges {std::vector<std::vector<edge_t>>()}
        {}

    // move and copy constructors are forbidden
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
    void modify_vertex (int& node_id, double longitude, double latitude);

    // clear entire erases the graph
    void clear () {
        _nbedges = 0;
        _edges.clear ();
    }

    // load a graph from a file with the format of the 9th DIMACS competition,
    // and stores the location of each vertex according to the given coordinates
    // given in radians. It returns the number of edges processed
    int load (const std::string& filename,
              const std::map<int, std::pair<double, double>>& coordinates);

    // randomly displace one vertex, returns <node_id, old_vertex>
    std::pair<size_t,vertex_t> mutate(int node_id);
    // evaluates an state in order to detect if mutation has been useful
    // or not
    double evaluate(std::unordered_map<int,int> *violations);

    // saves the graph data in a file following DYMACS format
    int save(std::string fileName);

    void set_vertices(const std::vector<vertex_t> &new_vertices);

}; // class graph_t

#endif // _GRAPH_T_H_

// Local Variables:
// mode:cpp
// fill-column:80
// End:
