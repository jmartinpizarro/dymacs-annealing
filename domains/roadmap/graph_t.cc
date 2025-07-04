// -*- coding: utf-8 -*-
// graph_t.cc
// -----------------------------------------------------------------------------
//
// Started on <jue 12-10-2023 18:34:16.501618311 (1697128456)>
// Carlos Linares López <carlos.linares@uc3m.es>
// Ian Herman <iankherman@gmail.com>   Ian Herman <iankherman@gmail.com>

//
// Definition of weighted directed graphs using adjacency lists.
//

#include <cmath>
#include <utility>
#include <random>
#include <stdexcept>
#include <regex>
#include <iostream>
#include <fstream>

#include "graph_t.h"
#include "utils.h"
#include "parameters.h"
#include "annealing.h"

using namespace std;

// add a new vertex to the graph. It returns true if the vertex was effectively
// added and false otherwise (e.g., because it exceeds the overall capacity of
// the graph)
bool graph_t::add_vertex (size_t vertex) {

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
void graph_t::add_edge (size_t from, size_t to, int weight) {

    // Make sure the from vertex exists
    if (!add_vertex (from)) {
        throw std::range_error ("[graph_t::add_edge] It was not possible to add the starting vertex");
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

void graph_t::modify_vertex(int& id, double longitude, double latitude) {
    _vertices[id] = vertex_t(longitude, latitude);
}

// load a graph from a file with the format of the 9th DIMACS competition, and
// stores the location of each vertex according to the given coordinates given
// in radians. It returns the number of edges processed
int graph_t::load (const std::string& filename,
                   const std::map<int, std::pair<double, double>>& coordinates) {

    // First of all, ensure the graph is void
    clear ();

    ifstream stream (filename);

    // create regex to process each line separately. Lines in the DIMACS
    // competition format start with a character: 'c' is used for comments; 'p'
    // is used for providing properties; 'a' adds a new edge to the graph. In
    // the following, both 'c' and 'p' are ignored
    regex comment ("^[cp].*");

    // Lines starting with 'a' add a new edge, and other than the prefix, they
    // come with three numbers: the vertex from, to and the edge weight. The
    // following regexp is used for processing those lines and, in particular,
    // for extracting its components
    regex newedge (R"(^a\s+(\d+)\s+(\d+)\s+(\d+)\s*$)");

    // parse the contents of the file
    string line;
    int lineno = 0;
    while (getline (stream, line)) {

        // skip this line in case it should be ignored
        if (regex_match (line, comment)) {
            continue;
        }

        // at this point, lines must match the add command 'a'
        smatch m;
        if (regex_match (line, m, newedge)) {

            // add a new edge from the first match to the second match with the
            // cost weight given third
            size_t from = stoi (m[1].str ());
            size_t to = stoi (m[2].str ());
            int cost = stoi (m[3].str ());

            // add this edge to the graph
            add_edge (from, to, cost);

            // and store the coordinates of both vertices ---does not matter
            // whether we are overriding or not. In doing so, ensure that the
            // coordinates of all the processed vertices are found
            auto ifrom = coordinates.find (from);
            auto ito = coordinates.find (to);
            if ( ifrom == coordinates.end () ) {
                throw std::range_error ("[graph_t::load] Missing coordinates in 'from' vertex");
            }
            _vertices[from] = vertex_t{ifrom->second.first, ifrom->second.second};

            if ( ito == coordinates.end () ) {
                throw std::range_error ("[graph_t::load] Missing coordinates in 'to' vertex");
            }
            _vertices[to] = vertex_t{ito->second.first, ito->second.second};

            // ----------------------------------------------------------------
            double lat1 = ifrom->second.second;
            double lat2 = ito->second.second;
            double long1 = ifrom->second.first;
            double long2 = ito->second.first;

            double h = double (acos (sin (lat1) * sin (lat2) + cos (lat1) * cos (lat2) * cos (long2 - long1)) * EARTH_RADIUS) * 10;

            // check inconsistencies
            if (h > cost) {

                // abort if any is found!
                cout << " cost (" << from << ", " << to << "): " << cost << endl;
                cout << " h (" << from << ", " << to << "): " << h << endl; cout.flush ();
                //throw std::runtime_error ("[graph_t::load] Inconsistency found! Aborting ...");
            }
            // ----------------------------------------------------------------

            // and add the number of edges processed
            lineno++;

        } else {
            std::cout << line << endl;
            // otherwise, a syntax error should be raised immediately
            throw runtime_error (" Syntax error found in the graph file");
        }
    }

    // return the number of edges processed, i.e., the number of lines parsed
    return lineno;
}

// Mutate: randomly displace one vertex, returns <node_id, old_vertex>
std::pair<size_t, vertex_t> graph_t::mutate(int node_id) {
    static std::mt19937_64 rng(std::random_device{}());
    std::uniform_real_distribution<double> uniVar(
        -COORDINATES_MAX_VARIATION,
        COORDINATES_MAX_VARIATION
    );

    vertex_t old_v = _vertices[node_id];

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
double graph_t::evaluate(std::unordered_map<int,int>* violations) {
    violations->clear();
    double cost = objective_function(this, violations);
    return cost;
}

// saves the graph data in a file following DYMACS format
int graph_t::save(string fileName){

    // file name
    if (fileName.length() >= 2) {
        fileName.insert(fileName.length() - 2, "fix.");
    }

    std::ofstream out{ fileName };
    if (!out.is_open()) {
        return -1;
    }

    // n vertex
    size_t N = get_nbvertices();

    out << "c 9th DIMACS Implementation Challenge: Shortest Paths\n";
    out << "c http://www.dis.uniroma1.it/~challenge9\n";
    out << "c TIGER/Line nodes coords for graph USA-road-d.NY\n";
    out << "c Dataset Coordinates File fixed by Simmulated Annealing, jmartinpizarro\n";
    out << "c\n";
    out << "p aux sp co " << N << endl;
    out << "c graph contains " << N << " nodes\n";
    out << "c\n";

    // for each vertex
    for (size_t idx = 1; idx < N; ++idx) {
        vertex_t v = get_vertex(idx);

        // radians -> degrees
        double deg_lon = v.get_longitude() * 180.0 / M_PI;
        double deg_lat = v.get_latitude()  * 180.0 / M_PI;

        // microgrades
        long ilon = static_cast<long>(std::lround(deg_lon * 1e6));
        long ilat = static_cast<long>(std::lround(deg_lat * 1e6));

        out << "v " << idx << " " << ilon << " " << ilat << "\n";
    }

    out.close();
    return 0;
}

void graph_t::set_vertices(const std::vector<vertex_t>& new_vertices) {
    if (new_vertices.size() != _vertices.size()) {
        std::cerr << "[set_vertices] Error: número de vértices no coincide. "
                  << "Esperado: " << _vertices.size()
                  << ", recibido: " << new_vertices.size() << std::endl;
        return;
    }

    for (size_t i = 0; i < new_vertices.size(); ++i) {
        int id = static_cast<int>(i);
        modify_vertex(id, new_vertices[i].get_longitude(), new_vertices[i].get_latitude());
    }
}


// Local Variables:
// mode:cpp
// fill-column:80
// End:
