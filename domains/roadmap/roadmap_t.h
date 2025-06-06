// -*- coding: utf-8 -*-
// roadmap_t.h
// -----------------------------------------------------------------------------
//
// Started on <jue 12-10-2023 21:18:45.416751021 (1697138325)>
// Carlos Linares López <carlos.linares@uc3m.es>
// Ian Herman <iankherman@gmail.com>   Ian Herman <iankherman@gmail.com>

//
// Definition of a single state in the roadmap doamin
//

#ifndef _ROADMAP_T_H_
#define _ROADMAP_T_H_

#include <cstdlib>
#include <map>
#include <string>
#include <tuple>
#include <vector>
#include <stdexcept>
#include <iostream>

#include "graph_t.h"

// Class definition
//
// Definition of a single state in the roadmap domain
class roadmap_t {

private:

    // INVARIANT: every state of the roadmap domain consists of a single index
    // to a static graph denoting its current location
    static graph_t _graph;                                  // underlying graph
    size_t _index;                                          // current location

    // This domain can be used under two different variants: unit and dimacs. In
    // the unit domain the cost of every edge is 1, while in the dimacs domain
    // the cost of every edge is read from the definition file
    static std::string _variant;

    // The heuristic used in the roadmap is very time consuming. Thus, in case
    // brute-force search variants are going to be used, the computation of the
    // heuristic function must be skipped for the sake of efficiency
    static bool _brute_force;

public:

    // Default constructors are forbidden by default
    roadmap_t () = delete;

    // Explicit constructor
    roadmap_t (size_t index):
        _index {index}
        {}

    // getters
    static const graph_t& get_graph () { return _graph; }
    const size_t get_index () const { return _index; }
    static const std::string get_variant () { return _variant; }

    // setters
    static void set_variant (std::string variant) {
        if (variant != "unit" && variant != "dimacs") {
            throw std::invalid_argument ("[roadmap_t::set_variant] Invalid variant");
        }
        _variant = variant;
    }
    static void set_brute_force (bool brute_force) { _brute_force = brute_force; }

    // operator overloading

    // this instance is less than another if and only if its index is strictly
    // less than the index of the other instance
    bool operator< (const roadmap_t& other) const {
        return _index < other.get_index ();
    }

    // this instance is equal to another if and only if its indices are the same
    bool operator== (const roadmap_t& other) const {
        return _index == other.get_index ();
    }

    // this instance is not equal to another if and only if its indices are different
    bool operator!= (const roadmap_t& other) const {
        return _index != other.get_index ();
    }

    friend std::ostream& operator<< (std::ostream& stream, const roadmap_t& right) {
        stream << right.get_index ();
        return stream;
    }

    // methods

    // load the roadmap graph given in the specified file qualifying every
    // vertex with the coordinates (in radians) given in the second argument
    // using the given variant. The file should define a graph in the format of
    // the 9th DIMACS competition.
    static void init (const std::string& filename,
                      const std::map<int, std::pair<double, double>>& coordinates,
                      const std::string& variant = "unit") {

        // set the given variant
        roadmap_t::_variant = variant;

        // next, load the roadmap graph
        roadmap_t::_graph.load (filename, coordinates);
    }

   // return the children of this state as a vector of tuples, each containing:
   // first, the cost of the operator, secondly, its heuristic value; thirdly,
   // the successor state.
    void children (int h, const roadmap_t& goal,
        std::vector<std::tuple<int, int, roadmap_t>>& successors);

   // return the heuristic distance to get from this state to the given goal state.
   // The heuristic function implemented is the air distance according to the
   // cosine law using a value for the earth radius equal to 6530 kms
    int h (const roadmap_t& goal) const;

}; // class roadmap_t

namespace std {

    // Definition of a hash function for instances of the roadmap. The
    // definition is included in the std namespace so that it can be used
    // directly by the functions implemented in that namespace
    template<>
    struct hash<::roadmap_t> {

        // hash function
        size_t operator() (const roadmap_t& right) const {

            // indices are unique and thus use it as the hash value
            return right.get_index ();
        }

    }; // struct hash<npancake_t>
}

#endif // _ROADMAP_T_H_

// Local Variables:
// mode:cpp
// fill-column:80
// End:
