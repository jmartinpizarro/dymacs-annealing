// -*- coding: utf-8 -*-
// roadmap_t.cc
// -----------------------------------------------------------------------------
//
// Started on <jue 12-10-2023 21:31:03.656116743 (1697139063)>
// Carlos Linares López <carlos.linares@uc3m.es>
// Ian Herman <iankherman@gmail.com>   Ian Herman <iankherman@gmail.com>

//
// Definition of a single state in the roadmap doamin
//

#include "roadmap_t.h"

using namespace std;

#include <cmath>

// Static vars
graph_t roadmap_t::_graph;
bool roadmap_t::_brute_force = false;
string roadmap_t::_variant = "unit";

// return the children of this state as a vector of tuples, each containing:
// first, the cost of the operator, secondly, its heuristic value; thirdly, the
// successor state.
void roadmap_t::children (int h, const roadmap_t& goal,
                          vector<tuple<int, int, roadmap_t>>& successors) {

    // for all edges issued from this vertex
    for (const auto& edge: roadmap_t::_graph.get_edges (_index)) {

        // create the successor state
        roadmap_t successor {edge.get_to ()};

        // add a new successor to the list of successors. Note there are no
        // heuristics in the unit variant
        if (roadmap_t::_variant == "unit") {
            successors.push_back (make_tuple (1,
                                              0,
                                              successor));
        } else {

            // even if we are running in the dimacs variant, ignore the
            // computation of the heuristic function isf a brute-force variant
            // is being used
            successors.push_back (make_tuple (edge.get_weight (),
                                              (roadmap_t::_brute_force) ? 0 : successor.h (goal),
                                              successor));
        }
    }
}

// return the heuristic distance to get from this state to the given goal state.
// The heuristic function implemented is the air distance according to the
// cosine law using a value for the earth radius equal to 6530 kms
int roadmap_t::h (const roadmap_t& goal) const {

    // in case a brute force search algorithm is being used, then ignore the
    // computation of the heuristic function
    if (roadmap_t::_brute_force) {
        return 0;
    }

    // if this is the goal, then return 0 immediately
    if (_index == goal.get_index ()) {
        return 0;
    }

    // otherwise, compute the great-circle distance between this state and the
    // goal state
    auto ivertex = roadmap_t::_graph.get_vertex (_index);
    auto jvertex = roadmap_t::_graph.get_vertex (goal.get_index ());

    // retrieve the latitude and longitude of each point over the earth in
    // radians
    double lat1 = ivertex.get_latitude ();
    double lat2 = jvertex.get_latitude ();
    double long1 = ivertex.get_longitude ();
    double long2 = jvertex.get_longitude ();

    // if (lat1, long1) == (lat2, long2), then return 0 immediately to avoid
    // errors in the computation of the formula
    if ( int (lat1 * 1000000) == int (lat2 * 1000000) &&
         int (long1 * 1000000) == int (long2 * 1000000)) {
        return 0;
    }

    // and return the arc length over the earth sphere
    return int (acos (sin (lat1) * sin (lat2) +
                      cos (lat1) * cos (lat2) * cos (long2 - long1)) * EARTH_RADIUS);
}


// Local Variables:
// mode:cpp
// fill-column:80
// End:
