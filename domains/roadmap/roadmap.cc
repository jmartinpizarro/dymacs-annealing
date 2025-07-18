// -*- coding: utf-8 -*-
// roadmap.cc
// -----------------------------------------------------------------------------
//
// Started on <jue 12-10-2023 14:03:45.527928974 (1697112225)>
// Carlos Linares López <carlos.linares@uc3m.es>
// Ian Herman <iankherman@gmail.com>   Ian Herman <iankherman@gmail.com>

//
// The roadmap domain is the 9th DIMACS Implementation Challenge: Shortest
// Paths http://www.dis.uniroma1.it/~challenge9
//

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <regex>
#include <string>
#include <utility>
#include <vector>
#include <vector>
#include <cmath>

#include <getopt.h>

#include "../../src/ksearch.h"

#include "graph_t.h"
#include "roadmap_t.h"
#include "annealing.h"
#include "parameters.h"

#define EXIT_OK 0
#define EXIT_FAILURE 1

using namespace std;

extern "C" {
    char *xstrdup (char *p);
}

/* Globals */
char *program_name;                       // The name the program was run with,

static struct option const long_options[] =
{
    {"graph", required_argument, 0, 'g'},
    {NULL, 0, NULL, 0}
};

bool get_coordinates_filename (const string& filename, string& coordinates_filename);
bool get_coordinates (const string& coordinates_filename,
                      map<int, pair<double, double>>& coordinates);
static int decode_switches (int argc, char **argv,
                            string& graph_name);
static void usage (int status);

// main entry point
int main (int argc, char** argv) {

    string graph_name;                        // file with the graph definition
    string coordinates_name;   // filename with the coordinates of all vertices
    map<int, pair<double, double>> coordinates;  // coordinates of all vertices
    string solver_name;                            // user selection of solvers
    string filename;                            // file with all cases to solve
    string variant;                                    // variant of the domain
    string k_params;                       // user selection of the values of k
    string csvname;                          // name of the output csv filename
    bool no_doctor;                    // whether the doctor is disabled or not
    bool want_summary;      // whether a summary of results is requested or not
    bool want_verbose;                  // whether verbose output was requested
    chrono::time_point<chrono::system_clock> tstart, tend;          // CPU time

    // variables
    program_name = argv[0];
    vector<string> variant_choices = {"unit", "dimacs"};

    // arg parse
    decode_switches (argc, argv, graph_name);

    // get the coordinates filename from the graph name
    if (!get_coordinates_filename (graph_name, coordinates_name)) {
        cerr << " Warning: no coordinates file found!" << endl;
        cerr << "          it will not be possible to apply any heuristics" << endl << endl;
    }

    if (!get_coordinates (coordinates_name, coordinates)) {
        cerr << " Error: the coordinates file '" << coordinates_name << "' could not be processed" << endl << endl;
        exit (EXIT_FAILURE);
    }


    // --graph
    if (graph_name == "") {
        cerr << "DEBUG: graph_name = '" << graph_name << "'\n";
        cerr << "\n Please, provide a filename with the description of the graph" << endl;
        cerr << " See " << program_name << " --help for more details" << endl << endl;
        exit(EXIT_FAILURE);
    }

    /* !------------------------- INITIALIZATION --------------------------! */

    // initialize the static data members of the definition of a roadmap
    roadmap_t::init (graph_name, coordinates, variant);
    auto nbedges = roadmap_t::get_graph ().get_nbedges ();

    graph_t graph;
    graph.load(graph_name, coordinates);

    /* !-------------------------------------------------------------------! */

    cout << endl;
    cout << "[main] Graph processed" << endl;
    cout << " graph: " << graph_name << " (" << graph.get_nbvertices() 
         << " vertices processed)" << endl;
    cout << " graph: " << graph_name << " (" 
         << nbedges << " edges processed)" << endl;
    cout << endl;

    /* !--------------------------- ANNEALING -----------------------------! */
    // start the clock
    tstart = chrono::system_clock::now ();

    annealing(graph);

    cout << "finished annealing" << endl;
    // we should print the total number of violations removed and left

    // and stop the clock
    tend = chrono::system_clock::now ();
    cout << " 🕒 CPU time: " <<
            1e-9*chrono::duration_cast<chrono::nanoseconds>(tend - tstart).count() 
            << " seconds" << endl;
    cout << endl;
    
    graph.save(coordinates_name);
    /* !-------------------------------------------------------------------! */

    // Well done! Keep up the good job!
    return (EXIT_OK);
}


// return true if the coordinates file of the given graph exists and is
// readable. If true, the name of the coordinates file is returned in the second
// argument; otherwise, its contents are undefined
bool get_coordinates_filename (const string& filename, string& coordinates_filename) {

    // replace the extension of the graph name by "co"
    auto path = filesystem::path (filename);
    auto cofile = path.replace_extension("co");

    // verify the file exists and it is an ordinary file
    if (filesystem::exists (cofile) &&
        filesystem::is_regular_file (cofile)) {

        // check the permissions of the coordinates file
        std::error_code ec;
        auto perms = filesystem::status(cofile, ec).permissions();
        if ((perms & filesystem::perms::owner_read) != filesystem::perms::none &&
            (perms & filesystem::perms::group_read) != filesystem::perms::none &&
            (perms & filesystem::perms::others_read) != filesystem::perms::none) {

            coordinates_filename = cofile.string ();
            return true;
        }
    }

    // at this point, the file either does not exist or can not be read, so
    // return false
    return false;
}

// Return true if all data could be properly processed and false otherwise. It
// retrieves all coordinates of all vertices given in the coordinates_filename
// and stores them in a map indexed by the vertex id that stores the longitude
// (x-value) and latitude (y-value). The coordinates are given in radians.
bool get_coordinates (const string& coordinates_filename,
                      map<int, pair<double, double>>& coordinates) {

    ifstream stream (coordinates_filename);

    // create regex to process each line separately. Lines in the DIMACS
    // competition format start with a character: 'c' is used for comments; 'p'
    // is used for providing properties; 'v' adds a new vertex. In the
    // following, both 'c' and 'p' are ignored
    regex comment ("^[cp].*");

    // Lines starting with 'v' add a new vertex, and other than the prefix, they
    // come with three integers: the vertex id, the longitude (x-value) and the
    // latitude (y-value). Note the longitude and latitude might come with a
    // sign, or not
    regex newedge (R"(^v\s+(\d+)\s+([+-]?\d+)\s+([+-]?\d+)\s*$)");

    // parse the contents of the file
    string line;
    int lineno = 0;
    while (getline (stream, line)) {

        // skip this line in case it should be ignored
        if (regex_match (line, comment)) {

            // increment the line counter and skip it
            lineno++;
            continue;
        }

        // at this point, lines must match the vertex command 'v'
        smatch m;
        if (regex_match (line, m, newedge)) {

            // add a new vertex to the map using the vertex id as the key and
            // storing a pair with the longitude and latitude. Note that
            // longitude and latitude are given as integers with six digits of
            // precision
            size_t id = stoi (m[1].str ());
            double lon = stoi (m[2].str ()) / 1'000'000.0;
            double lat = stoi (m[3].str ()) / 1'000'000.0;

            // and add these coordinates to the vertex with identifier id as
            // radians
            coordinates[id] = make_pair (lon*PI/180.0, lat*PI/180.0);

            // and add the number of edges processed
            lineno++;

        } else {

            // otherwise, a syntax error has been found
            cerr << " Syntax error in '" << coordinates_filename << "'::" << lineno << endl;
            return false;
        }
    }

    return true;
}

// Set all the option flags according to the switches specified. Return the
// index of the first non-option argument
static int
decode_switches (int argc, char **argv,
                 string& graph_name) {

    int c;

    // Default values
    graph_name = "";

    while ((c = getopt_long (argc, argv,
                             "g",
                             long_options, (int *) 0)) != EOF) {
        switch (c) {
        case 'g':  /* --graph */
            graph_name = optarg;
            break;
        default:
            cout << endl << " Unknown argument!" << endl;
            usage (EXIT_FAILURE);
        }
    }
    return optind;
}

// TO MODIFY / REMOVE
static void
usage (int status)
{
    cout << endl << " " << program_name << " implements various K shortest-path search algorithms in the 9th DIMACS Implementation Challenge: Shortest Paths" << endl << endl;
    cout << " Usage: " << program_name << " [OPTIONS]" << endl << endl;
    cout << "\
 Mandatory arguments:\n\
      -g, --graph [STRING]       filename with the graph to load. The file contents should be arranged according to the 9th DIMACS\n\
                                 Implementation Challenge: Shortest Paths. See the documentation for additional help\n\
      -s, --solver [STRING]+     K shortest-path algorithms to use. Choices are:\n\
                                    + Brute-force search algorithms:\n\
                                       > 'mDijkstra': brute-force variant of mA*\n\
                                       > 'K0': brute-force variant of K*\n\
                                       > 'belA0': brute-force variant of belA*\n\
                                    + Heuristic search algorithms:\n\
                                       > 'mA*': mA*\n\
                                       > 'K*': K*\n\
                                       > 'belA*': BELA*\n\
                                 It is possible to provide as many as desired in a blank separated list between double quotes,\n\
                                 e.g., \"mDijkstra belA0\"\n\
      -f, --file [STRING]        filename with the test cases to solve. It consists of of precisely two lines. The i-th test case\n\
                                 uses the i-th vertex from the first line as the starting vertex and the i-th vertex from the second\n\
                                 line as the goal.\n\
      -r, --variant [STRING]     Variant of the problem to consider. Choices are {unit, dimacs}. By default, 'dimacs' is used\n\
      -k, --k [NUMBER]+          Definition of the different values of K to test.\n\
                                 The entire specification consists of a semicolon separated list of single specifications\n\
                                 e.g., '1 5; 10 90 10; 100'\n\
                                 Every single specification consists of a blank separated list with up to three integers indicating\n\
                                 the first value of K, the last one and the increment between successive values of K.\n\
                                 If only one value is given (e.g., '100'), only one value of K is used; if only two are given\n\
                                 (e.g., '1 5'), all values of K between them are used with an increment equal to 1\n\
\n\
 Optional arguments:\n\
      -C, --csv [STRING]         name of the csv output files for storing results. If none is given, no file is generated\n\
      -D, --no-doctor            If given, the automated error checking is disabled. Otherwise, all solutions are automatically\n\
                                 checked for correctness\n\
      -S, --summary              If given, only the results of the last solution path found for every instance are shown. Otherwise,\n\
                                 the results of every single solution path are shown in the output csv file. It has no effect if\n\
                                 --csv is not given\n\
 Misc arguments:\n\
      --verbose                  print more information\n\
      -h, --help                 display this help and exit\n\
      -V, --version              output version information and exit\n\
\n";
    exit (status);
}


// Local Variables:
// mode:cpp
// fill-column:80
// End:
