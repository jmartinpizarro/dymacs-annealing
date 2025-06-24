#ifndef PARAMETERS_H
#define PARAMETERS_H

constexpr int MAX_ITERS = 10000000; // if no improvements after, HALT
constexpr double MIN_COST = 10000000;
constexpr double COORDINATES_MAX_VARIATION = 0.001; // bound variation of coord
constexpr double PI = 3.141592653589793238462643383279502884197;
const int TABU_LIST_MAX_SIZE = 20;

// for avoiding precision errors
static constexpr double EPS = 1e-6;

// TEMPERATURE PARAMETERS
constexpr double LOW_TEMP_THRESHOLD = 1e-6;
constexpr double REHEATING_FACTOR = 10;
constexpr double MIN_TEMP_LIMIT = 1e-8;
constexpr double COOLING_RATE = 0.999;

// it is necessary to use some weights to give more importance to
// the distanced moved of a node or the number of violations
static constexpr double W_VIOL = 1.0;
static constexpr double W_DIST = 1.0;
#endif // PARAMETERS_H