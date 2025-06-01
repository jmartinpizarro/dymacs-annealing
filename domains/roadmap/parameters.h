#ifndef PARAMETERS_H
#define PARAMETERS_H

constexpr int MAX_ITERS = 10000; // if no improvements after, HALT
constexpr double COOLING_RATE = 0.95;
constexpr double COORDINATES_MAX_VARIATION = 0.1; // bound variation of coord
constexpr double PI = 3.141592653589793238462643383279502884197;

// it is necessary to use some weights to give more importance to
// the distanced moved of a node or the number of violations
static constexpr double W_VIOL = 1.0;
static constexpr double W_DIST = 10.0;
#endif // PARAMETERS_H