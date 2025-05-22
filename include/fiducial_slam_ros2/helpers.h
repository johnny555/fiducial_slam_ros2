#ifndef HELPERS_H
#define HELPERS_H

#include <cmath>
#include <memory>

// Degrees to radians
constexpr double deg2rad(double deg) { return deg * M_PI / 180.0; }

// Radians to degrees
constexpr double rad2deg(double rad) { return rad * 180.0 / M_PI; }

#endif // HELPERS_H
