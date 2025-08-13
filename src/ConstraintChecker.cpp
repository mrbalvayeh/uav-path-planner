#include "ConstraintChecker.h"
#include <cmath>

ConstraintChecker::ConstraintChecker(const DEMMap &dem,
                                     double maxTurnDeg,
                                     double maxClimbDeg,
                                     double minHAGL)
        : dem_(dem),
          maxTurnDeg_(maxTurnDeg),
          maxClimbDeg_(maxClimbDeg),
          baseMinHAGL_(minHAGL) {}

bool ConstraintChecker::isSafeAltitude(double x, double y, double z) const {
    double terrainZ = dem_.getElevation(x, y);
    double minHAGL = getDynamicMinHAGL(x, y);
    return (z - terrainZ) >= minHAGL;
}
double ConstraintChecker::computeLocalSlope(double x, double y) const {
    double dzdx = (dem_.getElevation(x + 20, y) - dem_.getElevation(x - 20, y)) / 40.0;
    double dzdy = (dem_.getElevation(x, y + 20) - dem_.getElevation(x, y - 20)) / 40.0;
    return std::sqrt(dzdx * dzdx + dzdy * dzdy);
}
double ConstraintChecker::getSlope(double x, double y) const {
    // central difference for estimating slope (in meters per meter)
    double d = 30.0;  // offset in meters
    double z = dem_.getElevation(x, y);
    double dzdx = (dem_.getElevation(x + d, y) - dem_.getElevation(x - d, y)) / (2 * d);
    double dzdy = (dem_.getElevation(x, y + d) - dem_.getElevation(x, y - d)) / (2 * d);
    return std::sqrt(dzdx * dzdx + dzdy * dzdy);  // slope magnitude
}

double ConstraintChecker::getDynamicMinHAGL(double x, double y) const {
    double slope = getSlope(x, y);
    double multiplier = 1.0 + 10.0 * slope;  // you can tune this
    return baseMinHAGL_ * multiplier;
}
