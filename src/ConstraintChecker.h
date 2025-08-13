#ifndef CONSTRAINTCHECKER_H
#define CONSTRAINTCHECKER_H

#include "DEMMap.h"

class ConstraintChecker {
public:
    ConstraintChecker(const DEMMap &dem,
                      double maxTurnDeg,
                      double maxClimbDeg,
                      double minHAGL);

    bool isSafeAltitude(double x, double y, double z) const;
    double getDynamicMinHAGL(double x, double y) const;
    double computeLocalSlope(double x, double y) const;


private:
    const DEMMap &dem_;
    double maxTurnDeg_;
    double maxClimbDeg_;
    double baseMinHAGL_;

    double getSlope(double x, double y) const;
};

#endif
