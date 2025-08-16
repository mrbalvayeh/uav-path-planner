#include "ConstraintChecker.h"
#include "DEMMap.h"
#include "WaypointPath.h"

#include <ompl/config.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/PathSimplifier.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

/* -------------------------------------------------------------------------- */
/* Globals                                                                     */
/* -------------------------------------------------------------------------- */
std::shared_ptr<DEMMap> demMap;
std::shared_ptr<ConstraintChecker> checker;

// ---- altitude policy ----
constexpr double ALT_CEILING_M = 6000.0;   // hard ceiling (MSL). TUNE.
constexpr double HAGL_BASE_M   = 30.0;     // your base min HAGL
constexpr double SLOPE_GAIN    = 3000.0;   // your existing slope→HAGL gain

struct AvoidZone {
    double lat;    // deg
    double lon;    // deg
    double radius; // meters
};

// Edit as needed
std::vector<AvoidZone> avoidZones = {
        {28.9016, 54.9832, 40000.0},
        {27.1146, 56.817, 40000.0},
        {27.8804, 58.5829, 30000.0},
        {26.9444, 54.3379, 40000.0}
};

/* -------------------------------------------------------------------------- */
/* Per-leg flag: allow crossing avoid zones?                                   */
/* -------------------------------------------------------------------------- */
static thread_local bool g_ignoreAvoidZonesForThisLeg = false;

/* -------------------------------------------------------------------------- */
/* Validity checker (Dubins state: x, y, yaw)                                  */
/* -------------------------------------------------------------------------- */
bool isStateValid(const ob::State* state) {
    const auto* s = state->as<ob::DubinsStateSpace::StateType>();
    const double x = s->getX();
    const double y = s->getY();

    // Terrain at this (x,y)
    const double elev  = demMap->getElevation(x, y);
    if (elev <= -9000) return false; // outside DEM or nodata

    // Dynamic HAGL
    const double slope = checker->computeLocalSlope(x, y);
    const double minHaglHere = std::max(HAGL_BASE_M, HAGL_BASE_M + slope * SLOPE_GAIN);

    // Required altitude MSL to remain legal here
    const double requiredAltMSL = elev + minHaglHere;

    // HARD CEILING
    if (requiredAltMSL > ALT_CEILING_M) return false;

    // Circular avoid zones (skip if this leg allows going through)
    if (!g_ignoreAvoidZonesForThisLeg) {
        for (const auto& zone : avoidZones) {
            double zx, zy;
            demMap->latlonToUTM(zone.lat, zone.lon, zx, zy);
            const double dx = x - zx, dy = y - zy;
            if (std::sqrt(dx * dx + dy * dy) < zone.radius)
                return false;
        }
    }
    return true;
}

/* -------------------------------------------------------------------------- */
/* One-objective terrain-aware cost                                            */
/* -------------------------------------------------------------------------- */
class TerrainObjective : public ob::StateCostIntegralObjective {
public:
    TerrainObjective(const ob::SpaceInformationPtr& si,
                     std::shared_ptr<DEMMap> dem,
                     std::shared_ptr<ConstraintChecker> chk,
                     double wLen, double wElev, double wSlope,
                     double elevScale,
                     double goalX, double goalY,
                     double wHead)
        : ob::StateCostIntegralObjective(si, /*interpolate=*/true),
          dem_(std::move(dem)), chk_(std::move(chk)),
          wLen_(wLen), wElev_(wElev), wSlope_(wSlope),
          elevScale_(elevScale), gx_(goalX), gy_(goalY), wHead_(wHead) {}

    ob::Cost stateCost(const ob::State *s) const override {
        const auto *ds = s->as<ob::DubinsStateSpace::StateType>();
        const double x = ds->getX(), y = ds->getY();
        const double yaw = ds->getYaw();

        const double elev  = std::max(0.0, dem_->getElevation(x, y));
        const double slope = chk_->computeLocalSlope(x, y);
        const double elevNorm = elev / elevScale_;

        // Alignment with local bearing-to-goal for THIS leg:
        const double brg = std::atan2(gy_ - y, gx_ - x);
        double d = std::fabs(brg - yaw);
        d = std::fmod(d, 2 * M_PI);
        if (d > M_PI) d = 2 * M_PI - d;
        const double headPenalty = 1.0 - std::cos(d); // smooth, 0 when aligned

        const double density = wLen_ + wElev_ * elevNorm + wSlope_ * slope + wHead_ * headPenalty;
        return ob::Cost(density);
    }

private:
    std::shared_ptr<DEMMap> dem_;
    std::shared_ptr<ConstraintChecker> chk_;
    double wLen_, wElev_, wSlope_, elevScale_;
    double gx_, gy_, wHead_;
};

/* -------------------------------------------------------------------------- */
/* Helpers                                                                     */
/* -------------------------------------------------------------------------- */

static ob::RealVectorBounds makeLegBounds(double sx, double sy, double gx, double gy, double marginMeters) {
    ob::RealVectorBounds b(2);
    const double minX = std::min(sx, gx) - marginMeters;
    const double maxX = std::max(sx, gx) + marginMeters;
    const double minY = std::min(sy, gy) - marginMeters;
    const double maxY = std::max(sy, gy) + marginMeters;
    b.setLow(0, minX);  b.setHigh(0, maxX);
    b.setLow(1, minY);  b.setHigh(1, maxY);
    return b;
}

static inline double bearing(double x1, double y1, double x2, double y2) {
    return std::atan2(y2 - y1, x2 - x1);
}

static inline double clamp01(double t) { return std::max(0.0, std::min(1.0, t)); }

static double distancePointToSegment(double px, double py,
                                     double x1, double y1,
                                     double x2, double y2)
{
    const double vx = x2 - x1, vy = y2 - y1;
    const double v2 = vx*vx + vy*vy;
    if (v2 < 1e-12) return std::hypot(px - x1, py - y1);
    double t = ((px - x1)*vx + (py - y1)*vy) / v2;
    t = clamp01(t);
    const double projx = x1 + t*vx, projy = y1 + t*vy;
    return std::hypot(px - projx, py - projy);
}

static bool segmentIntersectsCircle(double sx, double sy,
                                    double gx, double gy,
                                    double cx, double cy,
                                    double radiusPlusMargin)
{
    return distancePointToSegment(cx, cy, sx, sy, gx, gy) <= radiusPlusMargin;
}

static inline bool pointInsideAnyAvoid(double x, double y, int* which = nullptr) {
    for (int i = 0; i < (int)avoidZones.size(); ++i) {
        double zx, zy; demMap->latlonToUTM(avoidZones[i].lat, avoidZones[i].lon, zx, zy);
        const double dx = x - zx, dy = y - zy;
        if (std::sqrt(dx*dx + dy*dy) < avoidZones[i].radius) {
            if (which) *which = i;
            return true;
        }
    }
    return false;
}

// Sample along a straight segment and verify terrain + ceiling + avoid zones (for post-shortcut).
static bool segmentIsOK(double x1, double y1, double x2, double y2, int samples = 60)
{
    for (int i = 0; i <= samples; ++i) {
        const double t = double(i) / samples;
        const double x = x1 + t * (x2 - x1);
        const double y = y1 + t * (y2 - y1);

        const double elev = demMap->getElevation(x, y);
        if (elev <= -9000) return false;

        const double slope = checker->computeLocalSlope(x, y);
        const double minHagl = std::max(HAGL_BASE_M, HAGL_BASE_M + slope * SLOPE_GAIN);
        const double requiredAlt = elev + minHagl;

        if (requiredAlt > ALT_CEILING_M) return false;

        // In post-shortcut we keep respecting avoid zones (even if a forced leg went through one)
        if (pointInsideAnyAvoid(x, y)) return false;
    }
    return true;
}

// Replace local polyline [i..k] by a single segment if safe.
static void terrainAwareShortcut(og::PathGeometric& path)
{
    if (path.getStateCount() < 3) return;

    bool improved = true;
    int guard = 0;
    while (improved && guard++ < 8) {        // a few passes
        improved = false;

        for (std::size_t i = 0; i + 2 < path.getStateCount(); ++i) {
            // try skip j..k-1 by a direct segment i->k
            for (std::size_t k = i + 2; k < path.getStateCount(); ++k) {
                const auto* si = path.getState(i)->as<ob::DubinsStateSpace::StateType>();
                const auto* sk = path.getState(k)->as<ob::DubinsStateSpace::StateType>();

                if (!segmentIsOK(si->getX(), si->getY(), sk->getX(), sk->getY()))
                    continue;

                // perform the shortcut: keep state i and k, drop (i+1..k-1)
                std::vector<const ob::State*> keep;
                keep.reserve(path.getStateCount() - (k - i - 1));
                for (std::size_t m = 0; m <= i; ++m) keep.push_back(path.getState(m));
                keep.push_back(path.getState(k));
                for (std::size_t m = k + 1; m < path.getStateCount(); ++m) keep.push_back(path.getState(m));

                og::PathGeometric newPath(path.getSpaceInformation());
                for (auto* st : keep) newPath.append(st);
                newPath.interpolate();

                path = std::move(newPath);
                improved = true;
                break; // restart outer loop after modification
            }
            if (improved) break;
        }
    }
}

// Utility: compute max turn (deg) on a PathGeometric (optional debugging)
static double computeMaxTurnDeg(const og::PathGeometric& path) {
    double maxTurn = 0.0;
    for (std::size_t i = 2; i < path.getStateCount(); ++i) {
        const auto* s0 = path.getState(i - 2)->as<ob::DubinsStateSpace::StateType>();
        const auto* s1 = path.getState(i - 1)->as<ob::DubinsStateSpace::StateType>();
        const auto* s2 = path.getState(i    )->as<ob::DubinsStateSpace::StateType>();
        const double a1 = std::atan2(s1->getY() - s0->getY(), s1->getX() - s0->getX());
        const double a2 = std::atan2(s2->getY() - s1->getY(), s2->getX() - s1->getX());
        double diff = std::fabs(a2 - a1);
        diff = std::fmod(diff, 2 * M_PI);
        if (diff > M_PI) diff = 2 * M_PI - diff;
        maxTurn = std::max(maxTurn, diff * 180.0 / M_PI);
    }
    return maxTurn;
}

/* -------------------------------------------------------------------------- */
/* Plan a single Dubins leg                                                    */
/* -------------------------------------------------------------------------- */
static bool planLegDubins(
        double sx, double sy, double sh,
        double gx, double gy, double gh,
        double turningRadius,
        double solveSeconds,
        double goalBias,
        bool    allowAvoidZonesForThisLeg,   // <— NEW
        og::PathGeometric& outPath)
{
    // set per-leg flag
    g_ignoreAvoidZonesForThisLeg = allowAvoidZonesForThisLeg;

    auto space = std::make_shared<ob::DubinsStateSpace>(turningRadius);
    auto bounds = makeLegBounds(sx, sy, gx, gy, /*margin*/ 80000.0); // ~80 km margin box
    space->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker(isStateValid);
    auto si = ss.getSpaceInformation();
    si->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(si));
    si->setStateValidityCheckingResolution(0.005); // 0.5%

    // headings: keep nose generally toward the target
    const double sh_used = (std::isfinite(sh) ? sh : bearing(sx, sy, gx, gy));
    const double gh_used = (std::isfinite(gh) ? gh : bearing(sx, sy, gx, gy));

    ob::ScopedState<> start(space), goal(space);
    start[0] = sx; start[1] = sy; start[2] = sh_used;
    goal[0]  = gx; goal[1]  = gy; goal[2]  = gh_used;
    ss.setStartAndGoalStates(start, goal);

    auto planner = std::make_shared<og::RRTstar>(ss.getSpaceInformation());
    planner->setGoalBias(goalBias);        // try 0.45 for intermediate legs, 0.70 for final
    planner->setRange(25000.0);            // shorter steps → smoother
    ss.setPlanner(planner);

    // Terrain-aware + heading alignment objective
    const double wLen  = 1.0;
    const double wElev = 3.5;
    const double wSlope= 4.0;
    const double wHead = 3.0;
    const double elevScaleMeters = 1000.0;

    ss.setOptimizationObjective(std::make_shared<TerrainObjective>(
            si, demMap, checker, wLen, wElev, wSlope, elevScaleMeters, gx, gy, wHead));

    const bool ok = ss.solve(solveSeconds) && ss.haveSolutionPath();

    // clear the flag for safety (not strictly required because we set it each leg)
    g_ignoreAvoidZonesForThisLeg = false;

    if (!ok) return false;

    // simplify & interpolate for smooth export
    {
        og::PathGeometric path = ss.getSolutionPath();
        og::PathSimplifier ps(si);
        ps.reduceVertices(path);
        ps.smoothBSpline(path);
        path.interpolate();
        outPath = std::move(path);
    }
    return true;
}

/* -------------------------------------------------------------------------- */
/* main                                                                        */
/* -------------------------------------------------------------------------- */
int main() {
    std::cout << "OMPL version: " << OMPL_VERSION << "\n";

    // Load DEM
    demMap = std::make_shared<DEMMap>("/home/dev/CLionProjects/path_planner_project/DEM_UTM.tif");

    // Constraint checker (turn limit, climb, HAGL base)
    checker = std::make_shared<ConstraintChecker>(*demMap, /*maxTurnDeg*/ 90.0,
                                                  /*maxClimbDeg*/ 15.0, /*minHAGL*/ 30.0);

    // Start / Goal in lat/lon  (edit as needed)
    const double startLat = 31.0289, startLon = 55.1869;
    const double goalLat  = 24.6469, goalLon  = 56.5793;

    // Forced waypoints (lat/lon) — example; edit/replace as you like
    std::vector<std::pair<double,double>> forcedLL = {
            {30.5138, 54.2021}

            // {lat, lon},
            // e.g. put something inside an avoid zone to test the warning/override:
            // {29.2340, 55.6380}
    };

    // Convert to UTM
    double sx, sy, gx, gy;
    demMap->latlonToUTM(startLat, startLon, sx, sy);
    demMap->latlonToUTM(goalLat,  goalLon,  gx, gy);

    std::vector<std::pair<double,double>> forcedXY;
    forcedXY.reserve(forcedLL.size());
    for (const auto& ll : forcedLL) {
        double x, y; demMap->latlonToUTM(ll.first, ll.second, x, y);
        // warn if inside an avoid zone
        int which=-1;
        if (pointInsideAnyAvoid(x, y, &which)) {
            std::cerr << "⚠️  Forced waypoint (" << ll.first << "," << ll.second
                      << ") is inside avoid zone #" << (which+1)
                      << " — planning will IGNORE avoid-zone checks for this leg.\n";
        }
        forcedXY.emplace_back(x, y);
    }

    // If the straight line crosses the first avoid zone, we could inject a bypass waypoint.
    // (Keeping your earlier logic; but forced points take precedence if you provide them.)
    // --- optional bypass omitted for clarity when using forced points ---

    const double turnR = 3800.0;

    // Build legs: start -> forced[0] -> ... -> forced[n-1] -> goal
    std::vector<og::PathGeometric> legs;
    double curX = sx, curY = sy;
    double curH = std::numeric_limits<double>::quiet_NaN();

    for (std::size_t i = 0; i <= forcedXY.size(); ++i) {
        const bool lastLeg = (i == forcedXY.size());
        const double tx = lastLeg ? gx : forcedXY[i].first;
        const double ty = lastLeg ? gy : forcedXY[i].second;
        const double th = std::numeric_limits<double>::quiet_NaN();

        // Allow avoid-zone crossing for legs to forced points; NOT for the final leg to goal
        const bool allowAZ = !lastLeg;

        const double solveTime  = lastLeg ? 3.5 : 3.0;
        const double goalBias   = lastLeg ? 0.70 : 0.45;

        std::cout << "Leg " << (i+1) << (allowAZ ? " (forced)" : " (final)") << " …\n";

        og::PathGeometric leg((ob::SpaceInformationPtr()));
        if (!planLegDubins(curX, curY, curH, tx, ty, th,
                           turnR, solveTime, goalBias, allowAZ, leg)) {
            std::cerr << "❌ Leg " << (i+1) << " failed.\n";
            return 1;
        }

        // Advance to next leg start
        const auto* last = leg.getState(leg.getStateCount() - 1)->as<ob::DubinsStateSpace::StateType>();
        curX = last->getX(); curY = last->getY(); curH = last->getYaw();
        legs.push_back(std::move(leg));
    }

    // Stitch legs
    og::PathGeometric pathCombined(legs.front().getSpaceInformation());
    for (std::size_t i = 0; i < legs.size(); ++i) {
        for (std::size_t j = (i == 0 ? 0 : 1); j < legs[i].getStateCount(); ++j)
            pathCombined.append(legs[i].getState(j));
    }

    // Smooth (OMPL 1.8-friendly)
    {
        og::PathSimplifier ps(pathCombined.getSpaceInformation());
        ps.reduceVertices(pathCombined);
        ps.smoothBSpline(pathCombined);
        pathCombined.interpolate();
    }
    // Terrain-aware shortcut (still respects avoid zones)
    terrainAwareShortcut(pathCombined);

    std::cout << "Max turn combined: " << computeMaxTurnDeg(pathCombined) << " deg\n";

    // Export: lat/lon + terrain + flight + hagl
    std::ofstream csv("path_waypoints.csv");
    csv << std::fixed << std::setprecision(7);
    csv << "lat_deg,lon_deg,terrain_m,flight_m,hagl_m,heading_deg\n";

    const double stepMeters = 40000.0; // ~40 km sampling
    std::size_t rows = 0;

    for (std::size_t i = 1; i < pathCombined.getStateCount(); ++i) {
        const auto* a = pathCombined.getState(i - 1)->as<ob::DubinsStateSpace::StateType>();
        const auto* b = pathCombined.getState(i    )->as<ob::DubinsStateSpace::StateType>();
        const double dx = b->getX() - a->getX();
        const double dy = b->getY() - a->getY();
        const double dist = std::hypot(dx, dy);
        const int steps = std::max(1, int(dist / stepMeters));

        for (int j = 0; j <= steps; ++j) {
            const double t = double(j) / steps;
            const double x = a->getX() + t * dx;     // UTM (m)
            const double y = a->getY() + t * dy;     // UTM (m)
            const double h = a->getYaw() + t * (b->getYaw() - a->getYaw());

            double terrain = demMap->getElevation(x, y);
            if (terrain <= -9990.0) continue;

            const double slope = checker->computeLocalSlope(x, y);
            const double hagl  = std::max(150.0, 150.0 * (1.0 + 10.0 * slope));  // tune
            const double flight = terrain + hagl;

            double lat, lon; demMap->utmToLatlon(x, y, lat, lon);

            csv << lat << ',' << lon << ','
                << terrain << ',' << flight << ',' << hagl << ','
                << (h * 180.0 / M_PI) << '\n';
            ++rows;
        }
    }
    csv.close();

    std::cout << "✅ Saved " << rows << " waypoints to path_waypoints.csv\n";
    return 0;
}
