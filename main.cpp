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
#include <limits>  // for std::numeric_limits

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
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

// Hardcode one example (add more if you want)
std::vector<AvoidZone> avoidZones = {
        {26.9684, 54.393, 20000.0},{28.2329, 54.0702, 40000.0},{29.919, 53.4707, 30000.0},{29.234, 55.638, 30000.0} // lat, lon, radius (m)
};

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

    // Dynamic HAGL (your existing rule)
    const double slope = checker->computeLocalSlope(x, y);
    const double minHaglHere = std::max(HAGL_BASE_M, HAGL_BASE_M + slope * SLOPE_GAIN);

    // The altitude we'd have to fly to remain legal here:
    const double requiredAltMSL = elev + minHaglHere;

    // HARD CEILING
    if (requiredAltMSL > ALT_CEILING_M) return false;

    // Circular avoid zones (hard)
    for (const auto& zone : avoidZones) {
        double zx, zy; demMap->latlonToUTM(zone.lat, zone.lon, zx, zy);
        const double dx = x - zx, dy = y - zy;
        if (std::sqrt(dx * dx + dy * dy) < zone.radius) return false;
    }
    return true;
}

/* -------------------------------------------------------------------------- */
/* One-objective terrain-aware cost (works on OMPL 1.8)                        */
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
        const double bearing = std::atan2(gy_ - y, gx_ - x);
        double d = std::fabs(bearing - yaw);
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

// Tight bounding box around a leg to focus sampling
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

// Compute one "bypass" waypoint skirting a circular zone
static std::pair<double, double> computeBypassUTM(
        double sx, double sy, double gx, double gy,
        double zx, double zy, double radius, double marginMeters)
{
    const double vx = gx - sx,  vy = gy - sy;
    const double v2 = vx * vx + vy * vy;
    if (v2 < 1e-8) return {sx, sy};      // degenerate

    // project center onto the line
    const double t  = ((zx - sx) * vx + (zy - sy) * vy) / v2;
    const double px = sx + t * vx, py = sy + t * vy;

    // unit perpendicular
    const double v  = std::sqrt(v2);
    const double nx = -vy / v, ny = vx / v;

    // choose side that increases distance from circle center
    const double dx = px - zx, dy = py - zy;
    const double side = (dx * nx + dy * ny) >= 0 ? 1.0 : -1.0;

    const double off = radius + marginMeters;
    return {px + side * off * nx, py + side * off * ny};
}
static inline double bearing(double x1, double y1, double x2, double y2) {
    return std::atan2(y2 - y1, x2 - x1);
}

// Plan a single Dubins leg (fresh SimpleSetup with tight bounds)
static bool planLegDubins(
        double sx, double sy, double sh,
        double gx, double gy, double gh,
        double turningRadius,
        double solveSeconds,
        double goalBias,
        og::PathGeometric& outPath)
{
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
    planner->setGoalBias(goalBias);        // try 0.45 for leg1, 0.70 for leg2
    planner->setRange(25000.0);            // shorter steps → smoother
    ss.setPlanner(planner);

    // Terrain-aware + heading alignment objective
    const double wLen  = 1.0;
    const double wElev = 3.5;
    const double wSlope= 4.0;
    const double wHead = 3.0;              // <— new: heading alignment weight
    const double elevScaleMeters = 1000.0;

    ss.setOptimizationObjective(std::make_shared<TerrainObjective>(
            si, demMap, checker, wLen, wElev, wSlope, elevScaleMeters, gx, gy, wHead));

    if (!ss.solve(solveSeconds) || !ss.haveSolutionPath())
        return false;

    // simplify & interpolate for smooth export
    ss.simplifySolution();                 // shortcut + prune (safe for Dubins)
    outPath = ss.getSolutionPath();
    outPath.interpolate();
    return true;
}

// Utility: compute max turn (deg) on a PathGeometric (optional debugging)
static double computeMaxTurnDeg(const og::PathGeometric& path) {
    double maxTurn = 0.0;
    for (std::size_t i = 2; i < path.getStateCount(); ++i) {
        const auto* s0 = path.getState(i - 2)->as<ob::DubinsStateSpace::StateType>();
        const auto* s1 = path.getState(i - 1)->as<ob::DubinsStateSpace::StateType>();
        const auto* s2 = path.getState(i)->as<ob::DubinsStateSpace::StateType>();
        const double a1 = std::atan2(s1->getY() - s0->getY(), s1->getX() - s0->getX());
        const double a2 = std::atan2(s2->getY() - s1->getY(), s2->getX() - s1->getX());
        double diff = std::fabs(a2 - a1);
        diff = std::fmod(diff, 2 * M_PI);
        if (diff > M_PI) diff = 2 * M_PI - diff;
        maxTurn = std::max(maxTurn, diff * 180.0 / M_PI);
    }
    return maxTurn;
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


// Sample along a straight segment and verify terrain + ceiling + avoid zones.
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

        for (const auto& zone : avoidZones) {
            double zx, zy; demMap->latlonToUTM(zone.lat, zone.lon, zx, zy);
            const double dx = x - zx, dy = y - zy;
            if (std::sqrt(dx * dx + dy * dy) < zone.radius) return false;
        }
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

/* -------------------------------------------------------------------------- */
/* main                                                                        */
/* -------------------------------------------------------------------------- */

int main() {
    std::cout << "OMPL version: " << OMPL_VERSION << "\n";

    // Load DEM
    demMap = std::make_shared<DEMMap>("/home/dev/CLionProjects/path_planner_project/DEM_UTM.tif");

    // Constraint checker (turn limit, climb, HAGL base). Values not critical here.
    checker = std::make_shared<ConstraintChecker>(*demMap, /*maxTurnDeg*/ 90.0,
            /*maxClimbDeg*/ 15.0, /*minHAGL*/ 30.0);

    // Start / Goal in lat/lon
    const double startLat = 31.3417, startLon = 55.3152;
    const double goalLat  = 25.4403, goalLon  = 53.6552;
//    const double startLat = 31.9649, startLon = 52.4022;
//    const double goalLat  = 28.5186, goalLon  = 50.1269;

    // Convert to UTM
    double startX, startY, goalX, goalY;
    demMap->latlonToUTM(startLat, startLon, startX, startY);
    demMap->latlonToUTM(goalLat,  goalLon,  goalX,  goalY);

    // Compute one bypass waypoint using the first avoid zone (extend if multiple)
    // ---- Compute center of the first (or most relevant) avoid zone in UTM
    double zx, zy;
    {
        const auto& z = avoidZones.front();
        demMap->latlonToUTM(z.lat, z.lon, zx, zy);
    }
    const double bypassMargin = 10000.0; // 10 km extra safety if we do bypass
    const double neededClear  = avoidZones.front().radius + bypassMargin;

// Do we actually need to go around this circle?
    const bool needBypass = segmentIntersectsCircle(startX, startY, goalX, goalY,
                                                    zx, zy, neededClear);

    const double turnR   = 3800.0;
    const double legTime1 = 3.0;
    const double legTime2 = 3.5;

    og::PathGeometric pathCombined((ob::SpaceInformationPtr()));

    if (!needBypass) {
        // Single leg: start -> goal
        std::cout << "Single leg (start -> goal)…\n";
        og::PathGeometric leg((ob::SpaceInformationPtr()));
        if (!planLegDubins(startX, startY, std::numeric_limits<double>::quiet_NaN(),
                           goalX,  goalY,  std::numeric_limits<double>::quiet_NaN(),
                           turnR, legTime2, /*goalBias*/ 0.65, leg)) {
            std::cerr << "Planning failed.\n";
            return 1;
        }
        pathCombined = og::PathGeometric(leg.getSpaceInformation());
        for (std::size_t i = 0; i < leg.getStateCount(); ++i)
            pathCombined.append(leg.getState(i));

    } else {
        // Two legs: start -> bypass -> goal
        auto [bypassX, bypassY] = computeBypassUTM(
                startX, startY, goalX, goalY, zx, zy, avoidZones.front().radius, bypassMargin);

        std::cout << "Leg 1 (start -> bypass)…\n";
        og::PathGeometric leg1((ob::SpaceInformationPtr()));
        if (!planLegDubins(startX, startY, std::numeric_limits<double>::quiet_NaN(),
                           bypassX, bypassY, std::numeric_limits<double>::quiet_NaN(),
                           turnR, legTime1, /*goalBias*/ 0.45, leg1)) {
            std::cerr << "Leg 1 failed.\n";
            return 1;
        }

        const auto* last = leg1.getState(leg1.getStateCount() - 1)
                ->as<ob::DubinsStateSpace::StateType>();
        const double cX = last->getX(), cY = last->getY();

        std::cout << "Leg 2 (bypass -> goal)…\n";
        og::PathGeometric leg2((ob::SpaceInformationPtr()));
        if (!planLegDubins(cX, cY, std::numeric_limits<double>::quiet_NaN(),
                           goalX, goalY, std::numeric_limits<double>::quiet_NaN(),
                           turnR, legTime2, /*goalBias*/ 0.70, leg2)) {
            std::cerr << "Leg 2 failed.\n";
            return 1;
        }

        pathCombined = og::PathGeometric(leg1.getSpaceInformation());
        for (std::size_t i = 0; i < leg1.getStateCount(); ++i) pathCombined.append(leg1.getState(i));
        for (std::size_t i = 1; i < leg2.getStateCount(); ++i) pathCombined.append(leg2.getState(i));
    }

    {
        og::PathSimplifier ps(pathCombined.getSpaceInformation());
        ps.reduceVertices(pathCombined);        // prune near-collinear vertices
        ps.smoothBSpline(pathCombined);         // OMPL 1.8-friendly smoother
        pathCombined.interpolate();
    }
    terrainAwareShortcut(pathCombined);

    std::cout << "Max turn combined: " << computeMaxTurnDeg(pathCombined) << " deg\n";

    // Densify/export ~ every 40 km; altitude = terrain + dynamic HAGL (for output only)
    std::vector<State> dense;
    const double stepMeters = 40000.0;

    std::ofstream csv("path_waypoints.csv");
    csv << std::fixed << std::setprecision(7);
// NEW: write explicit terrain/flight/hagl columns
    csv << "lat_deg,lon_deg,terrain_m,flight_m,hagl_m,heading_deg\n";
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

            // NEW: sample DEM in UTM, then compute flight altitude
            double terrain = demMap->getElevation(x, y);        // meters MSL (or -9999 if OOB)
            if (terrain <= -9990.0) continue;                   // skip points outside DEM

            // NEW: realistic dynamic HAGL (no 10km debug floor)
            const double slope = checker->computeLocalSlope(x, y);
            const double hagl  = std::max(150.0, 150.0 * (1.0 + 10.0 * slope));  // tune as you like
            const double flight = terrain + hagl;

            // Convert to lat/lon only for output
            double lat, lon;
            demMap->utmToLatlon(x, y, lat, lon);

            // write row
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
