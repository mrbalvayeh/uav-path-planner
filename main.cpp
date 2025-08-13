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

/* ------------------ Globals / knobs ------------------ */
std::shared_ptr<DEMMap> demMap;
std::shared_ptr<ConstraintChecker> checker;

constexpr double ALT_CEILING_M = 6000.0;   // hard MSL ceiling
constexpr double HAGL_BASE_M   = 30.0;     // base min HAGL
constexpr double SLOPE_GAIN    = 3000.0;   // slope→HAGL gain

struct AvoidZone { double lat, lon, radius; };
std::vector<AvoidZone> avoidZones = {
        // edit as needed
        {31.3896, 52.0294, 20000.0},
        {30.5186, 51.4226, 20000.0},
        {29.7384, 50.8974, 20000.0},
        {28.9582, 50.3839, 20000.0},
};

/* ------------------ Validity checker ------------------ */
bool isStateValid(const ob::State* state) {
    const auto* s = state->as<ob::DubinsStateSpace::StateType>();
    const double x = s->getX();
    const double y = s->getY();

    const double elev = demMap->getElevation(x, y);
    if (elev <= -9000) return false; // OOB / nodata

    const double slope = checker->computeLocalSlope(x, y);
    const double minHaglHere = std::max(HAGL_BASE_M, HAGL_BASE_M + slope * SLOPE_GAIN);
    const double requiredAltMSL = elev + minHaglHere;
    if (requiredAltMSL > ALT_CEILING_M) return false; // hard ceiling

    // circular avoid zones
    for (const auto& zone : avoidZones) {
        double zx, zy; demMap->latlonToUTM(zone.lat, zone.lon, zx, zy);
        const double dx = x - zx, dy = y - zy;
        if (std::sqrt(dx*dx + dy*dy) < zone.radius) return false;
    }
    return true;
}

/* ---------- One-objective terrain/heading cost -------- */
class TerrainObjective : public ob::StateCostIntegralObjective {
public:
    TerrainObjective(const ob::SpaceInformationPtr& si,
                     std::shared_ptr<DEMMap> dem,
                     std::shared_ptr<ConstraintChecker> chk,
                     double wLen, double wElev, double wSlope,
                     double elevScale,
                     double gx, double gy,
                     double wHead)
        : ob::StateCostIntegralObjective(si, true),
          dem_(std::move(dem)), chk_(std::move(chk)),
          wLen_(wLen), wElev_(wElev), wSlope_(wSlope),
          elevScale_(elevScale), gx_(gx), gy_(gy), wHead_(wHead) {}

    ob::Cost stateCost(const ob::State* s) const override {
        const auto* ds = s->as<ob::DubinsStateSpace::StateType>();
        const double x = ds->getX(), y = ds->getY(), yaw = ds->getYaw();

        const double elev  = std::max(0.0, dem_->getElevation(x, y));
        const double slope = chk_->computeLocalSlope(x, y);
        const double elevNorm = elev / elevScale_;

        // heading alignment to current leg goal
        const double brg = std::atan2(gy_ - y, gx_ - x);
        double d = std::fabs(brg - yaw);
        d = std::fmod(d, 2*M_PI);
        if (d > M_PI) d = 2*M_PI - d;
        const double headPenalty = 1.0 - std::cos(d);

        const double density = wLen_ + wElev_*elevNorm + wSlope_*slope + wHead_*headPenalty;
        return ob::Cost(density);
    }

private:
    std::shared_ptr<DEMMap> dem_;
    std::shared_ptr<ConstraintChecker> chk_;
    double wLen_, wElev_, wSlope_, elevScale_;
    double gx_, gy_, wHead_;
};

/* ---------------------- Helpers ----------------------- */
static ob::RealVectorBounds makeLegBounds(double sx, double sy, double gx, double gy, double marginMeters) {
    ob::RealVectorBounds b(2);
    b.setLow(0, std::min(sx, gx) - marginMeters);
    b.setHigh(0, std::max(sx, gx) + marginMeters);
    b.setLow(1, std::min(sy, gy) - marginMeters);
    b.setHigh(1, std::max(sy, gy) + marginMeters);
    return b;
}

static inline double bearing(double x1, double y1, double x2, double y2) {
    return std::atan2(y2 - y1, x2 - x1);
}

static inline bool waypointUsable(double x, double y) {
    // same checks as in isStateValid but for an (x,y) point
    const double elev = demMap->getElevation(x, y);
    if (elev <= -9000) return false;
    const double slope = checker->computeLocalSlope(x, y);
    const double minHaglHere = std::max(HAGL_BASE_M, HAGL_BASE_M + slope * SLOPE_GAIN);
    if (elev + minHaglHere > ALT_CEILING_M) return false;
    for (const auto& z : avoidZones) {
        double zx, zy; demMap->latlonToUTM(z.lat, z.lon, zx, zy);
        const double dx = x - zx, dy = y - zy;
        if (std::sqrt(dx*dx + dy*dy) < z.radius) return false;
    }
    return true;
}

static bool segmentIsOK(double x1, double y1, double x2, double y2, int samples = 60) {
    for (int i = 0; i <= samples; ++i) {
        const double t = double(i) / samples;
        const double x = x1 + t * (x2 - x1);
        const double y = y1 + t * (y2 - y1);
        if (!waypointUsable(x, y)) return false;
    }
    return true;
}

static double computeMaxTurnDeg(const og::PathGeometric& path) {
    double maxTurn = 0.0;
    for (std::size_t i = 2; i < path.getStateCount(); ++i) {
        const auto* s0 = path.getState(i-2)->as<ob::DubinsStateSpace::StateType>();
        const auto* s1 = path.getState(i-1)->as<ob::DubinsStateSpace::StateType>();
        const auto* s2 = path.getState(i  )->as<ob::DubinsStateSpace::StateType>();
        const double a1 = std::atan2(s1->getY() - s0->getY(), s1->getX() - s0->getX());
        const double a2 = std::atan2(s2->getY() - s1->getY(), s2->getX() - s1->getX());
        double d = std::fabs(a2 - a1);
        d = std::fmod(d, 2*M_PI);
        if (d > M_PI) d = 2*M_PI - d;
        maxTurn = std::max(maxTurn, d * 180.0/M_PI);
    }
    return maxTurn;
}

/* ------------- Terrain-aware shortcutting ------------- */
static void terrainAwareShortcut(og::PathGeometric& path) {
    if (path.getStateCount() < 3) return;
    bool improved = true;
    int passes = 0;
    while (improved && passes++ < 6) {
        improved = false;
        for (std::size_t i = 0; i + 2 < path.getStateCount(); ++i) {
            for (std::size_t k = i + 2; k < path.getStateCount(); ++k) {
                const auto* si = path.getState(i)->as<ob::DubinsStateSpace::StateType>();
                const auto* sk = path.getState(k)->as<ob::DubinsStateSpace::StateType>();
                if (!segmentIsOK(si->getX(), si->getY(), sk->getX(), sk->getY()))
                    continue;

                // splice
                std::vector<const ob::State*> keep;
                for (std::size_t m = 0; m <= i; ++m) keep.push_back(path.getState(m));
                keep.push_back(path.getState(k));
                for (std::size_t m = k + 1; m < path.getStateCount(); ++m) keep.push_back(path.getState(m));
                og::PathGeometric np(path.getSpaceInformation());
                for (auto* st : keep) np.append(st);
                np.interpolate();
                path = std::move(np);
                improved = true;
                break;
            }
            if (improved) break;
        }
    }
}

/* --------------------- Planning leg ------------------- */
struct PlannerRamp {
    double marginMeters{80000.0};
    double rangeMeters{25000.0};
    double timeSec{3.0};
    double goalBias{0.45};
};

static bool planLegDubins(double sx, double sy, double sh,
                          double gx, double gy, double gh,
                          double turningRadius,
                          const std::vector<PlannerRamp>& ramps,
                          og::PathGeometric& out)
{
    for (std::size_t r = 0; r < ramps.size(); ++r) {
        const auto& rp = ramps[r];
        std::cout << "    [ramp " << (r+1) << "] margin=" << int(rp.marginMeters)
                  << " range=" << int(rp.rangeMeters)
                  << " time=" << rp.timeSec
                  << " goalBias=" << rp.goalBias << '\n';

        auto space  = std::make_shared<ob::DubinsStateSpace>(turningRadius);
        auto bounds = makeLegBounds(sx, sy, gx, gy, rp.marginMeters);
        space->setBounds(bounds);

        og::SimpleSetup ss(space);
        ss.setStateValidityChecker(isStateValid);
        auto si = ss.getSpaceInformation();
        si->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(si));
        si->setStateValidityCheckingResolution(0.005);

        const double sh_used = std::isfinite(sh) ? sh : bearing(sx, sy, gx, gy);
        const double gh_used = std::isfinite(gh) ? gh : bearing(sx, sy, gx, gy);

        ob::ScopedState<> s(space), g(space);
        s[0]=sx; s[1]=sy; s[2]=sh_used;
        g[0]=gx; g[1]=gy; g[2]=gh_used;
        ss.setStartAndGoalStates(s, g);

        auto planner = std::make_shared<og::RRTstar>(ss.getSpaceInformation());
        planner->setGoalBias(rp.goalBias);
        planner->setRange(rp.rangeMeters);
        ss.setPlanner(planner);

        // terrain/heading objective
        const double wLen=1.0, wElev=3.5, wSlope=4.0, wHead=3.0, elevScale=1000.0;
        ss.setOptimizationObjective(std::make_shared<TerrainObjective>(
                si, demMap, checker, wLen, wElev, wSlope, elevScale, gx, gy, wHead));

        bool ok = ss.solve(rp.timeSec) && ss.haveSolutionPath();
        if (!ok) {
            std::cout << "      -> no solution on this ramp, trying next…\n";
            continue;
        }

        ss.simplifySolution();
        out = ss.getSolutionPath();
        out.interpolate();
        return true;
    }
    return false;
}

/* ------------------------- main ----------------------- */
int main() {
    std::cout << "OMPL version: " << OMPL_VERSION << "\n";

    demMap  = std::make_shared<DEMMap>("/home/dev/CLionProjects/path_planner_project/DEM_UTM.tif");
    checker = std::make_shared<ConstraintChecker>(*demMap, 90.0, 15.0, 30.0);

    // Start / goal (edit as needed)
    const double startLat = 31.9649, startLon = 52.4022;
    const double goalLat  = 28.5186, goalLon  = 50.1269;

    // Forced waypoints (lat/lon) — example with two
    std::vector<std::pair<double,double>> forcedLL = {
            {31.2000, 51.8000},
            {30.2000, 50.9000}
    };

    // Convert to UTM
    double sx, sy, gx, gy;
    demMap->latlonToUTM(startLat, startLon, sx, sy);
    demMap->latlonToUTM(goalLat,  goalLon,  gx, gy);

    std::vector<std::pair<double,double>> forcedXY;
    forcedXY.reserve(forcedLL.size());
    for (auto& ll : forcedLL) {
        double x,y; demMap->latlonToUTM(ll.first, ll.second, x, y);
        forcedXY.emplace_back(x,y);
    }

    // Pre‑check forced waypoints so we don't stall on impossible ones
    for (std::size_t i=0;i<forcedXY.size();++i) {
        const auto [fx,fy] = forcedXY[i];
        if (!waypointUsable(fx, fy)) {
            std::cerr << "  Forced waypoint #" << (i+1)
                      << " is unusable (avoid/ceiling/DEM). "
                         "Nudge it or raise ALT_CEILING_M.\n";
            // You can auto‑nudge here if you want.
        }
    }

    std::cout << "Planning through " << forcedXY.size() << " forced waypoint(s)…\n";

    const double turnR = 3800.0;
    const std::vector<PlannerRamp> ramps1 = {
            {80000.0, 25000.0, 3.0, 0.45},
            {120000.0, 35000.0, 4.5, 0.55},
    };
    const std::vector<PlannerRamp> ramps2 = {
            {80000.0, 25000.0, 3.5, 0.70},
            {120000.0, 35000.0, 4.5, 0.75},
    };

    // Build legs: start -> forced[0] -> ... -> forced[n-1] -> goal
    std::vector<og::PathGeometric> legs;
    double curX = sx, curY = sy;
    double curH = std::numeric_limits<double>::quiet_NaN();

    for (std::size_t i=0;i<=forcedXY.size();++i) {
        const bool lastLeg = (i == forcedXY.size());
        const double tx = lastLeg ? gx : forcedXY[i].first;
        const double ty = lastLeg ? gy : forcedXY[i].second;
        const double th = std::numeric_limits<double>::quiet_NaN();

        std::cout << "  Leg " << (i+1) << " …\n";
        og::PathGeometric leg((ob::SpaceInformationPtr()));
        const auto& ramps = lastLeg ? ramps2 : ramps1;
        if (!planLegDubins(curX, curY, curH, tx, ty, th, turnR, ramps, leg)) {
            std::cerr << "  ❌ Leg " << (i+1) << " failed across all ramps.\n";
            return 1;
        }
        // advance
        const auto* last = leg.getState(leg.getStateCount()-1)->as<ob::DubinsStateSpace::StateType>();
        curX = last->getX(); curY = last->getY(); curH = last->getYaw();
        legs.push_back(std::move(leg));
    }

    // Stitch
    og::PathGeometric combined(legs.front().getSpaceInformation());
    for (std::size_t i=0;i<legs.size();++i) {
        for (std::size_t j = (i==0?0:1); j<legs[i].getStateCount(); ++j)
            combined.append(legs[i].getState(j));
    }

    // OMPL‑1.8 smoothing: reduceVertices + ropeShortcutPath
    {
        og::PathSimplifier ps(combined.getSpaceInformation());
        ps.reduceVertices(combined);
        ps.ropeShortcutPath(combined);
        combined.interpolate();
    }
    // Terrain-aware post shortcut
    terrainAwareShortcut(combined);

    std::cout << "Max turn combined: " << computeMaxTurnDeg(combined) << " deg\n";

    // Export (terrain + flight + hagl)
    std::ofstream csv("path_waypoints.csv");
    csv << std::fixed << std::setprecision(7);
    csv << "lat_deg,lon_deg,terrain_m,flight_m,hagl_m,heading_deg\n";

    const double stepMeters = 40000.0;
    for (std::size_t i = 1; i < combined.getStateCount(); ++i) {
        const auto* a = combined.getState(i-1)->as<ob::DubinsStateSpace::StateType>();
        const auto* b = combined.getState(i  )->as<ob::DubinsStateSpace::StateType>();
        const double dx = b->getX() - a->getX();
        const double dy = b->getY() - a->getY();
        const double dist = std::hypot(dx, dy);
        const int steps = std::max(1, int(dist / stepMeters));

        for (int j=0;j<=steps;++j) {
            const double t = double(j)/steps;
            const double x = a->getX() + t*dx;
            const double y = a->getY() + t*dy;
            const double h = a->getYaw() + t*(b->getYaw() - a->getYaw());

            double terrain = demMap->getElevation(x, y);
            if (terrain <= -9990.0) continue;
            const double slope = checker->computeLocalSlope(x, y);
            const double hagl  = std::max(150.0, 150.0*(1.0 + 10.0*slope)); // tune
            const double flight = std::min(ALT_CEILING_M, terrain + hagl);

            double lat, lon; demMap->utmToLatlon(x, y, lat, lon);
            csv << lat << ',' << lon << ',' << terrain << ',' << flight << ',' << hagl << ','
                << (h*180.0/M_PI) << '\n';
        }
    }
    csv.close();

    std::cout << "✅ Saved waypoints to path_waypoints.csv\n";
    return 0;
}
