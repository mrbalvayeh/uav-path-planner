#ifndef PATH_PLANNER_WAYPOINTPATH_H
#define PATH_PLANNER_WAYPOINTPATH_H


#include <vector>

/* simple state used for dense way-points */
struct State {
    double x;       // UTM easting  (m)   – or lat  if you prefer
    double y;       // UTM northing (m)   – or lon
    double z;       // altitude      (m)
    double heading; // heading deg (0-360)

    State(double X, double Y, double Z, double Hdg)
            : x(X), y(Y), z(Z), heading(Hdg) {}
};

class WaypointPath {
public:
    void add(const State &s)           { states_.push_back(s); }
    const std::vector<State>& states() const { return states_; }
    std::size_t size()       const     { return states_.size(); }
    const State& operator[](std::size_t i) const { return states_[i]; }

private:
    std::vector<State> states_;
};


#endif //PATH_PLANNER_WAYPOINTPATH_H
