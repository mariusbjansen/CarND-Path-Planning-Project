#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <map>
#include <string>

#include "main.hpp"
#include "prediction.hpp"
#include "spline.h"

void funcTest() { std::cout << "v_ego " << std::endl; }

double absSize2D(double x, double y) { return sqrt(x * x + y * y); }

Lane determineLane(double d) {
  if (d > 0 && d < 4) {
    return LEFT;
  } else if (d >= 4 && d < 8) {
    return MIDDLE;
  } else if (d >= 8 && d < 12) {
    return RIGHT;
  }
}

bool isRelevant(double s, double ego_s) {
  double thres_relevant = 45.;
  if (fabs(s - ego_s) < thres_relevant) {
    return true;
  } else {
    return false;
  }
}

Trajectory trajectoryCalc(const VehicleState& car, uint8_t n_step) {
  // starting at current s
  double current_s = car.m_s;
  Trajectory trajectory;
  for (auto i = 0; i < n_step; ++i) {
    XY xy = getXY(current_s, car.m_d, map_waypoints_s, map_waypoints_x,
                  map_waypoints_y);
    VehicleState step_car(car.m_id, xy[0], xy[1], car.m_vx,
                          car.m_vy, current_s, car.m_d, car.m_lane);
    trajectory.push_back(step_car);
    current_s += absSize2D(car.m_vx, car.m_vy);
  }
  return trajectory;
}