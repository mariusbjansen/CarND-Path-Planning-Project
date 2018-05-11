#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <string>

#include "main.hpp"
#include "prediction.hpp"
#include "spline.h"

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
    VehicleState step_car(car.m_id, xy[0], xy[1], car.m_vx, car.m_vy, current_s,
                          car.m_d, car.m_lane);
    trajectory.push_back(step_car);
    current_s += absSize2D(car.m_vx, car.m_vy);
  }
  return trajectory;
}

double velocityTarAheadinLane(const vector<VehicleState>& vehicles,
                              const VehicleState& ego, uint8_t lane) {
  double nearest_s = numeric_limits<double>::max();
  double min_v = numeric_limits<double>::max();

  for (auto veh : vehicles) {
    if (veh.m_lane != lane) {
      continue;
    }
    if ((veh.m_s >= ego.m_s) && (veh.m_s < nearest_s)) {
      nearest_s = veh.m_s;
      min_v = absSize2D(veh.m_vx, veh.m_vy);
    }
  }

  return min_v;
}

bool isCollisionFree(const VehicleState& ego,
                     const vector<VehicleState>& targets, uint8_t n_step) {
  bool collisionFree = true;
  float minSafetyDistance = 8.;

  Trajectory egoTraj = trajectoryCalc(ego, n_step);

  for (auto target : targets) {
    // ego lane is set to the lane change target by the function caller already
    if (target.m_lane != ego.m_lane) {
      continue;
    }

    Trajectory targetTraj = trajectoryCalc(target, n_step);
    auto tarIter = targetTraj.begin();
    auto egoIter = egoTraj.begin();
    for (; tarIter != targetTraj.end(); ++tarIter, ++egoIter) {
      double distance =
          absSize2D(tarIter->m_x - egoIter->m_x, tarIter->m_y - egoIter->m_y);
      if (distance < minSafetyDistance) {
        collisionFree = false;
      }
    }
  }

  return collisionFree;
}
