#ifndef PREDICTION_H
#define PREDICTION_H
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using namespace std;

void funcTest();

enum Lane { LEFT, MIDDLE, RIGHT };

struct VehicleState {
  // given by sensor fusion_data
  uint8_t m_id;  // car's unique ID
  float m_x;     // car's x position in map coordinates
  float m_y;     // car's y position in map coordinates
  float m_vx;    // car's x velocity in m/s
  float m_vy;    // car's y velocity in m/s
  float m_s;     // car's s position in frenet coordinates
  float m_d;     // car's d position in frenet coordinates
  // added
  Lane m_lane;  // car's lane position

  VehicleState(uint8_t id, float x, float y, float vx, float vy, float s,
               float d, Lane l) {
    m_id = id;
    m_x = x;
    m_y = y;
    m_vx = vx;
    m_vy = vy;
    m_s = s;
    m_d = d;
    m_lane = l;
  }

  VehicleState() {}
};

using Trajectory = vector<VehicleState>;
using XY = vector<double>;

Trajectory trajectoryCalc(const VehicleState& car, uint8_t n_step);
double velocityTarAheadinLane(const vector<VehicleState>& vehicles,
                              const VehicleState& ego, uint8_t lane);
double absSize2D(double x, double y);
Lane determineLane(double d);
bool isRelevant(double s, double ego_s);

bool isCollisionFree(const VehicleState& ego, const vector<VehicleState>& targets, uint8_t n_step);

#endif  // PREDICTION_H
