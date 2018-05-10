
#ifndef MAIN_H
#define MAIN_H

#include <vector>

using namespace std;


vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

extern vector<double> map_waypoints_x;
extern vector<double> map_waypoints_y;
extern vector<double> map_waypoints_s;



#endif // MAIN_H