#ifndef CONSTANT_H
#define CONSTANT_H

#include <map>
#include <iostream>
#include <vector>

const double dt = 0.02;                       //unit time for which didt is cal
const double T = 2.0;			      //JMT period
const int N_SAMPLES = 10;
const double VEHICLE_RADIUS = 1.5;
const double T_diff = 4.0;                    //used in time diff cost, if time diff of traj is > 4 
const double SPEED_LIMIT = 49.5;

const double  MPH_TO_METERS_PER_SEC = 2.24;   // MPH to meters-per-second, 2.24mph -> 1 m/s
const double  SPLINE_SPACING = 40;            // 30 meters between spline segments
const int     MAX_TICKS = 30;                 // number of simulator "ticks" before behavior planner processing for ego
const double  SECS_PER_TICK = 0.02;           // number of elapsed seconds per "tick" of simulator
const int     HORIZON_DIST = 50;              // 50 meters
#define MAX_JERK  10                          // m/s/s/s
#define MAX_ACCEL 10                          // m/s/s

#define EXPECTED_JERK_IN_ONE_SEC 2            // m/s/s
#define EXPECTED_ACC_IN_ONE_SEC 1             // m/s
#define LANE_WIDTH 4
#define KL       1
#define PRP_LC   2
#define LC       3
#define LC_Done  4
//#define PRP_LC 4
#define lane_left 0
#define lane_right 2
#define lane_middle 1
#define buffer 40 //m dist buffer for deceleration

#endif
