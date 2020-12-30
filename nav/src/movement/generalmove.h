#ifndef GENERALMOVE_H
#define GENERALMOVE_H
#include <stdio.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include "../utils/kinetic_math.h"
#include "../utils/yamlRead.h"
#include "trj_config.h"
class GeneralMove
{
 private:
  double start_x, start_y, start_z, start_yaw;
  double end_x, end_y, end_z, end_yaw;
  double start_time, curr_time;

  double vv_limit, hv_limit, av_limit;
  double x_max, y_max, x_min, y_min;
  double vx, vy, vz, v_yaw;
  int est_flag;
  double est_t;

 public:
  GeneralMove();
  GeneralMove(double start_time, double start_x, double start_y, double start_z, double start_yaw,
              double end_x, double end_y, double end_z, double end_yaw, double duration);
  void getPose(double curr_time, geometry_msgs::PoseStamped& pose);
  void getEnding(double &x, double &y, double &z, double &yaw);
  bool finished();


};

#endif // GENERALMOVE_H
