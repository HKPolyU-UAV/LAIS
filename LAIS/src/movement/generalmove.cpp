#include <stdio.h>
#include "generalmove.h"
using namespace std;
GeneralMove::GeneralMove()
{
  vv_limit = DEFAULT_VV_LIMIT;
  hv_limit = DEFAULT_HV_LIMIT;
  av_limit = DEFAULT_AV_LIMIT;
  est_flag = 0;
}
GeneralMove::GeneralMove(double time, double start_x, double start_y, double start_z,
                         double start_yaw, double end_x, double end_y, double end_z, double end_yaw,
                         double duration)
{

  this->start_time = time;
  this->start_x = start_x;
  this->start_y = start_y;
  this->start_z = start_z;
  this->start_yaw = start_yaw;
  this->end_x = end_x;
  this->end_y = end_y;
  this->end_z = end_z;
  this->end_yaw = end_yaw;
  this->est_t = duration;
  this->vv_limit = DEFAULT_VV_LIMIT;
  this->hv_limit = DEFAULT_HV_LIMIT;
  this->av_limit = DEFAULT_AV_LIMIT;
  this->x_max = X_MAX;
  this->y_max = Y_MAX;
  this->x_min = X_MIN;
  this->y_min = Y_MIN;
  this->est_flag = 0;
  if(!(start_x < x_max && start_x > x_min) || !(start_y < y_max && start_y > y_min) || !(end_x < x_max && end_x > x_min) || !(end_y < y_max && end_y > y_min) )
  {
    cout << " exceed geofence " << endl;
  }

  if(end_x > x_max){
      cout << " end_x larger than x_max " << endl;
      vector <double> tmp = cross_point(start_x, start_y, end_x, end_y, x_max, "x");
      this->end_x = tmp[0];
      this->end_y = tmp[1];
  }
  if(end_x < x_min){
      cout << " end_x smaller than x_min " << endl;
      vector <double> tmp = cross_point(start_x, start_y, end_x, end_y, x_min, "x");
      this->end_x = tmp[0];
      this->end_y = tmp[1];

  }
  if(end_y > y_max){
      cout << " end_y larger than y_max " << endl;
      vector <double> tmp = cross_point(start_x, start_y, end_x, end_y, y_max, "y");
      this->end_x = tmp[0];
      this->end_y = tmp[1];

  }
  if(end_y < y_min){
      cout << " end_y smaller than y_min " << endl;
      vector <double> tmp = cross_point(start_x, start_y, end_x, end_y, y_min, "y");
      this->end_x = tmp[0];
      this->end_y = tmp[1];

  }
  cout << "general move trajectory " <<endl;
  cout<<"start_x: "<<this->start_x<<endl;
  cout<<"start_y: "<<this->start_y<<endl;
  cout<<"start_z: "<<this->start_z<<endl;
  cout<<"start_yaw : "<<this->start_yaw<<endl;
  cout<<"end_x: "<<this->end_x<<endl;
  cout<<"end_y: "<<this->end_y<<endl;
  cout<<"end_z: "<<this->end_z<<endl;
  cout<<"end_yaw: "<<this->end_yaw<<endl;
  cout << endl;



}

void GeneralMove::getPose(double time,
                     geometry_msgs::PoseStamped& pose)
{

  curr_time = time;
  if(est_flag == 0)
  {

    if(est_t > 0 && est_t<=1000)
    {
      double vv = (sqrt(pow(start_x-end_x,2)+pow(start_y-end_y,2)))/est_t;
      double hv = (sqrt(pow(start_z-end_z,2)))/est_t;
      double av = fabs(end_yaw-start_yaw)/est_t;
      if (start_yaw>=M_PI)  start_yaw-=2*M_PI;
      if (start_yaw<=-M_PI) start_yaw+=2*M_PI;
      if (end_yaw>=M_PI)    end_yaw-=2*M_PI;
      if (end_yaw<=-M_PI)   end_yaw+=2*M_PI;
      double d_yaw = end_yaw - start_yaw;
      if (d_yaw>=M_PI)  d_yaw-=2*M_PI;
      if (d_yaw<=-M_PI) d_yaw+=2*M_PI;
      if(vv > vv_limit){std::cout<<"check vertical velocity limit "<<std::endl;}
      if(hv > hv_limit){std::cout<<"check horizontal velocity limit "<<std::endl;}
      if(av > av_limit){std::cout<<"check angular velocity limit "<<std::endl;}
      if(vv < vv_limit && hv < hv_limit && av < av_limit){
        vx = (end_x - start_x)/est_t;
        vy = (end_y - start_y)/est_t;
        vz = (end_z - start_z)/est_t;
        v_yaw = d_yaw/est_t;

        est_flag = 1;
      }

    }

  }
  if(est_flag == 1)
  {

    double dt = curr_time - start_time;
//    std::cout << "dt: "<<dt<<std::endl;

    pose.pose.position.x = start_x + vx*dt;
    pose.pose.position.y = start_y + vy*dt;
    pose.pose.position.z = start_z + vz*dt;

    Quaterniond q = rpy2Q(Vec3(0,0,start_yaw+v_yaw*dt));
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
  }
}

bool GeneralMove::finished(){

  if(curr_time-start_time >= est_t)
  {
    est_flag = 2;
    return true;

  }
  else {
    return false;
  }
}
void GeneralMove::getEnding(double &x, double &y, double &z, double &yaw)
{
  x = end_x;
  y = end_y;
  z = end_z;
  yaw = end_yaw;
}

