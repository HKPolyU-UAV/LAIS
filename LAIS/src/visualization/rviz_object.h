#ifndef RVIZ_OBJECT_H
#define RVIZ_OBJECT_H
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <string>
#include "../utils/eigen_typedef.h"
#include "../utils/kinetic_math.h"

using namespace std;

class rviz_object
{
private:

   ros::Publisher pose_pub;
   ros::Publisher marker_pub;
   string frame_id_pose, frame_id_objects;


public:
  rviz_object(ros::NodeHandle &nh, string Pose_frame, string Objects_frame,
              string PoseTopicName, string ObjectsTopicName, int pose_buffersize, int objects_buffersize);

  ~rviz_object();
  void visualization_geofence(const double x_max, const double x_min, const double y_max, const double y_min, const ros::Time = ros::Time::now());
  void visualization_objects(Vec3 objects);
};

#endif // RVIZ_OBJECT_H
