#include "rviz_object.h"

rviz_object::rviz_object(ros::NodeHandle &nh, string Pose_frame, string Objects_frame,
                         string PoseTopicName, string ObjectsTopicName, int pose_buffersize, int objects_buffersize)
{

  marker_pub = nh.advertise<visualization_msgs::Marker>(ObjectsTopicName, objects_buffersize);
  this->frame_id_pose = Pose_frame;

}
rviz_object::~rviz_object(){

}
void rviz_object::visualization_geofence(const double x_max, const double x_min, const double y_max, const double y_min, const ros::Time stamp){

}
