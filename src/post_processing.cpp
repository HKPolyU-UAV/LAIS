#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "movement/generalmove.h"
#include "movement/circle.h"
#include <iostream>
#include "utils/kinetic_math.h"
#include "utils/yamlRead.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#define PI (3.1415926)
using namespace std;

//static string video_save_path = "/home/sy/fyrws/src/LAIS/result/vo_flight/2.avi";
//static cv::VideoWriter writer(video_save_path, cv::VideoWriter::fourcc('M', 'P', 'E', 'G'), 15, cv::Size(640,480));
void visualizeDepthImg(cv::Mat& visualized_depth, cv::Mat depth)
{
    cv::Mat d_img = depth;
    int size=d_img.cols*d_img.rows;
    for(int i=0; i<size; i++)
    {
        if(isnan(d_img.at<ushort>(i)))
        {
            d_img.at<ushort>(i)=0;
        }
        if(d_img.at<ushort>(i)>10000||d_img.at<ushort>(i)<100)
        {
            d_img.at<ushort>(i)=0;
        }
    }
    cv::Mat adjMap;
    d_img.convertTo(adjMap,CV_8UC1, 255 / (10000.0), 0);
    cv::applyColorMap(adjMap, visualized_depth, cv::COLORMAP_RAINBOW);
    for(int i=0; i<size; i++)
    {
        if(d_img.at<ushort>(i)==0)
        {
            cv::Vec3b color = visualized_depth.at<cv::Vec3b>(i);
            color[0]=255;
            color[1]=255;
            color[2]=255;
            visualized_depth.at<cv::Vec3b>(i)=color;
        }
    }
}

void depth_img_cb(const sensor_msgs::ImageConstPtr & msg)
{
  cout<<"1 "<<endl;
  cv_bridge::CvImagePtr depth_ptr  = cv_bridge::toCvCopy(msg, msg->encoding);
  cv::Mat depth_img = depth_ptr->image;
  cv::Mat colorized_depth;
  visualizeDepthImg(colorized_depth, depth_img);
  cv::namedWindow("depth_color");

  string file_path = "/home/sy/fyrws/src/LAIS/result/depth_post/";
  static int count = 0;
  try {
    string save_path = file_path+to_string(count)+".jpg";
    //cout<<"save path "<< save_path<<endl;
    cv::imwrite(save_path,colorized_depth);
    count++;
    cv::imshow("depth_color", colorized_depth);
    cv::waitKey(1);
//    writer << image_rgb;   //write video
  }
  catch (cv_bridge::Exception & e)
  {
    ROS_ERROR("cv_bridge depth_img_cb exception: %s", e.what());
    return;
  }


}

void rgb_compressed_cb(const sensor_msgs::ImageConstPtr & msg)
{

  cv::Mat image_rgb;
  cv_bridge::CvImagePtr a;
  //image_rgb = cv::imdecode(cv::Mat(msg->data),1);

  a = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
  image_rgb = a->image;
  string file_path = "/home/sy/fyrws/src/LAIS/result/rgb_post/";
  static int count = 0;
  try {
    string save_path = file_path+to_string(count)+".jpg";
    //cout<<"save path "<< save_path<<endl;
    cv::imwrite(save_path,image_rgb);
    count++;
    cv::imshow("helloimage", image_rgb);
    cv::waitKey(1);
//    writer << image_rgb;   //write video
  }
  catch (cv_bridge::Exception & e)
  {
    ROS_ERROR("cv_bridge rgb_img_cb exception: %s", e.what());
    return;
  }

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "post_processing");
  ros::NodeHandle nh("~");
  ros::Subscriber sub = nh.subscribe("/camera/color/image_raw/", 1, rgb_compressed_cb);
  ros::Subscriber sub_dep = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depth_img_cb);
  ros::Rate loop_rate(30);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
