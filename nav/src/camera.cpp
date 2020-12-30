#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include "utils/yoloNet.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include "nav/identify_command.h"
#include "nav/object.h"
#include "nav/Objects.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include "utils/kinetic_math.h"
#include "utils/yamlRead.h"
#include <numeric>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

//camera intrinsic pramater
static double fx, fy, cx, cy; //focal length and principal point

// uav local parameter
static double uav_x,uav_y,uav_z;
static double uav_qx,uav_qy,uav_qz,uav_qw;
static double uav_size = 0.5;
static geometry_msgs::PoseStamped obj_pose;
static nav::Objects Object_array;

#define radius 1
// YOLO file path
static string cfg_path = "/home/panda/Downloads/yolov4_5_classes/yolov4_8_31.cfg";
static string weight_path = "/home/panda/Downloads/yolov4_5_classes/yolov4_8_31_best.weights";
static string classid_path = "/home/panda/Downloads/yolov4_5_classes/obj.names";
static string object_save_file_path = "/home/panda/fyrws/src/nav/config/testconfig.yaml";
static std::vector <string> object_class = {"bulb", "traffic light", "cctv"};

//Creat the YOLO network
static yoloNet yolo = yoloNet(cfg_path, weight_path, classid_path, 608, 608, 0.5);

static bool detected_flag;
static bool obstacle = 0;
static bool identify_flag = true;
//static cv::VideoWriter writer(video_save_path, cv::VideoWriter::fourcc('M', 'P', 'E', 'G'), 1, cv::Size(1280,720));

//IRR filter parameter
static Mat3x3 IIR_worldframe;

static visualization_msgs::Marker point_list;

void camera_info_cb(const sensor_msgs::CameraInfoPtr& msg){

  fx = msg->K[0];
  fy = msg->K[4];
  cx = msg->K[2];
  cy = msg->K[5];
}

void identify_cb(const nav::identify_commandConstPtr & msg){

    identify_flag = msg->identify;
//    cout << identify_flag << endl;
}

void uav_lp_cb(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  //get uav pose

  uav_x = pose->pose.position.x;
  uav_y = pose->pose.position.y;
  uav_z = pose->pose.position.z;
  uav_qx = pose->pose.orientation.x;
  uav_qy = pose->pose.orientation.y;
  uav_qz = pose->pose.orientation.z;
  uav_qw = pose->pose.orientation.w;

}

inline void depth_caculate(const yoloObject_t & object, cv::Mat frame, string & name)
{


    Quaterniond q(uav_qw,uav_qx,uav_qy,uav_qz);
    Mat3x3 mat = Q2R(q);
    cout<<"it is " << name <<endl;
    YAML::Node config = YAML::LoadFile(object_save_file_path);
    auto obj_size = config[name]["size"].as <double>();
//    cout << "load parameter size: "<< obj_size << endl;

    vector <double> sum_env_dep, sum_obj_dep; // vector to store environment depth and object depth
    vector <double> sum_u, sum_v, sum_w;  // vector to store world frame coordinate
    vector <double> sum_i, sum_j, sum_k;

    for (auto j : object.env_point)
    {
        double env_dep = 0.001 * frame.at<ushort>(j);
        if (env_dep > 0.5 && env_dep < 10.0)
        {
            sum_env_dep.push_back(env_dep);
        }

    }
    for (auto j : object.depth_point)
    {
        double sur_dep = 0.001 * frame.at<ushort>(j);
        auto obj_dep = sur_dep + obj_size;

        if (sur_dep > 0.5 && sur_dep < 8.0)
        {
            sum_obj_dep.push_back(obj_dep);
        }

        if (obj_dep > 0.5 && obj_dep < 10.0 )
        {
             double z = obj_dep; // the distance between center of the object is surface depth + object size
             double x = z * (j.x - cx)/fx;  //pixel coordinate u,v -> camera coordinate x,y
             double y = z * (j.y - cy)/fy;

             Vec4 camera_coord(x,y,z,1);
             Vec4 body_coord,world_coord; //(i,j,k,1) (u,v,w,1)
//             Vec4 offset(0.13,0,0,0);

             Vec4 offset(0.13, 0.2, 0, 0);
             Mat4x4 matrix_camera_to_body;
             Mat4x4 matrix_body_to_world;
             matrix_camera_to_body << 0, 0, 1, 0,
                                      -1, 0, 0, 0,
                                       0, -1, 0, 0,
                                       0, 0, 0, 1;

             matrix_body_to_world << mat(0,0), mat(0,1), mat(0,2), uav_x,
                                     mat(1,0), mat(1,1), mat(1,2), uav_y,
                                     mat(2,0), mat(2,1), mat(2,2), uav_z,
                                     0, 0, 0, 1;
             body_coord = matrix_camera_to_body * camera_coord + offset; //(i,j,k,1) from camera coordinate to body coordinate
             world_coord = matrix_body_to_world * body_coord; //(u,v,w,1) from body coordinate to world coordinate

             sum_i.push_back(body_coord(0));
             sum_j.push_back(body_coord(1));
             sum_k.push_back(body_coord(2));
             sum_u.push_back(world_coord(0));
             sum_v.push_back(world_coord(1));
             sum_w.push_back(world_coord(2));
         }

    }

    IIRfilter(IIR_worldframe(0,getIndex(object_class,name)),average(sum_u));
    IIRfilter(IIR_worldframe(1,getIndex(object_class,name)),average(sum_v));
    IIRfilter(IIR_worldframe(2,getIndex(object_class,name)),average(sum_w));

    cout << IIR_worldframe << endl;

    cout << endl;

    obj_pose.header.frame_id = "world";
    obj_pose.pose.position.x = IIR_worldframe(0,getIndex(object_class,name));
    obj_pose.pose.position.y = IIR_worldframe(1,getIndex(object_class,name));
    obj_pose.pose.position.z = IIR_worldframe(2,getIndex(object_class,name));
    obj_pose.pose.orientation.w = 1.0;
    obj_pose.pose.orientation.x = 0.0;
    obj_pose.pose.orientation.y = 0.0;
    obj_pose.pose.orientation.z = 0.0;

    sort(sum_env_dep.begin(),sum_env_dep.end());
    sort(sum_obj_dep.begin(),sum_obj_dep.end());

    double avg_env_dep = average(sum_env_dep);
    double avg_obj_dep = average(sum_obj_dep);


        cout << "avg_env_dep: "<< avg_env_dep << endl;
        cout << "avg_obj_dep: "<< avg_obj_dep << endl;
        cout << "radius: " << radius << endl;
        cout <<"obj_size + uav_size: " << obj_size + uav_size << endl;
        cout <<"avg_env_dep - avg_obj_dep: "<<avg_env_dep - avg_obj_dep << endl;

    //if(avg_env_dep - 2 * (0.5 * avg_obj_dep + obj_size + uav_size)  >= 0.0)
    if (radius > avg_obj_dep)
    {
        cout << "UAV is too close to object " << endl;
        
    }
    else if (radius > obj_size + uav_size && radius < avg_env_dep - avg_obj_dep)
    {

        obstacle = false;
        cout << " pass " <<endl;
    }
    else
    {

        obstacle = true;

        cout << " collision " <<endl;
    }

    config[name]["X_w"] = obj_pose.pose.position.x;
    config[name]["Y_w"] = obj_pose.pose.position.y;
    config[name]["Z_w"] = obj_pose.pose.position.z;
    config[name]["obstacle"] = obstacle;
    Object_array.object_array.at(getIndex(object_class,name)).class_name = name;
    Object_array.object_array.at(getIndex(object_class,name)).X_w = IIR_worldframe(0,getIndex(object_class,name));
    Object_array.object_array.at(getIndex(object_class,name)).Y_w = IIR_worldframe(1,getIndex(object_class,name));
    Object_array.object_array.at(getIndex(object_class,name)).Z_w = IIR_worldframe(2,getIndex(object_class,name));
    Object_array.object_array.at(getIndex(object_class,name)).has_obstacle = obstacle;

    ofstream fout(object_save_file_path);
    fout << config;
    fout.close();




//        cout << "size1 "<< sum_u.size() << " "<< sum_v.size() << " "<< sum_w.size() << " "<< endl;
//        cout << "size2 "<< sum_env_dep.size() << " "<< sum_obj_dep.size() << " " << endl;
//        cout << "env_dep array: "<< endl;
//        for(const auto &j : sum_env_dep)
//        {

//            cout << j << " " ;

//        }

//        cout << endl << " ++++++++++++++++++++++++++++++++++++++++++++++++++ " << endl;

//        cout << "obj_dep array: "<< endl;
//        for(const auto &k : sum_obj_dep)
//        {
//            cout << k << " " ;


//        }

//        cout << endl << " !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << endl;

//        cout << "avg_env_dep: "<< avg_env_dep << endl;
//        cout << "obj_env_dep: "<< avg_obj_dep << endl;


//    }

}

void callback(const sensor_msgs::CompressedImageConstPtr & rgb, const sensor_msgs::ImageConstPtr & depth)
{
//  cout << "hi " << endl;
  cv::Mat image_rgb;
//  cv::Mat image_dep;
  try
  {
      image_rgb = cv::imdecode(cv::Mat(rgb->data),1);
//      image_dep = cv::imdecode(cv::Mat(depth->data),1);

  }
  catch(cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  //depth_alignment call back
  cv_bridge::CvImagePtr depth_ptr  = cv_bridge::toCvCopy(depth, depth->encoding);
  cv::Mat image_dep = depth_ptr->image;
  if(identify_flag == true)
  {
      //run the network
      yolo.runOnFrame(image_rgb);
      detected_flag = yolo.check_detected();

      //draw the boudingbox
      yolo.init_box();
      yolo.init_points();

    //  yolo.display_points();
//      yolo.drawBoudingBox(image_rgb);

    //  writer << image_rgb;

      if(!detected_flag)
      {

        cout<<"not detect object "<<endl;

      }

      else
      {
        if(uav_x == 0 || uav_y == 0 || uav_z == 0 ||uav_qx == 0 || uav_qy == 0 ||uav_qz == 0 ||uav_qw == 0)
        {
            cout<<"not receive msg from mavros !!!!"<<endl;
        }
        else
        {
            for(auto & object_i : yolo.objects)
            {
                for (auto name_i : object_class)
                {
                    if (name_i.compare(object_i.classId) == 0)
                    {
//                        cout << name_i << endl;
                        getIndex(object_class, name_i);
                        depth_caculate(object_i, image_dep, name_i);
                    }
                }
            }
        }
      }
  }
  else {
//      std::cout << "identify not begin " << endl;
  }

}
void vicon_pose_recall(const geometry_msgs::PoseStamped::ConstPtr & pose1, const geometry_msgs::PoseStamped::ConstPtr & pose2)
{
  point_list.points.clear();
  point_list.header.frame_id = "world";
  point_list.header.stamp = ros::Time::now();
  point_list.ns = "GT_points";
  point_list.id = 0;
  point_list.action = visualization_msgs::Marker::ADD;
  point_list.pose.orientation.w = 1.0;
  point_list.type = visualization_msgs::Marker::SPHERE_LIST;
  point_list.scale.x = point_list.scale.y = point_list.scale.z = 0.2;
//  std_msgs::ColorRGBA color;
//  color.r= 0.8;
//  color.g= 0.8;
//  color.b= 0.8;
//  color.a= 1;

  point_list.color.a = 1;
  point_list.color.r = 1;
  point_list.color.g = 1;
  point_list.color.b = 1;

  geometry_msgs::Point obj1, obj2, obj3;

  obj1.x = pose1->pose.position.x;
  obj1.y = pose1->pose.position.y;
  obj1.z = pose1->pose.position.z;
  obj2.x = pose2->pose.position.x;
  obj2.y = pose2->pose.position.y;
  obj2.z = pose2->pose.position.z;
  point_list.points.push_back(obj1);
//  point_list.colors.push_back(color);
  point_list.points.push_back(obj2);
//  point_list.colors.push_back(color);



}


int main(int argc, char **argv)
{
  cout<<"hello camera "<<endl;

  ros::init(argc, argv, "camera");
  ros::NodeHandle nh;
  ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);
  ros::Subscriber uavposlp_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, uav_lp_cb);
  ros::Publisher objects_pub = nh.advertise<nav::Objects>("/Objects_array", 1);


  ros::Publisher object_pub = nh.advertise<geometry_msgs::PoseStamped>("objectpose",1);
  ros::Subscriber identify_sub = nh.subscribe<nav::identify_command>("/identify", 1, identify_cb);

  ros::Publisher groundtruth_pub = nh.advertise <visualization_msgs::Marker>("gt_points",10);
  message_filters::Subscriber<CompressedImage> rgb_sub(nh, "/camera/color/image_raw/compressed", 1);
  message_filters::Subscriber<Image> dep_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> gt1_sub(nh, "/vrpn_client_node/traffic_light/pose", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> gt2_sub(nh, "/vrpn_client_node/bulb/pose", 1);

  typedef sync_policies::ApproximateTime<CompressedImage, Image> MySyncPolicy;
  typedef sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy2;
//  typedef sync_policies::ApproximateTime<CompressedImage, Image, geometry_msgs::PoseStamped> MySyncPolicy;

//  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, dep_sub, localposition_sub);
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, dep_sub);
  Synchronizer<MySyncPolicy2> sync2(MySyncPolicy2(10), gt1_sub, gt2_sub);

  sync.registerCallback(boost::bind(&callback, _1, _2));
  sync2.registerCallback(boost::bind(&vicon_pose_recall, _1, _2));
  ros::Rate loop_rate(20);

  while(ros::ok())
  {

    objects_pub.publish(Object_array);
    object_pub.publish(obj_pose);
    groundtruth_pub.publish(point_list);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


