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
#include "utils/yoloNet.h"
#include "utils/object.h"
#include "LAIS/Objects.h"
#include "LAIS/identify_command.h"
#include <algorithm>
#define PI (3.1415926)
using namespace std;

static string video_save_path = "/home/fengyurong/out.avi";
static string testconfig_path = "/home/panda/fyrws/src/LAIS/config/testconfig.yaml";
static std::vector <string> object_class = {"bulb", "traffic light", "cctv"};

static mavros_msgs::State current_state; //subscribe FCU state through State msg to check arm and offboard
//static cv::VideoWriter writer(video_save_path, cv::VideoWriter::fourcc('M', 'P', 'E', 'G'), 1, cv::Size(1280,720));

static bool flag = false;
static LAIS::identify_command identify_flag;

void rgb_image_cb(const sensor_msgs::ImageConstPtr& msg)
{

  cv::Mat image_rgb;
  cv_bridge::CvImagePtr a;
  a = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
  image_rgb = a->image;

  //  string file_path = "/home/fengyurong/pic/rgb/";
  //  static int count = 0;

  if(flag)
  {
    //writer << image_rgb;   //write video
    //    string save_path = file_path+to_string(count)+".jpg";
    //    //cout<<"save path "<< save_path<<endl;
    //    cv::imwrite(save_path,image_rgb);
    //    count++;

  }

}

// uav local parameter
static double uav_x, uav_y, uav_z;
static double uav_qx, uav_qy, uav_qz, uav_qw;
static double x_max, y_max, x_min, y_min; //geofence
static double traffic_x, traffic_y, traffic_z, bulb_x, bulb_y, bulb_z, cctv_x, cctv_y, cctv_z;
static bool traffic_obs, bulb_obs, cctv_obs;

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

void state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}

void objects_cb(const LAIS::Objects::ConstPtr & Objects){

  traffic_x = Objects->object_array.at(getIndex(object_class,"traffic light")).X_w;
  traffic_y = Objects->object_array.at(getIndex(object_class,"traffic light")).Y_w;
  traffic_z = Objects->object_array.at(getIndex(object_class,"traffic light")).Z_w;
  traffic_obs =  Objects->object_array.at(getIndex(object_class,"traffic light")).has_obstacle;
  bulb_x = Objects->object_array.at(getIndex(object_class,"bulb")).X_w;
  bulb_y = Objects->object_array.at(getIndex(object_class,"bulb")).Y_w;
  bulb_z = Objects->object_array.at(getIndex(object_class,"bulb")).Z_w;
  bulb_obs = Objects->object_array.at(getIndex(object_class,"bulb")).has_obstacle;
  cctv_x = Objects->object_array.at(getIndex(object_class,"cctv")).X_w;
  cctv_y = Objects->object_array.at(getIndex(object_class,"cctv")).Y_w;
  cctv_z = Objects->object_array.at(getIndex(object_class,"cctv")).Z_w;
  cctv_obs = Objects->object_array.at(getIndex(object_class,"cctv")).has_obstacle;


}
static double o_x, o_y, o_z, t1_x, t1_y, t1_z, t2_x, t2_y, t2_z;
static double tmp_x,tmp_y,tmp_z,tmp_yaw;
static double turn_dur= 10, move_dur = 10;

enum stage{
  IDLE,
  TAKEOFF1,
  TAKEOFF2,
  HOVER1,
  INSPECTION,
  IDENTIFY1,
  IDENTIFY2,
  IDENTIFY3,
  TURN1,
  CHECK1,
  TURN2,
  CHECK2,
  TURN3,
  CHECK3,
  HOVER2,
  MOVE,
  HOVER3,
  RETURN,
  LANDING,
  END
}static mission_stage=IDLE;

static bool force_start;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fj005");
  ros::NodeHandle nh("~");
  string config_file_path;
  cout << "get parameter: "<<endl;

  if(nh.getParam("basic_config",config_file_path))
  {
    cout<<"config path: "<< config_file_path<<endl;
  }
  else {
    cout<<"cannot load basic_config "<<endl;
  }

  nh.getParam("force_start",force_start);

  x_max = getDoubleVariableFromYamlhierarchy(config_file_path, "FJ005_geo_mode", "x_max");
  x_min = getDoubleVariableFromYamlhierarchy(config_file_path, "FJ005_geo_mode", "x_min");
  y_max = getDoubleVariableFromYamlhierarchy(config_file_path, "FJ005_geo_mode", "y_max");
  y_min = getDoubleVariableFromYamlhierarchy(config_file_path, "FJ005_geo_mode", "y_min");
  cout<<"FJ005_geofence: "<< endl;
  cout<<"x_max: "<<x_max<<endl;
  cout<<"x_min: "<<x_min<<endl;
  cout<<"y_max: "<<y_max<<endl;
  cout<<"y_min: "<<y_min<<endl;

  o_x = getDoubleVariableFromYaml(config_file_path, "original_x");
  o_y = getDoubleVariableFromYaml(config_file_path, "original_y");
  o_z = getDoubleVariableFromYaml(config_file_path, "original_z");
  cout<<"original_x: "<<o_x<<endl;
  cout<<"original_y: "<<o_y<<endl;
  cout<<"original_z: "<<o_z<<endl;
  t1_x = getDoubleVariableFromYaml(config_file_path, "take_off1_x");
  t1_y = getDoubleVariableFromYaml(config_file_path, "take_off1_y");
  t1_z = getDoubleVariableFromYaml(config_file_path, "take_off1_z");
  cout<<"take_off1_x: "<<t1_x<<endl;
  cout<<"take_off1_y: "<<t1_y<<endl;
  cout<<"take_off1_z: "<<t1_z<<endl;
  t2_x = getDoubleVariableFromYaml(config_file_path, "take_off2_x");
  t2_y = getDoubleVariableFromYaml(config_file_path, "take_off2_y");
  t2_z = getDoubleVariableFromYaml(config_file_path, "take_off2_z");
  cout<<"take_off2_x: "<<t2_x<<endl;
  cout<<"take_off2_y: "<<t2_y<<endl;
  cout<<"take_off2_z: "<<t2_z<<endl;



  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  ros::Subscriber uavposlp_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, uav_lp_cb);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  ros::Publisher identify_pub = nh.advertise<LAIS::identify_command>("/identify", 1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
  ros::Publisher approach_pub = nh.advertise<visualization_msgs::Marker>("/approach_marker", 10);
  ros::Subscriber objects_sub = nh.subscribe <LAIS::Objects>("/Objects_array", 1, objects_cb);

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  if(force_start)
  {
    cout<<"force start 1"<<endl;

  }
  else
  {

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
      cout<<" waiting for FCU connection "<<endl;
      ros::spinOnce();
      rate.sleep();
    }

  }


  geometry_msgs::PoseStamped pose;
  pose.header.frame_id="map";
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0;
  pose.pose.orientation.w = 1;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = 0;


  if(force_start)
  {
    cout<<"force start 2"<<endl;
  }
  else
  {
    //send a few setpoints before starting
    cout<<"send a few setpoints before starting "<<endl;
    for(int i = 100; ros::ok() && i > 0; --i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  cout<<"change time to now"<<endl;

  ros::Time last_request = ros::Time::now();
  while(ros::ok())
  {

    if(force_start)
    {
      mission_stage = TAKEOFF1;
      cout<<"mission stage : TAKEOFF1 "<<endl;
      last_request = ros::Time::now();

    }
    else
    {
      if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
      {
        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
          ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
      }
      else
      {
        if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(1.0)))
        {
          if( arming_client.call(arm_cmd) && arm_cmd.response.success)
          {
            ROS_INFO("Vehicle armed");
            mission_stage = TAKEOFF1;
            cout<<"mission stage : TAKEOFF 1---------------------- "<<endl;
          }
          last_request = ros::Time::now();
        }
      }
    }


    static std::vector <object> objects;
    object traffic_light(traffic_x, traffic_y, traffic_z, traffic_obs, "traffic light");
    object bulb(bulb_x, bulb_y, bulb_z, bulb_obs, "bulb");
    object cctv(cctv_x, cctv_y, cctv_z, cctv_obs, "cctv");


    Vec3 t2(t2_x,t2_y,t2_z);
    Vec3 uavpos(uav_x,uav_y,uav_z);

    objects.push_back(traffic_light);
    objects.push_back(bulb);
    objects.push_back(cctv);
    insertSort(objects, t2);
    double o0_x = objects[0].worldframe[0], o0_y = objects[0].worldframe[1], o0_z = objects[0].worldframe[2];
    double o1_x = objects[1].worldframe[0], o1_y = objects[1].worldframe[1], o1_z = objects[1].worldframe[2];
    double o2_x = objects[2].worldframe[0], o2_y = objects[2].worldframe[1], o2_z = objects[2].worldframe[2];
    //        double o0_x = 1, o0_y = 1, o0_z = 1;
    //        double o1_x = 0, o1_y = 2, o1_z = 1 ;
    //        double o2_x = 1, o2_y = -1, o2_z = 1.5;

    string o0_name = objects[0].name, o1_name = objects[1].name, o2_name = objects[2].name;
    vector <double>  approach_0 = approach_point(uav_x, uav_y, o0_x, o0_y, 1);
    vector <double>  approach_1 = approach_point(uav_x, uav_y, o1_x, o1_y, 1);
    vector <double>  approach_2 = approach_point(uav_x, uav_y, o2_x, o2_y, 1);

    double a0_x = approach_0[0], a0_y = approach_0[1], a0_z = objects[0].worldframe[2];
    double a1_x = approach_1[0], a1_y = approach_1[1], a1_z = objects[1].worldframe[2];
    double a2_x = approach_2[0], a2_y = approach_2[1], a2_z = objects[2].worldframe[2];
    //        cout << "object 1: " << o0_x << " "<< o0_y << " " << o0_z << endl;
    //        cout << "object 2: " << o1_x << " "<< o1_y << " " << o1_z << endl;
    //        cout << "object 3: " << o2_x << " "<< o2_y << " " << o2_z << endl;

    if(mission_stage == TAKEOFF1)
    {
      identify_flag.identify = true;
      static GeneralMove takeoff1(ros::Time::now().toSec(),o_x,o_y,o_z-0.2,0,t1_x,t1_y,t1_z,0,6);
      takeoff1.getPose(ros::Time::now().toSec(),pose);
      if(takeoff1.finished())
      {
        mission_stage = TAKEOFF2;
        cout<<"mission stage : TAKEOFF 2---------------------- "<<endl;
        last_request = ros::Time::now();
      }
    }

    if(mission_stage == TAKEOFF2)
    {
      identify_flag.identify = true;
      static GeneralMove takeoff2(ros::Time::now().toSec(),t1_x,t1_y,t1_z,0,t2_x,t2_y,t2_z,0,6);
      takeoff2.getPose(ros::Time::now().toSec(),pose);
      if(takeoff2.finished())
      {
        mission_stage = HOVER1;
        cout<<"mission stage : HOVER 1 --------------------------"<<endl;
        last_request = ros::Time::now();
      }
    }

    if(mission_stage == HOVER1)
    {
      identify_flag.identify = true;
      if(ros::Time::now() - last_request > ros::Duration(6.0))
      {
        mission_stage = IDENTIFY1;
        cout<<"mission stage : IDENTIFY 1 --------------------------"<<endl;
        last_request = ros::Time::now();
      }
    }

    if(mission_stage == IDENTIFY1)
    {
      identify_flag.identify = true;
      static GeneralMove counterclockwise(ros::Time::now().toSec(),t2_x,t2_y,t2_z,0,t2_x,t2_y,t2_z,PI/2,10);
      counterclockwise.getPose(ros::Time::now().toSec(),pose);
      if(counterclockwise.finished())
      {
        mission_stage = HOVER2;
        cout<<"mission stage : IDENTIFY 2 --------------------------"<<endl;
        last_request = ros::Time::now();
      }

    }
    if(mission_stage == HOVER2)
    {
      identify_flag.identify = true;
      if(ros::Time::now() - last_request > ros::Duration(2.0)){
        mission_stage = IDENTIFY2;
      }
    }

    if(mission_stage == IDENTIFY2) //this stage must ensure the model can detect the correct object
    {
      identify_flag.identify = true;
      static GeneralMove clockwise(ros::Time::now().toSec(),t2_x,t2_y,t2_z,PI/2,t2_x,t2_y,t2_z,-PI/2,10);
      clockwise.getPose(ros::Time::now().toSec(),pose);

      if(clockwise.finished())
      {
        mission_stage = IDENTIFY3;
        cout<<"mission stage : IDENTIFY 3 --------------------------"<<endl;
        last_request = ros::Time::now();
      }

    }
    if(mission_stage == IDENTIFY3) //this stage must ensure the model can detect the correct object
    {
      identify_flag.identify = true;
      static GeneralMove clockwise(ros::Time::now().toSec(),t2_x,t2_y,t2_z,-PI/2,t2_x,t2_y,t2_z,0,10);
      clockwise.getPose(ros::Time::now().toSec(),pose);

      if(clockwise.finished())
      {
        clockwise.getEnding(tmp_x, tmp_y, tmp_z, tmp_yaw);
        mission_stage = INSPECTION;
        cout<<"mission stage : INSPECTION --------------------------"<<endl;
        last_request = ros::Time::now();
      }

    }

    if(mission_stage == INSPECTION)
    {
      identify_flag.identify = true;

      if(ros::Time::now() - last_request > ros::Duration(5.0))
      {
        cout << endl;
        cout << "objects sorted: " << endl;
        for(int i = 0; i < objects.size(); i++)
        {
          cout << i << endl;
          objects[i].display();
          cout << endl;
        }
        cout << endl;
        mission_stage = TURN1;

        cout<<"mission stage :  TURN 1  --------------------------"<<endl;
        // mission_stage = HOVER3;
        // cout<<"mission stage :  HOVER 3  --------------------------"<<endl;

        last_request = ros::Time::now();
      }
    }
    // turn
    if(mission_stage == TURN1)
    {
      identify_flag.identify = true;

      static GeneralMove TURN1(ros::Time::now().toSec(),t2_x,t2_y,t2_z,0,t2_x,t2_y,t2_z,atan2(o0_y-t2_y,o0_x-t2_x),turn_dur);
      TURN1.getPose(ros::Time::now().toSec(),pose);

      if(TURN1.finished())
      {
        static ros::Time local_time = ros::Time::now();
        if(ros::Time::now() - local_time > ros::Duration(6.0))
        {
          TURN1.getEnding(tmp_x, tmp_y, tmp_z, tmp_yaw);
          mission_stage = CHECK1;
          cout<<"mission stage : CHECK 1 move to object 1 --------------------------"<<endl;
          last_request = ros::Time::now();
        }

      }
    }
    // move
    if(mission_stage == CHECK1)
    {
      identify_flag.identify = true;

      static GeneralMove move1(ros::Time::now().toSec(),tmp_x,tmp_y,tmp_z,tmp_yaw, a0_x, a0_y, a0_z, atan2(o0_y-tmp_y,o0_x-tmp_x),10);
      move1.getPose(ros::Time::now().toSec(),pose);
      if(move1.finished())
      {
        identify_flag.identify = false;
        move1.getEnding(tmp_x, tmp_y, tmp_z, tmp_yaw);
        static ros::Time local_time = ros::Time::now();
        const static bool obs = objects[0].check_obstacle();
        cout << "object 1 obstacle : " << obs << endl;

        if (obs == false)
        {

          const static double o0_x_const = o0_x, o0_y_const = o0_y, o0_z_const = o0_z;
          static circleTrj CHECK1(ros::Time::now().toSec(), tmp_x, tmp_y, tmp_z, o0_x_const, o0_y_const, o0_z_const, 2*PI,15,1);
          CHECK1.getPose(ros::Time::now().toSec(),pose);
          //               cout << "no obstacle check object 1 " << endl;
          if(CHECK1.finished())
          {
            CHECK1.getEnding(tmp_x, tmp_y, tmp_z, tmp_yaw);
            mission_stage = TURN2;
            cout<< "mission stage : TURN 2 --------------------------" << endl;
            last_request = ros::Time::now();
          }
        }
        else
        {
          cout << "object 1 has obstacle " << endl;
          if(ros::Time::now() - local_time > ros::Duration(6.0))
          {
            move1.getEnding(tmp_x, tmp_y, tmp_z, tmp_yaw);
            mission_stage = TURN2;
            cout<<"mission stage : TURN 2 --------------------------" << endl;
            last_request = ros::Time::now();
          }

        }

      }
    }

    if(mission_stage == TURN2)
    {
      identify_flag.identify = true;
      static GeneralMove TURN2(ros::Time::now().toSec(),tmp_x, tmp_y, tmp_z, tmp_yaw, tmp_x, tmp_y, tmp_z, atan2(o1_y-tmp_y,o1_x-tmp_x),turn_dur);
      TURN2.getPose(ros::Time::now().toSec(),pose);
      if(TURN2.finished())
      {
        static ros::Time local_time = ros::Time::now();
        if(ros::Time::now() - local_time > ros::Duration(6.0))
        {
          TURN2.getEnding(tmp_x, tmp_y, tmp_z, tmp_yaw);
          mission_stage = CHECK2;
          cout<<"mission stage : CHECK 2 move to object 2 --------------------------"<<endl;
          last_request = ros::Time::now();
        }

      }
    }
    if(mission_stage == CHECK2)
    {
      identify_flag.identify = true;


      static GeneralMove move2(ros::Time::now().toSec(), tmp_x, tmp_y, tmp_z, tmp_yaw, a1_x, a1_y, a1_z, atan2(o1_y-tmp_y,o1_x-tmp_x), 10);
      move2.getPose(ros::Time::now().toSec(),pose);
      if(move2.finished())
      {
        identify_flag.identify = false;
        move2.getEnding(tmp_x, tmp_y, tmp_z, tmp_yaw);
        static ros::Time local_time = ros::Time::now();

        const static bool obs = objects[1].check_obstacle();
        cout << "object 2 obstacle : " << obs << endl;
        if (obs == false)
        {
          const static double o1_x_const = o1_x, o1_y_const = o1_y, o1_z_const = o1_z;
          static circleTrj CHECK2(ros::Time::now().toSec(),tmp_x, tmp_y, tmp_z, o1_x_const, o1_y_const, o1_z_const, 2*PI,15,1);
          CHECK2.getPose(ros::Time::now().toSec(),pose);
          if(CHECK2.finished())
          {
            CHECK2.getEnding(tmp_x, tmp_y, tmp_z, tmp_yaw);
            mission_stage = TURN3;
            //                        mission_stage = HOVER3;

            cout<<"mission stage : TURN 3 -------------------------"<<endl;
            last_request = ros::Time::now();
          }
        }
        else
        {
          cout << "object 2 has obstacle " <<endl;
          if(ros::Time::now() - local_time > ros::Duration(6.0))
          {
            move2.getEnding(tmp_x, tmp_y, tmp_z, tmp_yaw);
            mission_stage = TURN3;
            //                        mission_stage = HOVER3;

            cout<<"mission stage : TURN 3 --------------------------"<<endl;
            last_request = ros::Time::now();
          }

        }

      }
    }
    if(mission_stage == TURN3)
    {
      identify_flag.identify = true;
      static GeneralMove TURN3(ros::Time::now().toSec(),tmp_x, tmp_y, tmp_z, tmp_yaw, tmp_x, tmp_y, tmp_z, atan2(o2_y-tmp_y,o2_x-tmp_x),turn_dur);
      TURN3.getPose(ros::Time::now().toSec(),pose);
      if(TURN3.finished())
      {
        static ros::Time local_time = ros::Time::now();
        if(ros::Time::now() - local_time > ros::Duration(6.0))
        {
          TURN3.getEnding(tmp_x, tmp_y, tmp_z, tmp_yaw);
          mission_stage = CHECK3;
          cout<<"mission stage : CHECK 3 move to object 3 --------------------------"<<endl;
          last_request = ros::Time::now();
        }

      }
    }

    if(mission_stage == CHECK3)
    {
      identify_flag.identify = true;

      static GeneralMove move3(ros::Time::now().toSec(),tmp_x, tmp_y, tmp_z, tmp_yaw, a2_x, a2_y, a2_z, atan2(o2_y-tmp_y,o2_x-tmp_x), 10);
      move3.getPose(ros::Time::now().toSec(),pose);
      if(move3.finished())
      {
        identify_flag.identify = false;
        move3.getEnding(tmp_x, tmp_y, tmp_z,tmp_yaw);
        static ros::Time local_time = ros::Time::now();
        static const bool obs = objects[2].check_obstacle();
        cout << "object 3 obstacle : " << obs << endl;

        if (obs == false)
        {
          const static double o2_x_const = o2_x, o2_y_const = o2_y, o2_z_const = o2_z;
          static circleTrj CHECK3(ros::Time::now().toSec(), tmp_x, tmp_y, tmp_z, o2_x_const, o2_y_const, o2_z_const, 2*PI, 15, 1);
          CHECK3.getPose(ros::Time::now().toSec(),pose);
          if(CHECK3.finished())
          {
            CHECK3.getEnding(tmp_x, tmp_y, tmp_z, tmp_yaw);
            mission_stage = HOVER3;
            cout<<"mission stage : HOVER 3 --------------------------"<<endl;
            last_request = ros::Time::now();
          }
        }
        else
        {
          cout << "object 3 has obstacle " << endl;
          if(ros::Time::now() - local_time > ros::Duration(6.0))
          {
            move3.getEnding(tmp_x, tmp_y, tmp_z, tmp_yaw);
            mission_stage = HOVER3;
            cout<<"mission stage : HOVER 3 --------------------------"<<endl;
            last_request = ros::Time::now();
          }
        }
      }
    }

    if(mission_stage == HOVER3)
    {
      if(ros::Time::now() - last_request > ros::Duration(2.0))
      {
        mission_stage = RETURN;
        cout<<"HOVER3 finished ****************************"<<endl;
        cout<<"returning ****************************"<<endl;
        last_request = ros::Time::now();
      }
    }

    if(mission_stage == RETURN)
    {

      static GeneralMove ret(ros::Time::now().toSec(),tmp_x, tmp_y, tmp_z, tmp_yaw, t2_x, t2_y, t2_z, 0, 10);
      ret.getPose(ros::Time::now().toSec(),pose);
      if(ret.finished())
      {
        cout<<"Vehicle returned to takeoff point ****************************"<<endl;
        mission_stage = LANDING;
        last_request = ros::Time::now();
      }

    }

    if(mission_stage == LANDING)
    {

      double secs = (ros::Time::now()-last_request).toSec();
      cout<<"secs: "<<secs<<endl;
      pose.pose.position.z = t2_z - (secs)*0.1;
      cout<<"z: "<<pose.pose.position.z<<endl;
      if(pose.pose.position.z < -0.3)
      {
        cout<<"disarming **************************** "<<endl;
        pose.pose.position.z = -0.3;
        arm_cmd.request.value = false;
        if( arming_client.call(arm_cmd) &&
            arm_cmd.response.success)
        {
          ROS_INFO("Vehicle disarmed****************************");

          mission_stage = END;

          return 0;
        }
      }
    }

    if(mission_stage == END)
    {
      cout<<"enter END ****************************"<<endl;
      pose.pose.position.x = 0;
      pose.pose.position.y = 0;
      pose.pose.position.z = -0.3;
      return 0;

    }

    visualization_msgs::Marker points, line_strip, line_list;
    visualization_msgs::Marker objects_d, objects_cube;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = objects_d.header.frame_id = objects_cube.header.frame_id = "map";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = objects_d.header.stamp = objects_cube.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = objects_d.ns = objects_cube.ns ="points_and_lines";
    points.action = line_strip.action = line_list.action = objects_d.action = objects_cube.action= visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w =
        objects_d.pose.orientation.w = objects_cube.pose.orientation.w = 1.0;

    //points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;
    objects_d.id = 0;
    objects_cube.id = 3;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    objects_d.type = visualization_msgs::Marker::SPHERE_LIST;
    objects_cube.type = visualization_msgs::Marker::CUBE_LIST;
    points.scale.x = 0.1;
    points.scale.y = 0.1;

    objects_d.scale.x = objects_d.scale.y = objects_d.scale.z = 0.2 ;
    objects_cube.scale.x = objects_cube.scale.y = objects_cube.scale.z = 0.25;


    std_msgs::ColorRGBA color_safe, color_collision, color_cube;
    color_safe.r= 0;
    color_safe.g= 1;
    color_safe.b= 0;
    color_safe.a= 0.5;
    color_collision.r = 1;
    color_collision.g = 0;
    color_collision.b = 0;
    color_collision.a = 0.5;
    color_cube.r= 1;
    color_cube.g= 1;
    color_cube.b= 1;
    color_cube.a= 0.9;

//    geometry_msgs::Point obj1, obj2, obj3;
//    obj1.x = o0_x;
//    obj1.y = o0_y;
//    obj1.z = o0_z;
//    obj2.x = o1_x;
//    obj2.y = o1_y;
//    obj2.z = o1_z;
//    obj3.x = o2_x;
//    obj3.y = o2_y;
//    obj3.z = o2_z;
    for (auto object : objects)
    {
      if (object.check_obstacle() == false)
      {
        geometry_msgs::Point object_safe;
        object_safe.x = object.worldframe.x();
        object_safe.y = object.worldframe.y();
        object_safe.z = object.worldframe.z();

        objects_d.points.push_back(object_safe);
        objects_d.colors.push_back(color_safe);
        objects_cube.points.push_back(object_safe);
        objects_cube.colors.push_back(color_cube);

      }
      else
      {
        geometry_msgs::Point object_collision;
        object_collision.x = object.worldframe.x();
        object_collision.y = object.worldframe.y();
        object_collision.z = object.worldframe.z();

        objects_d.points.push_back(object_collision);
        objects_d.colors.push_back(color_collision);
        objects_cube.points.push_back(object_collision);
        objects_cube.colors.push_back(color_cube);
      }
    }

//    objects_d.points.push_back(obj1);
//    objects_d.colors.push_back(color_safe);
//    objects_d.points.push_back(obj2);
//    objects_d.colors.push_back(color_safe);
//    objects_d.points.push_back(obj3);
//    objects_d.colors.push_back(color_collision);

    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;

    points.color.g = 1.0f;
    points.color.a = 1.0;

    line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;

    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    geometry_msgs::Point p_0, p_1, p_2, p_3, p_4;
    p_0.x = p_3.x = x_min;
    p_0.y = p_1.y = y_min;
    p_1.x = p_2.x = x_max;
    p_2.y = p_3.y = y_max;
    p_0.z = p_1.z = p_2.z= p_3.z = 0;
    p_4 = p_0;

    // approach point position initialization
    geometry_msgs::Point a_0, a_1, a_2;
    a_0.x = a0_x;
    a_0.y = a0_y;
    a_0.z = a0_z;
    a_1.x = a1_x;
    a_1.y = a1_y;
    a_1.z = a1_z;
    a_2.x = a2_x;
    a_2.y = a2_y;
    a_2.z = a2_z;

    points.points.push_back(a_0);
    points.points.push_back(a_1);
    points.points.push_back(a_2);

    approach_pub.publish(points);


    line_strip.points.push_back(p_0);
    line_strip.points.push_back(p_1);
    line_strip.points.push_back(p_2);
    line_strip.points.push_back(p_3);
    line_strip.points.push_back(p_4);

    line_list.points.push_back(p_0);
    p_0.z += 1.0;
    line_list.points.push_back(p_0);
    line_list.points.push_back(p_1);
    p_1.z += 1.0;
    line_list.points.push_back(p_1);
    line_list.points.push_back(p_2);
    p_2.z += 1.0;
    line_list.points.push_back(p_2);
    line_list.points.push_back(p_3);
    p_3.z += 1.0;
    line_list.points.push_back(p_3);

    marker_pub.publish(line_strip);
    marker_pub.publish(line_list);
    marker_pub.publish(objects_d);
    marker_pub.publish(objects_cube);

    local_pos_pub.publish(pose);
    identify_pub.publish(identify_flag);
    objects.clear();


    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
