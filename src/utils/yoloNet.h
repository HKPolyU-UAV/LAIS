//
//  yoloNet.h
//  C++
//
//  Created by Lib on 2020/5/13.
//  Copyright © 2020 Lib. All rights reserved.
//
#include <string>
#include <iostream>
#include <fstream>
#include <istream>
//opencv
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <iomanip>

using namespace std;
using namespace cv;
using namespace dnn;
typedef struct env_boxes{
    Rect out;
    Rect top, bottom, left, right;

}env_boxes;

typedef struct yoloObject_t{
    Rect boundingBox;
    String classId;
    float confidence;
    Rect depth_box;
    Rect draw_box;
    env_boxes env_box;
    std::vector<Point> depth_point, env_point;

}yoloObject_t;




class yoloNet{
private:
    String modelpath;
    String weightpath;
    int width, height;
    float confThreshold, nmsThreshold;
    float scale;
    bool swapRB;
    bool Crop;
    int flag;
    Net net;
    std::vector<String> classes;
    std::vector<String> net_outputNames;

    float inference_fps;
    float total_fps;
    chrono::time_point <chrono::steady_clock> total_start, total_end, dnn_start, dnn_end;

    
public:


    yoloNet(const String modelpath, const String weightpath, const String class_path,const int width = 608, const int height = 608, const float confidence = 0.5);
    ~yoloNet();

    void runOnFrame(Mat & frame);
    void drawBoudingBox(Mat & img);
    bool check_detected();
    void init_box();
    void init_points();
    void display_points();
    std::vector<yoloObject_t> get_objects();
    std::vector<yoloObject_t> objects;

};


