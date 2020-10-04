//
//  yoloNet.cpp
//  C++
//
//  Created by Lib on 2020/5/13.
//  Copyright © 2020 Lib. All rights reserved.
//

#include <stdio.h>
#include "yoloNet.h"
using namespace cv;
using namespace dnn;
const cv::Scalar colors[] = {
    {0, 255, 255},
    {255, 255, 0},
    {0, 255, 0},
    {255, 0, 0}
};
const auto NUM_COLORS = sizeof(colors)/sizeof(colors[0]);


yoloNet::yoloNet(const String modelpath, const String weightpath, const String class_path,const int width, const int height, const float confidence)
{
    this->modelpath = modelpath;
    this->weightpath = weightpath;
    this->width = width;
    this->height = height;
    this->confThreshold = confidence;
    this->nmsThreshold = 0.4;
    this->scale = 0.00392;
    this->swapRB = true;
    this->Crop = false;
    this->flag = 0;
    // load class names
    ifstream ifs(class_path.c_str());
    CV_Assert(ifs.is_open());
    string line;
    while (getline(ifs, line)) {
        this->classes.push_back(line);
    }
    for (auto i = classes.begin(); i != classes.end(); i++){
        cout<< "classes begin "<< * i << endl;
    }

    //load a model
    this->net = readNet(modelpath, weightpath);
    this->net.setPreferableBackend(DNN_BACKEND_DEFAULT);
    this->net.setPreferableTarget(DNN_TARGET_CPU);

    //get output names 输出层名字
    this->net_outputNames = net.getUnconnectedOutLayersNames();
    for (auto i = net_outputNames.begin(); i != net_outputNames.end(); i++){
        cout<< "net_outputNames  "<< * i << endl;
    }

}


yoloNet::~yoloNet(){

}

void yoloNet::runOnFrame(Mat& frame)
{

    if(this->flag == 0)
    {
      this->total_start = std::chrono::steady_clock::now();
      Mat blob;
      // create a 4D blob from a frame 前处理，把一帧做缩放，调换RB channel，重新调整大小，并存储到blob中

      blobFromImage(frame, blob, this->scale, Size(this->width,this->height), Scalar(0,0,0), this->swapRB);
      // set the network input 对网络设置输入处理后的blob

      net.setInput(blob);

      std::vector<Mat> netOuts;
      this->dnn_start = chrono::steady_clock::now();
      net.forward(netOuts, this->net_outputNames); //前向传播，计算net_outputNames中存储的层的输出,并存储在netOuts数组中
      this->dnn_end = chrono::steady_clock::now();

      //decide the classes for bounding boxes
      vector<float> confidences; // from 0 to 1
      vector<int> classIds;  //0, 1, 2, 3
      vector<Rect> boundingBoxes;  // Rect class

      for (auto & output : netOuts)
      {

          for(int j = 0; j<output.rows; j++)  //查找符合条件的bounding box
          {

              auto objectnessPrediction = output.at <float> (j, 4);

              if (objectnessPrediction >= this->confThreshold) // 目标置信度大于confThreshold
              {

                  Mat classPrediction = output.row(j).colRange(5, output.cols); // class prob part
                  Point maxPoint; // class number corresponding to maximum class probaility
                  double maxVal; //maximum class probability
                  minMaxLoc(classPrediction, 0, &maxVal, 0, &maxPoint); //find the maximum class probability
                  auto x = output.at <float> (j, 0) * frame.cols;
                  auto y = output.at <float> (j, 1) * frame.rows;
                  auto w = output.at <float> (j, 2) * frame.cols;
                  auto h = output.at <float> (j, 3) * frame.rows;

                  confidences.push_back(maxVal); // probability maybe not larger than zero
                  classIds.push_back(maxPoint.x); // class number in obj.names
                  boundingBoxes.push_back(Rect(x, y, w, h));

              }
          }

      }


      // remove the bounding boxes indicate the same object using NMS
      std::vector<int> indices;
      NMSBoxes(boundingBoxes, confidences, this->confThreshold, this->nmsThreshold, indices);

      // save bounding boxes
      this->objects.resize(indices.size());
      for (int i = 0; i < indices.size(); i++)
      {
//          cout<<"indices size "<< indices.size()<<endl;
//          cout<< "i "<< i <<endl;
          int idx = indices[i];
/*          cout<<"idx: "<< idx<<endl;
          cout << "classIds[idx] " << classIds[idx] <<endl;*/ // class[idx] should range from 0 to number of class - 1 eg. 0,1,2,3
          CV_Assert(classIds[idx] < this->classes.size());// check the classIds[idx] == 0，0，3，0 < classes.size == 4
          yoloObject_t object = {
              .boundingBox = boundingBoxes[idx],
              .classId = this->classes[classIds[idx]],
              .confidence = confidences[idx]
          };
          this->objects[i] = object;
      }

    }


}



void yoloNet::init_box()
{
  double scale_in = 0.33, scale_out = 1.2;
  int scale_in_width, scale_in_height;
  int scale_out_width, scale_out_height;

  for (int i = 0; i < this->objects.size(); i++)
  {
      int x = objects[i].boundingBox.x - objects[i].boundingBox.width/2;
      int y = objects[i].boundingBox.y - objects[i].boundingBox.height/2;

      scale_in_width = scale_in * objects[i].boundingBox.width;
      scale_in_height = scale_in * objects[i].boundingBox.height;
      scale_out_width = scale_out * objects[i].boundingBox.width;
      scale_out_height = scale_out * objects[i].boundingBox.height;

      int x_in = objects[i].boundingBox.x - scale_in_width/2;
      int y_in = objects[i].boundingBox.y - scale_in_height/2;
      int x_out = objects[i].boundingBox.x - scale_out_width/2;
      int y_out = objects[i].boundingBox.y - scale_out_height/2;

      objects[i].draw_box = Rect(x, y, objects[i].boundingBox.width, objects[i].boundingBox.height);
      objects[i].depth_box= Rect(x_in, y_in, scale_in_width, scale_in_height);
      objects[i].env_box.out = Rect(x_out, y_out, scale_out_width, scale_out_height);
      objects[i].env_box.top = Rect(x_out, y_out, scale_out_width, (scale_out_height - objects[i].draw_box.height)/2 );
      objects[i].env_box.bottom = objects[i].env_box.top + Point(0, scale_out_height - objects[i].env_box.top.height);
      objects[i].env_box.left = Rect(x_out, y_out, (scale_out_width - objects[i].draw_box.width)/2, scale_out_height);
      objects[i].env_box.right = objects[i].env_box.left + Point(scale_out_width - objects[i].env_box.left.width, 0);
  }
}

void yoloNet::drawBoudingBox(Mat &img)
{

  if(this->flag == 0)
  {
    for (int i = 0; i < this->objects.size(); i++)
    {
        this->inference_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(dnn_end - dnn_start).count();
        const auto color = colors[i % NUM_COLORS];

        rectangle(img, objects[i].draw_box, color, 3);
        rectangle(img, objects[i].depth_box, color, 2);
        rectangle(img, objects[i].env_box.out, color, 1);
        rectangle(img, objects[i].env_box.top, Scalar(198,227,171), -1);
        rectangle(img, objects[i].env_box.bottom, Scalar(198,227,171), -1);
        rectangle(img, objects[i].env_box.left, Scalar(198,227,171), -1);
        rectangle(img, objects[i].env_box.right, Scalar(198,227,171), -1);
        //Create the label Text
        String labelText = format("%.2f", objects[i].confidence);
        labelText = objects[i].classId + ":" + labelText;
        //Draw the label text on the image
        int baseline;
        Size labelSize = getTextSize(labelText, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        objects[i].draw_box.y = max(objects[i].draw_box.y , labelSize.height);
        rectangle(img, Point(objects[i].draw_box.x, objects[i].draw_box.y  - labelSize.height) , Point (objects[i].draw_box.x + labelSize.width, objects[i].draw_box.y  + baseline), color, FILLED);
        putText(img, labelText, objects[i].draw_box.tl(), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,0));
    }

    this->total_end = std::chrono::steady_clock::now();
    float inference_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(dnn_end - dnn_start).count();
    float total_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start).count();
    std::ostringstream stats_ss;
    stats_ss << std::fixed << std::setprecision(2);
    stats_ss << "Inference FPS: " << inference_fps << ", Total FPS: " << total_fps;
    auto stats = stats_ss.str();
    int baseline;
    auto stats_bg_sz = cv::getTextSize(stats.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
    cv::rectangle(img, cv::Point(0, 0), cv::Point(stats_bg_sz.width, stats_bg_sz.height + 10), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(img, stats.c_str(), cv::Point(0, stats_bg_sz.height + 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255));
    imshow("Output of prediction", img);
    waitKey(1);
  }
}

bool yoloNet::check_detected()
{
  if(this->flag == 0)
  {
    if(this->objects.size() == 0) // no object detected
    {
      return false;
    }
    else
    {
      return true;
    }
  }
}

void yoloNet::init_points(){

  for (int i = 0; i < this->objects.size(); i++)
  {
//      dep_points.clear();
//      env_points.clear();
      objects[i].depth_point.clear();
      objects[i].env_point.clear();
//      cout << "Depth_box: "<< "top left: " << objects[i].depth_box.tl() <<
//              ", bottom right: " << objects[i].depth_box.br()<< endl;
//      cout << "Env_box out: " << objects[i].env_box.out<<endl;
//  //    cout << "Env_box top: " << objects[i].env_box.top << endl;
//      cout << "Env_box top left: "<< "top left: " << objects[i].env_box.out.tl() <<
//              ", bottom right: " << objects[i].env_box.out.br()<< endl;

    for (int j = 0; j < 40 ; j++)
    {
        for (int k = 0; k < 40 ; k++)
        {
          objects[i].depth_point.push_back(Point(objects[i].depth_box.x + j*objects[i].depth_box.width/40, objects[i].depth_box.y + k * objects[i].depth_box.height/40));
          objects[i].env_point.push_back(Point(objects[i].env_box.top.x + j*objects[i].env_box.top.width/40, objects[i].env_box.top.y + k * objects[i].env_box.top.height/40));
          objects[i].env_point.push_back(Point(objects[i].env_box.bottom.x + j*objects[i].env_box.bottom.width/40, objects[i].env_box.bottom.y + k * objects[i].env_box.bottom.height/40));
          objects[i].env_point.push_back(Point(objects[i].env_box.left.x + j*objects[i].env_box.left.width/40, objects[i].env_box.left.y + k * objects[i].env_box.left.height/40));
          objects[i].env_point.push_back(Point(objects[i].env_box.right.x + j*objects[i].env_box.right.width/40, objects[i].env_box.right.y + k * objects[i].env_box.right.height/40));
        }
     }


   }
}


void yoloNet::display_points()
{
    for (int i = 0; i < this->objects.size(); i++)
    {
    cout << "display_points function recall "<<endl;
    cout<< objects[i].classId << endl;
    cout <<"depth points size :"<<objects[i].depth_point.size()<<endl;
    cout<< "env points size :"<<objects[i].env_point.size()<<endl;
//    cout<<"env points :" << objects[i].env_point <<endl;
    cout<<"depth points: "<< objects[i].depth_point << " " << endl;

    }

}
std::vector<yoloObject_t> yoloNet::get_objects()
{
   return this->objects;
}



