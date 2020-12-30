#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include "utils/yoloNet.h"

using namespace std;
// YOLO file path
static string cfg_path = "/home/sy/Downloads/yolov4_8_31.cfg";
static string weight_path = "/home/sy/Downloads/yolov4_8_31_best.weights";
static string classid_path = "/home/sy/Downloads/obj.names";
static string object_save_file_path = "/home/panda/fyrws/src/LAIS/config/testconfig.yaml";
static std::vector <string> object_class = {"bulb", "traffic light", "cctv"};

//Creat the YOLO network
static yoloNet yolo = yoloNet(cfg_path, weight_path, classid_path, 512, 512, 0.5);
string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
Ptr <Tracker> createTrackerByName(string trackerType)
{
  Ptr <Tracker> tracker;
  if (trackerType == trackerTypes[2]){
    tracker = cv::TrackerKCF::create();
  }
  return tracker;
}
void getRandomColors(vector <cv::Scalar> & colors, int numColors)
{
  cv::RNG rng(0);
  for(int i = 0; i < numColors; i++){
    colors.push_back(Scalar(rng.uniform(0,255), rng.uniform(0,255),rng.uniform(0,255)));
  }
}
enum state{
  Detection,
  Tracking,
  Tracking_Begin,
  Detection_Success,
  Detection_Failure,
  Tracking_Success,
  Tracking_Failure,

}static state = Detection;


int main(int argc, char **argv)
{

  static cv::Ptr <cv::MultiTracker> multitracker;

  cv::VideoCapture cap("/home/sy/fyrws/src/LAIS/result/2.avi");
  if(!cap.isOpened())
  {
    cout << "Could not open video file" << endl;
    return 1;
  }

  cv::Mat frame;
  vector <cv::Rect> bboxes;
  vector <cv::Scalar> colors;
  cap >> frame;

  while(1)
  {
    char key = cv::waitKey(1);
    if(key == 'd')
    {
      cap >> frame;
    }
    if(key == 'q')
    {
      break;
    }
    cv::imshow("first", frame);
  }
  cv::destroyWindow("first");

  while(cap.read(frame))
  {
    bboxes.clear();
    if(frame.empty())
    {
      break;
    }

    if (state == Detection || state == Tracking_Failure || state == Detection_Failure)
    {
      if(multitracker.empty())
      {
        cout << "reinit the tracker" << endl;
        multitracker = cv::MultiTracker::create();
      }
      yolo.runOnFrame(frame);
      yolo.init_box();
      //yolo.drawBoudingBox(frame);
      cout << "YOLO is running " << endl;
      bool detected_flag = yolo.check_detected();
      if (detected_flag == true)
      {
        state = Detection_Success;
      }
      else
      {
        state = Detection_Failure;
      }
    }

    if (state == Detection_Success)
    {
      cout << "State: Detection_success "<< endl;
      for (auto object : yolo.objects)
      {
        bboxes.push_back(object.draw_box);
      }

      getRandomColors(colors, bboxes.size());
      for(int i =0; i <bboxes.size(); i++)
      {
        multitracker->add(createTrackerByName("KCF"), frame, bboxes[i]);
      }
      state = Tracking_Success;
    }

    if (state == Detection_Failure)
    {
      cout << "State: Detection_Failure "<< endl;
    }

    if (state == Tracking_Success)
    {
      cout << "State: Tracking_success "<< endl;
      double timer = (double)cv::getTickCount();
      bool ok = multitracker->update(frame);
      float fps= cv::getTickFrequency() / ((double)cv::getTickCount() - timer);
      if (ok)
      {
        cout << "multitracker size " << multitracker->getObjects().size() << endl;
        for (unsigned i = 0; i<multitracker->getObjects().size(); i++)
        {
          cv::rectangle(frame, multitracker->getObjects()[i], colors[i], 2, 1);
        }
        cv::putText(frame, "FPS : " + to_string(float(fps)), cv::Point(20,20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0), 2);
      }
      else
      {
        state = Tracking_Failure;
      }

    }

    if(state == Tracking_Failure)
    {
      cout << "State: Tracking_Failure"<< endl;
      cv::putText(frame, "Tracking failure detected", cv::Point(20,20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
      multitracker.release();

    }

    cv::imshow("tracker", frame);
    cv::waitKey(30);
    if(cv::waitKey(1)==27)
    {
      cout << "exit a" << endl;
      break;
    }

  }

  return 0;
}
