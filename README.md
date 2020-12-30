# LAIS
## Learning-based Autonomous Inspection UAV System
LAIS is a learning-based automonous inspection system developed for quadrotor platform.
### Video

[![LAIS Demo](/others/1.gif)](https://www.youtube.com/watch?v=OKSm8_4rhzU)

### Requirements
* **Ubuntu 16.04 or 18.04**
* **ROS Kinetic or Melodic:** [ROS Install](http://wiki.ros.org/ROS/Installation)
* **OpenCV >= 4.4:** [OpenCV Linux Install](https://docs.opencv.org/4.4.0/d7/d9f/tutorial_linux_install.html)
* **Python 3.8** 
* **CUDA >= 10.0:** [CUDA Toolkit Archive](https://developer.nvidia.com/cuda-toolkit-archive) 
* **CUDNN >= 7.0:** [cuDNN Archive](https://developer.nvidia.com/rdp/cudnn-archive)

### Build instructions on Linux
1. clone repository into working space

```
cd ~/catkin_ws/src
git clone https://github.com/JazzyFeng/LAIS.git
```

2. Install 3rd Party library
``` 
cd ~/catkin_ws/src/LAIS/3rdPartLib/
./install3rdPartLib.sh
```
3. Compile 
```
cd ~/catkin_ws
catkin_make
```

### How to use (to detect objects in our experiment)
1. Download the pretrained weight file [Yolov4.weight](https://drive.google.com/file/d/1yJNK_knUa5nMmq-85mgNHjUg6-WzYIfj/view?usp=sharing) [Yolov4.cfg](https://drive.google.com/file/d/1PgXbc63EkwIB3KO_2TWK-IK50g81r7g-/view?usp=sharing) and [obj.names](https://drive.google.com/file/d/1oBB9okRyAmfumOJo8-RkzY6Cc_wxAKrH/view?usp=sharing)
2. Modify the argument in `~/catkin_ws/src/LAIS/src/camera.cpp` :
- cfg_path
- weight_path
- classid_path
3. Open a terminal and launch the file:
```
cd ~/catkin_ws
catkin_make
rosrun LAIS camera
```
4. To visualize the detected object, open a teminal and launch the file:
```
cd ~/catkin_ws
roslaunch rviz.launch
```

### How to use (to inspect objects in our experiment)
1. **Make sure your quadrotor and D435 camera is ready**
2. Open a teminal and launch the file:
```
cd ~/catkin_ws
roslaunch LAIS fj005.launch
```
## Maintainer 
Yurong Feng(Dept.ME,PolyU): [yurong.feng@connect.polyu.hk](yurong.feng@connect.polyu.hk)
