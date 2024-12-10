# ROS1 Package for JAKA(Huaray) 2D Camera
- Written in Python
- Services for capturing image and setting exposure time
- Topic pulisher for raw Image and CamInfo

# Language
- en [English](README.md)
- zh_CN [简体中文](readme/README_zh_CN.md)

# Environment
- First of all, you need to install the MVViewer software provided by Huaray.
- You may have to install opencv in your python environment.
- Change all paths according to your own environment.
- The package was tested under Ubuntu 20.04 using ROS neotic, and seemed to work fine. 
- Do not forget to `source path_to_your_workspace/devel/setup.bash` after `catkin_make`, you may want to add this to your `.bashrc` file.

# Usage
- Start the node through .launch file:
```
$ roslaunch jk_cam_py jk_cam_all.launch
```
- To capture one frame and save the image (you can choose image format), run:
```
$ rosservice call /cam_hand/capture '' 'png'
```
- To set exposuretime, run: 
```
$ rosservice call /cam_hand/set_exposure_time 6000
```