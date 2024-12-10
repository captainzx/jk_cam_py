# 节卡（华睿）2D 相机 ROS1 功能包
- 基于 Python 开发
- 支持单帧抓图&设置曝光时间服务
- 支持发布图像数据&CamInfo话题

# 语言
- en [English](README.md)
- zh_CN [简体中文](readme/README_zh_CN.md)

# 环境
- 首先，安装华睿提供的 MVViewer 软件。
- 需要在 python 环境中安装 opencv。
- 根据实际情况更改源文件中的路径。
- 功能包在 Ubuntu 20.04 系统下，使用 ROS neotic 进行了测试，看起来功能是正常的。
- 运行 `catkin_make` 后，记得 `source path_to_your_workspace/devel/setup.bash`，可以直接把这条 source 命令加到 `.bashrc` 文件里。

# 使用
- 通过 .launch 文件启动节点：
```
$ roslaunch jk_cam_py jk_cam_all.launch
```
- 抓取并保存单帧图像（可更改文件格式），运行：
```
$ rosservice call /cam_hand/capture '' 'png'
```
- 调整曝光时间，运行：
```
$ rosservice call /cam_hand/set_exposure_time 6000
```