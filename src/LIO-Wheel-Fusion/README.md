# 激光惯性轮速计系统 lio_wheel
**Features:**
- 支持IMU+轮速计+LIDAR传感器输入，构建视觉惯性轮速紧耦合系统
    - 轮速计预积分
    - 在线时空标定（雷达和IMU外参、时延标定）
    - 基于PCA的雷达退化判断
    - 地面点滤波算法
    - 静止初始化
    - 基于imu+轮速计积分结果对激光点云去畸变
    - 自动识别Apriltag进行评估
- 地图管理
    - 基于ikd-tree
    - 地图点的动态增减、地图动态更新
    - 高效的点云查询与匹配

## 1. 安装依赖
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)
### 1.2 **Eigen**
install eigen：
```asm
git clone https://gitlab.com/libeigen/eigen.git
cd ./eigen
git checkout 3.3.9
mkdir build && cd build
cmake ..
sudo make install
```
### 1.3 **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).
Version: 1.4.0 (Ubuntu18.04)

### 1.4 **Sophus**
install Sophus (the latest code in master branch):
```asm
git clone https://github.com/strasdat/Sophus.git
cd ./Sophus/
mkdir build
cd ./build
cmake ..
make
sudo make install
```
### 1.5 **Opencv**
**Opencv 3.4.5 + opencv_contrib-3.4.5 + cuda**

Follow [Opencv Installation](https://blog.csdn.net/weixin_45117253/article/details/103985646).

### 1.6 **Glog**
install Glog:
```
sudo apt-get install libgoogle-glog-dev
```
## 2. 使用注意事项
  - 所有的默认参数配置均为针对示例（交大采集）数据集（见4.）设置
  - 若想使用宝时得含lidar的除草剂录制数据请修改配置文件
    1. 修改 include/global_definition/global_definition.h.in 中的 "sjtu_config.yaml"为"config.yaml"
    2. 修改 launch/mapping_zvision_wheel.launch 中的 "sjtu_config.yaml"为"config.yaml"
    3. 重新执行 catkin_make 编译代码
  - 相关参数配置在 config.yaml 和 sjtu_config.yaml 中均有说明
  - 如果想要用rviz可视化显示当前lio_wheel 建图和轨迹结果，请把launch/mapping_zvision_wheel.launch 第三行中的"false" -> "true"
  - 输出数据说明：雷达slam输出位姿话题 /LIO_odometry , 雷达slam定位结果将在多传感器融合节点进行统一评估
## 3. Build lio_wheel
复制此代码包，然后执行 catkin_make:
```
    cd ~/catkin_ws/src
    cp -r (lio_wheel代码包路径) ./
    cd ../
    catkin_make
```
## 4. 示例数据集

交大采集数据集
链接: https://pan.baidu.com/s/1GHFUzat0g7z4H0PbvIolYQ  
密码: ds05

## 5. Run
```
  cd ~/catkin_ws
  source devel/setup.bash
  roslaunch lio_wheel mapping_zvision_wheel.launch
  
  开启新的终端
  rosbag play YOUR_BAG.bag
```

