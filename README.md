[README待完善]
# Pikachu战队2026 Vision Project

 本项目为福建师范大学Pikachu战队2026赛季视觉主项目框架。
 本仓库存放2026赛季**哨兵**算法代码，包括自瞄与导航部分。

## 一、项目结构

```
.
│
├── rm_bringup (启动及参数文件)
│
├── rm_robot_description (机器人urdf文件，坐标系的定义)
│
├── rm_interfaces (自定义msg、srv)
│
├── rm_hardware_driver
│   │
│   ├── rm_camera_driver (相机驱动)（大华新相机包）
│   │
│   ├── rm_hik_camera_driver (相机驱动)（新相机包）
│   │
│   └── rm_serial_driver (串口驱动)
│
├── rm_auto_aim (自瞄算法)
│   │
│   ├── armor_detector
│   │
│   └── armor_solver
│
├── rm_sentry_nav (导航包)
│
├── rm_utils (工具包) 
│
└── rm_upstart (自启动配置)
```

## 二、环境配置

### Ubuntu 22.04

### 相机驱动

#### 海康相机SDK
海康相机免驱，可不用安装

保证该包内包含海康SDK即可。

*注：新PC第一次连接相机使用前要下一遍海康MVK。原因未知。

#### 大华相机驱动（待补充）
!大华相机必须安装相机驱动!
*注：大华相机驱动安装过程麻烦，若明确不使用大华相机则可选择性安装。

内核版本：5.11.0及以下
编译器版本：gcc 11及以下

*若执行过鱼香肉丝ros安装，gcc版本已经为11，故只需要降低内核版本*

##### 降内核安装后升回
由于Ubuntu 22.04内核版本为6.5.0，需要导入Ubuntu20.04 focal仓库源下载5.11.0版内核

**参考链接**[https://blog.csdn.net/qq_62368277/article/details/134273919?ops_request_misc=&request_id=&biz_id=102&utm_term=ubuntu%E9%99%8D%E4%BD%8E%E4%BD%BF%E7%94%A8%E7%9A%84%E5%86%85%E6%A0%B8%E5%88%B0%E6%8C%87%E5%AE%9A%E7%9A%84%E7%89%88%E6%9C%AC&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-0-134273919.142^v102^pc_search_result_base2&spm=1018.2226.3001.4187](ubuntu降低使用的内核到指定的版本)

(1)先清理一遍原先的源：鱼香肉丝ros一键操作即可

(2)加入ubuntu focal仓库
  ```bash
  sudo nano /etc/apt/sources.list
  ```
  将以下内容加到后面
  ```txt
   deb http://archive.ubuntu.com/ubuntu/ focal main restricted universe multiverse
   deb http://archive.ubuntu.com/ubuntu/ focal-updates main restricted universe multiverse
   deb http://archive.ubuntu.com/ubuntu/ focal-security main restricted universe multiverse
  ```

(3)bash内操作
  ```bash
  apt-cache search linux| grep 5.11
  ```
  此时能够搜索到5.11.0-xx的内核版本
  ```bash
  sudo apt-get install linux-headers-5.11.0-44-generic linux-image-5.11.0-44-generic
  # 查看安装的内核版本
  dpkg --get-selections | grep linux-image
  # 以下两个是扩展，不一定成功，有网卡依赖不兼容问题
  # 反正降内核安装完就升回来了可以不管它
  sudo apt-get install linux-tools-5.11.0-44-generic
  sudo apt-get install linux-modules-extra-5.11.0-44-generic
  # 再次查看安装的内核版本
  dpkg --get-selections  | grep linux
  # 一定要更新一遍！
  sudo update-grub
  ```
  
(4)开机重启 按Esc Shift等进入grub界面 指定进入的内核

  ```bash
  # 查看内核版本，出现5.11即成功
  uname -r
  ```

(5)战队仓库**RM_PKA_Camera**内有大华驱动，下载后解压，然后执行
  ```bash
  chmod +x MVviewer_Ver2.3.1_Linux_x86_Build20210926.run 
  sudo ./MVviewer_Ver2.3.1_Linux_x86_Build20210926.run
  ```

(6)安装好后再次重启，在grub界面把内核版本换回来，后建议使用鱼香肉丝ros再次清理源。

##### 编译器版本
建议先运行ros2一键安装，其gcc版本即11。

### ROS2 humble + OpenCV

*注：鱼香肉丝一键安装ros内有包含opencv 4.5.0，故无需自行安装opencv。
  ```bash
  wget http://fishros.com/install -O fishros && . fishros
  ```
安装时注意换源。
如此前安装过大华相机驱动，此时也可把旧源一起清理掉。

### fmt库
  ```bash
  sudo apt install libfmt-dev
  ```

**以下为各数学库，请务必！按顺序安装**
**注：安装环境时请搞清楚各个库之间的相互依赖关系，请按顺序安装，如Ceres库作为Sophus的库依赖，而Sophus库则作为G2O库的依赖!**
**注意这些库都不能直接git clone项目，必须下载release中的文件，否则会导致make install时找不到版本头文件。**

### Ceres库
  
#### 安装相关依赖

   ```bash
   sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev
   ```

   **通过包管理器安装的依赖仍有不全，需补充下载abseil-cpp和googletest库**
   
#### googletest库

   ```bash
   wget https://github.com/google/googletest/releases/download/v1.16.0/googletest-1.16.0.tar.gz
   tar -xzvf googletest-1.16.0.tar.gz
   cd googletest-1.16.0
   mkdir build && cd build 
   cmake ..
   make -j1
   sudo make install
   ```

#### abseil-cpp库

   ```bash
   wget https://github.com/abseil/abseil-cpp/releases/download/20250127.1/abseil-cpp-20250127.1.tar.gz
   tar -xzvf abseil-cpp-20250127.1.tar.gz
   cd abseil-cpp-20250127.1
   mkdir build && cd build 
   cmake ..
   make -j1
   sudo make install
   ```
   
#### Cere源码编译
   ```bash
   wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.2.0.tar.gz
   tar -xzvf 2.2.0.tar.gz
   cd 2.2.0/ceres-solver-2.2.0
   mkdir build && cd build 
   cmake ..
   make -j1
   sudo make install
   ```

#### Cere apt下载
**由于源码编译后colcon build找不到cere，故还需apt一遍**

   ```bash
   sudo apt install libceres-dev
   ```

#### Sophus库
  
   **注：git下来的Sophus库的CMakeLists.txt中的cmake_minimum_required(VERSION xx.xx)的版本要求可能会高于系统的版本，将xx.xx改成符合系统的版本即可**
   
   ```bash
   wget https://github.com/strasdat/Sophus/archive/refs/tags/1.24.6.tar.gz
   tar -xzvf 1.24.6.tar.gz
   cd 1.24.6/Sophus-1.24.6
   mkdir build && cd build
   cmake ..
   make -j1
   sudo make install
   ```

#### G2O库
    
##### 安装依赖
   ```bash
   sudo apt install libeigen3-dev libspdlog-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
   ```

##### 源码编译
   ```bash
   wget https://github.com/RainerKuemmerle/g2o/archive/refs/tags/20241228_git.tar.gz
   tar -xzvf 20241228_git.tar.gz
   cd 20241228_git/g2o-20241228_git
   mkdir build && cd build
   cmake ..
   make -j1
   sudo make install
   ```

#### OpenVINO (神经网络识别)
  
   参考[OpenVINO官方文档](https://docs.openvino.ai/2025/index.html)

   ```bash
   wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
   sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
   echo "deb https://apt.repos.intel.com/openvino ubuntu22 main" | sudo tee /etc/apt/sources.list.d/intel-openvino.list
   sudo apt update
   apt-cache search openvino
   sudo apt install openvino-2025.4.0
   ```

**本文档中可能有缺漏，如有，可以用`rosdep`安装剩下依赖**

```bash
rosdep install --from-paths src --ignore-src -r -y
```

**注：在进行完本指令之后，如果直接编译运行会出现serial库的一些问题，需再执行一句指令便可成功编译和运行**

```bash
sudo apt remove libasio-dev
```

#### 导航相关

- Install [small_icp](https://github.com/koide3/small_gicp):

  ```bash
  sudo apt install -y libeigen3-dev libomp-dev
  
  git clone https://github.com/koide3/small_gicp.git
  cd small_gicp
  mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
  sudo make install
  ```

- Install [ndt_omp](https://github.com/koide3/ndt_omp.git):

  ```bash
  sudo apt install -y libeigen3-dev libomp-dev
  
  git clone https://github.com/koide3/ndt_omp.git
  cd small_gicp
  mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
  sudo make install
  ```

   ```bash
   sudo apt install ros-humble-perception-pcl \
            ros-humble-pcl-msgs \
            ros-humble-vision-opencv \
            ros-humble-xacro
   sudo apt install libpcap-dev
   sudo add-apt-repository ppa:borglab/gtsam-release-4.1
   sudo apt install libgtsam-dev libgtsam-unstable-dev
   sudo apt-get install libgeographic-dev
   sudo apt install ros-humble-nav2-*
   ```

   ```bash
   rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
   ```

   * 注：如有其他依赖未安装，请在编译时按实际情况操作。

## 三、编译与运行

**修改rm_bringup/config/launch_params.yaml，选择需要启动的功能，将serial模式设置成虚拟下位机（不懂看注释！），相机模式设置成相机模式，其他不动即可，成功编译运行会出现no camera是正常的！**

```bash
# 编译
rm -rf build install log
colcon build --symlink-install --parallel-workers 2 
#本仓库包含的功能包过多，建议限制同时编译的线程数
# 手动运行
source install/setup.bash
ros2 launch rm_bringup bringup.launch.py
```

* 注：可直接运行`build_vision.sh`，包含编译和自瞄部分的启动
导航部分的启动请使用`build_nav.sh`

**测试是否启动成功**
```bash
ros2 topic list
# 正常时会出现很多条，如果话题数量很少则说明未启动成功
# armor_detector/...
# armor_solver/...
# ...
```

## 四、开机自启动

* 注：自启动仅自瞄，导航上场前手动开启

- 编译程序后，进入rm_upstart文件夹

```bash
cd RM_PKA_...(对应文件夹)
cd rm_upstart
```

- 修改**rm_watch_dog.sh**中的`NAMESPACE`（ros命名空间）、`NODE_NAMES`（需要看门狗监控的节点）和`WORKING_DIR` （代码路径）

- 注册服务
  
```bash
cd rm_upstart
sudo chmod +x ./register_service.sh
sudo ./register_service.sh

# 正常时有如下输出
# Creating systemd service file at /etc/systemd/system/rm.service...
# Reloading systemd daemon...
# Enabling service rm.service...
# Starting service rm.service...
# Service rm.service has been registered and started.
```

- 查看程序状态

```bash
systemctl status rm
```

- 查看终端输出
```
查看screen.output或~/fyt2024-log下的日志
```

- 开启服务
```bash
systemctl start rm
```

- 开启自启动
```bash
systemctl enable rm
```
**注：建议此时重启pc进行测试**
```bash
# 重新启动
sudo reboot
# 关闭系统
sudo poweroff
```

- 关闭程序

```bash
systemctl stop rm
```

- 取消自启动

```bash
systemctl disable rm
```

## 五、调车步骤

### rviz2

```bash
rviz2
```
->订阅result_image话题


### foxglove

#### 安装
在官网安装foxgolve客户端
先安装foxglove的ros2依赖
```bash
sudo apt update
sudo apt install ros-humble-foxglove-bridge
```

#### 调试
在PKA自瞄包下先启动，然后在同目录下再开一个终端

```bash
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

此时打开foxglove：打开连接（不要选ros2那个选项！）->ws://localhost:xxxx（会自动填好）

> 获取hostname

```bash
# 获取ip地址
hostname -i
# 禁用防火墙
sudo ufw disable
```
ws://ip地址:xxxx
->点击topic可看到各类话题即为成功

## TF树关系查看
```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

## 将代码上传github
```bash
git clone https://github.com/FJNUpikachu/RM_PKA_Sentry.git
git add .
git commit -m "启动！"
git push origin main
```
```bash
git checkout -b instability
git push origin instability
```

## 维护者及开源许可证
待更新

## 致谢
待更新

ros2 run camera_calibration cameracalibrator --size 7x10 --square 0.022 image:=/image_raw

ros2 topic echo /nav_waypoint_id