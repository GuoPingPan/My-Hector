### 简述

此MyHector算法基于原算法[hector_slam](https://github.com/tu-darmstadt-ros-pkg/hector_slam)的基础上进行了去ROS化，无需安装ROS(Robot Operation System)便可使用，同时还去除了某些用处不大的调试组件，并合并了对应文件，尽量简化整体工程的结构，实现高度集成。此外，还会继续实现其他SLAM算法的去ROS化，以便SLAM小伙伴能直接对算法进行使用与学习。

### MyHector整体结构图      [(算法原理详解)](https://gitee.com/guopingpan/my_-hector)

![image](https://upload-images.jianshu.io/upload_images/23877926-9c051770525d62ed.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)
### 使用说明

#### 一、环境搭配

#####  1.安装Eigen
- 方法1.linux自带库下载
```
sudo apt-get install libeigen3-dev
#位置会被安装到"/usr/include/eigen3/" , 可以通过一下命令查看路径
sudo updatedb
locate eigen3
```
- 方法2.源码编译
通过github下载或者官网下载压缩包
```
wget  https://gitlab.com/libeigen/eigen/-/archive/3.2.10/eigen-3.2.10.tar.gz
#进行解压
tar -xvzf eigen-3.2.10.tar.gz 
#通过cmake编译
cd eigen-3.2.10
mkdir build && cd build
cmake ..
sudo make install #默认安装到/usr/local/include/eigen3路径下
#可以修改此路径，其基本结构为<CMAKE_INSTALL_PREFIX>/<INCLUDE_INSTALL_DIR>
#默认:CMAKE_INSTALL_PREFIX 为 /usr/local，INCLUDE_INSTALL_DIR 为 include/eigen3
#cmake 时可以修改参数
#例如：
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
#就可以将安装路径设置到 /usr/include/eigen3/
```
#####  2.安装OpenCV ( [请参考](https://www.jianshu.com/p/403ab3aa04ed) )
#### 二、使用
当上面的库都安装完成后，便可以使用该算法
== 注意 ==：这里案例中使用的是思岚科技rplidarA系列雷达，如需使用自己的雷达建图可参考下面的 **使用自己的雷达建图**
```
git clone https://gitee.com/guopingpan/my_-hector.git -b combine
cd my_hector
mkdir build
cd build
cmake ..
make
#运行
cd ..
bin/hector_mapping       param/rplidar.txt
```
#### 二、参数配置(请参考默认文件)
##### param/rplidar.txt
```
channel_type = serial  // 两种雷达数据传输方式
serial_port = /dev/rplidar
#serial/tcp
tcp_ip = 192.168.0.7    // 选择tcp方式才会用到，这里我们使用serial
tcp_port = 20108

serial_baudrate = 115200 // 波特率
hector_param_file = ../param/hector_slam.txt  // hector_slam配置路径

frame_id = laser_frame
inverted = false   // 是否上下翻转安装雷达
angle_compensate = false // 是否进行角度补偿
scan_mode = Boost // 扫描模式 
//详情请查询 https://github.com/Slamtec/rplidar_sdk.git
#	Standard: max_distance: 8.0 m, Point number: 2.0K
#	Express: max_distance: 8.0 m, Point number: 4.0K
#	Boost: max_distance: 8.0 m, Point number: 8.0K
#	Stability: max_distance: 8.0 m, Point number: 4.0K

```
##### param/hector_slam.txt
```
state = 0  // hector 工作模式     0：SLAM      1：Prue_Localization      2：PrueMapping
use_odom = false  // 是否使用里程计，抱歉这里尚未完善，请设为false
map_pub_period = 10 // 地图更新频率，默认：10Hz
load_map_path =  ../map/map1.png  // 地图的加载路径
save_map = false  // 是否保存地图
save_map_path = ../map/map2.png // 地图的保存路径

map_size = 400  // 地图的大小，长宽均为400个网格
map_resolution = 0.05 // 地图的分辨率
map_start_x = 0.5 // 地图的起点在地图中的占比，如 400*0.5 = 200
map_start_y = 0.5
map_multi_res_levels = 3  // 多分辨率地图层数
update_factor_free = 0.4 // 占用栅格地图的 空闲概率
update_factor_occupied = 0.9 // 占用栅格地图的 占用概率
map_update_distance_thresh = 0.4 // 地图更新的距离阈值
map_update_angle_thresh = 0.06 // 地图更新的角度阈值
```
#### 三、模式选择

*   SLAM（state=0）
 同时定位与建图模式，想要保存地图，设置`param/hector_slam.txt`
```
state = 0
...

save_map = true
save_map_path = 你想保存的路径
```
*   prueLocalization（state=1）
纯定位模式，需要先验地图，将SLAM模式建好的地图进行使用，设置`param/hector_slam.txt`
```
state = 1
...

load_map_path = 加载地图的路径
```
*   prueMapping（state=2）
纯建图模式，要保证输入的里程计准确，则直接使用该里程计建图，当前尚未完善

#### 四、使用自己的雷达建图

上述方法编译完成后，可以发现在`/bin`文件夹下有一个可执行文件`hector_mapping`，在`/lib`文件夹下有一个共享库`libhector_slam.so`

##### 在自己编写的`main.cpp`程序中
```
#include "Hector_Slam.h"
...

HectorSLAM hector_slam("../param/hector_slam.txt");

LaserScan scan = 自己对数据进行处理;

hector_slam.callback(scan);
```
##### 在CMakeLists.txt文件中
```
set(HECTORSLAM_LIBRARY hector_slam )
target_link_libraries(main ${HECTORSLAM_LIBRARY})
```

#### 五、有问题怎么办?
qq:731061720