/*
 * Copyright 2021 The Project Author: guopingpan
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef HECTOR_SLAM_H_
#define HECTOR_SLAM_H_

#include "map/GridMap.h"
#include "slam_main/HectorSlamProcessor.h"
#include "scan/DataPointContainer.h"
#include "util/MapLockerInterface.h"
#include "util/HectorMapMutex.h"
#include "ParamReader.h"

#include <boost/thread.hpp>
#include <string>
#include <chrono>
#include <opencv2/opencv.hpp>

enum  HectorState:int{
    SLAM = 0,
    PrueLocalization = 1,
    PrueMapping = 2
};

/**
 * @brief 雷达数据结构 LaserScan
 * @param seq 数据类型id
 * @param stamp 时间戳
 * @param angle_increment 两个相邻雷达点扫描角度间隔
 * @param time_increment  两个相邻雷达点扫描时间间隔
 * @param ranges 雷达数据距离
*/
struct LaserScan{
    float seq; 
    std::chrono::steady_clock::time_point stamp;
    std::string frame_id;
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    std::vector<float> ranges;
    std::vector<float> intensities;
};

/**
 * @brief HectorSLAM，完成雷达数据处理、定位、建图
 * @param map_show_thread_ 地图线程
 * @param slamProcessor slam处理类
 * @param laserScanContainer 雷达数据
*/
class HectorSLAM
{
public:

    /**
     * @brief 完成初始化工作
     * @param[in] filename 参数文件 
    */
    HectorSLAM(std::string filename);
    ~HectorSLAM();

    /**
     * @brief 接收雷达数据并处理
     * @param[in] scan 雷达数据 
    */
    void scanCallback(const LaserScan &scan);

    /**
     * @brief 地图线程
     * @param[in] p_map_pub_period_ 地图更新频率 
    */
    void showMap(double p_map_pub_period_);

    bool shut_down_;

private:

    /**
     * @brief 参数初始化，在HectorSLAM中调用
     * @param[in] filename 参数文件 
    */
    void InitParams(std::string filename);

    /**
     * @brief 雷达格式转换
     * @param[in] scan 参数文件
     * @param[in out] dataContainer Hector处理的雷达格式 
    */
    void rosLaserScanToDataContainer(const LaserScan& scan, hectorslam::DataContainer& dataContainer, float scaleToMap);

protected:

    
    HectorState state_;     // 工作模式
    
    bool init_pose_set_;    // 初始位置输入
    Eigen::Vector3f init_pose_;
    
    bool use_odom_;         // 是否使用odom
    Eigen::Vector3f odom_pose_;
    
    bool prue_mapping_;    // 纯建图
    bool save_map_;        // 保存地图
    
    std::string load_map_path_; // 纯定位要加上地图路径
    std::string save_map_path_;

    boost::thread *map_show_thread_;
    cv::Mat *map;
    hectorslam::HectorSlamProcessor *slamProcessor;
    hectorslam::DataContainer laserScanContainer;

    int lastGetMapUpdateIndex;  // 地图索引


    // 地图更新参数
    double p_update_factor_free_;
    double p_update_factor_occupied_;
    double p_map_update_distance_threshold_;
    double p_map_update_angle_threshold_;

    // map parameters --- resolution / size / init pose / map levels 
    double p_map_resolution_;
    int p_map_size_;
    double p_map_start_x_;
    double p_map_start_y_;
    int p_map_multi_res_levels_;
    double p_map_pub_period_;

    bool p_timing_output_;
    
    // laser data filter
    float p_sqr_laser_min_dist_;
    float p_sqr_laser_max_dist_;
    float p_laser_z_min_value_;
    float p_laser_z_max_value_;

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
    std::chrono::duration<double> time_used_;

};// class HectoSLAM

#endif // HECTOR_SLAM_H_
