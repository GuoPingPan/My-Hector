//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include "Hector_Slam.h"
#include "map/GridMap.h"
#include "util/HectorMapMutex.h"

/**
 * @details 这里分为三种模式：SLAM，PrueLocalization，PrueMapping
 *          1.SLAM（同时定位与建图） 初始位置start_estimate 雷达输入scan 通过slamProcessor完成位置修正和建图
 *          2.PrueLocalization（纯定位模式）初始位置start_estimate 雷达输入scan 地图输入map_path update完成定位，不更新地图
 *          3.PrueMapping（纯建图）updata时直接使用输入的初始位置和雷达输入scan完成建图
 * @param[in] filename 参数配置文件
*/
HectorSLAM::HectorSLAM(std::string filename){

    InitParams(filename);

    if(state_ == SLAM){
        std::cout<<"SLAM"<<endl; 
        slamProcessor = new hectorslam::HectorSlamProcessor(
            static_cast<float>(p_map_resolution_),           //地图最高分辨率
            p_map_size_, p_map_size_,                        //地图的尺寸 像素大小
            Eigen::Vector2f(p_map_start_x_, p_map_start_y_), //地图的初始坐标，gridMap的左上角原点是地图实际尺寸的中心点
            p_map_multi_res_levels_);                        //地图层数

    }
    else if(state_ == PrueLocalization){
        std::cout<<"PrueLocalization"<<endl;
        slamProcessor = new hectorslam::HectorSlamProcessor(
            static_cast<float>(p_map_resolution_),
            p_map_size_, p_map_size_,
            Eigen::Vector2f(p_map_start_x_, p_map_start_y_),
            p_map_multi_res_levels_,
            true , load_map_path_);   // true表示开启纯定位模式，map_path为地图加载路径

    }
    else if(state_ == PrueMapping){
        std::cout<<"PrueMapping"<<endl;
        prue_mapping_ = true;    // true表示开启纯建图模式

        slamProcessor = new hectorslam::HectorSlamProcessor(
            static_cast<float>(p_map_resolution_),
            p_map_size_, p_map_size_,
            Eigen::Vector2f(p_map_start_x_, p_map_start_y_), 
            p_map_multi_res_levels_);
    }
    else{
        std::cerr<<"Error, the hector_state is not exist."<<
        " Please set the 'state = SLAM or PrueLocalization or PrueMapping.' "<<std::endl;
        exit(EXIT_FAILURE);
    }
    slamProcessor->setUpdateFactorFree(p_update_factor_free_);
    slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);
    slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_);
    slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);

    // 最高分辨率的图进行加锁，为了可视化
    slamProcessor->addMapMutex(0, new HectorMapMutex()); 

    // 地图线程
    map_show_thread_ = new boost::thread(boost::bind(&HectorSLAM::showMap, this, p_map_pub_period_));

}

HectorSLAM::~HectorSLAM()
{
  delete slamProcessor;

   if(map_show_thread_)
     delete map_show_thread_;
}

/**
 * @brief 接收雷达数据并处理
 * @param[in] scan 雷达数据 
*/
void HectorSLAM::scanCallback(const LaserScan& scan)
{
    start_time_ = std::chrono::steady_clock::now();
    
    Eigen::Vector3f start_estimate;

    if (init_pose_set_){    //当使用者要自行指定位姿时为最高优先级
        init_pose_set_ = false;
        start_estimate = init_pose_;
    }
    else if(use_odom_){     //其次是里程计
        start_estimate = odom_pose_;
    }
    else{                   //最后是只依赖雷达完成定位
        start_estimate = slamProcessor->getLastScanMatchPose();
    }

    // slamProcessor->getScaleToMap() 是第一层地图的分辨率 即mapResolution
    this->rosLaserScanToDataContainer(scan, laserScanContainer, slamProcessor->getScaleToMap());
    
    slamProcessor->update(laserScanContainer,start_estimate,prue_mapping_);

    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::duration<double>>(end_time-start_time_);

    Eigen::Vector3f updated_pose_;
    updated_pose_ = slamProcessor->getLastScanMatchPose();

    cout<<"update pose : "<< updated_pose_ <<endl;

}

/**
 * @brief 参数初始化，在HectorSLAM中调用
 * @param[in] filename 参数文件 
*/
void HectorSLAM::InitParams(std::string filename){
    ParamterReader paramterReader(filename);
    int state = paramterReader.getData<int>("state",0);
    state_ = HectorState(state);
    use_odom_ = paramterReader.getData<bool>("use_odom",false);
    p_map_pub_period_ = paramterReader.getData<double>("map_pub_period",10);
    load_map_path_ = paramterReader.getString("load_map_path",std::string());
    prue_mapping_ = paramterReader.getData<bool>("prue_mapping",false);
    
    p_map_resolution_ = paramterReader.getData<double>("map_resolution",0.05);
    p_map_size_ = paramterReader.getData<int>("map_size",2048);
    p_map_start_x_ = paramterReader.getData<double>("map_start_x",0.5);
    p_map_start_y_ = paramterReader.getData<double>("map_start_y",0.5);
    p_map_multi_res_levels_ = paramterReader.getData<int>("map_multi_res_levels",3);
    p_update_factor_free_ = paramterReader.getData<double>("update_factor_free",0.4);
    p_update_factor_occupied_ = paramterReader.getData<double>("update_factor_occupied",0.9);
    p_map_update_distance_threshold_ = paramterReader.getData<double>("map_update_distance_thresh",0.4);
    p_map_update_angle_threshold_ = paramterReader.getData<double>("map_update_angle_thresh",0.9);
    
    save_map_ = paramterReader.getData<bool>("save_map",false);
    save_map_path_ = paramterReader.getString("save_map_path",std::string());
    shut_down_ = false;
}

/**
 * @brief 雷达格式转换
 * @param[in] scan 参数文件
 * @param[in out] dataContainer Hector处理的雷达格式 
*/
void HectorSLAM::rosLaserScanToDataContainer(const LaserScan& scan, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = scan.ranges.size();

  float angle = scan.angle_min;

  dataContainer.clear();

  dataContainer.setOrigo(Eigen::Vector2f::Zero());

  float maxRangeForContainer = scan.range_max - 0.1f;

  for (size_t i = 0; i < size; ++i)
  {
    float dist = scan.ranges[i];

    if ( (dist > scan.range_min) && (dist < maxRangeForContainer))
    {
      dist *= scaleToMap;
      dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
    }

    angle += scan.angle_increment;
  }
}

/**
 * @brief 地图线程
 * @param[in] p_map_pub_period_ 地图更新频率 
*/
void HectorSLAM::showMap(double p_map_pub_period_){
    while(1){
        // 防止一张地图显示更新数次
        if (lastGetMapUpdateIndex != slamProcessor->getGridMap(0).getUpdateIndex()){ 
            // 获得最高分辨率地图转化为图片
            const hectorslam::GridMap &gridMap = slamProcessor->getGridMap(1);
            MapLockerInterface *mapMutex = slamProcessor->getMapMutex(0);

            delete this->map;
            this->map = new cv::Mat(gridMap.getSizeX(), gridMap.getSizeY(), CV_8UC1);
    
            if(mapMutex == nullptr)
                cerr<<"Warn, mutex not exit"<<endl;

            if (mapMutex)
                mapMutex->lockMap();

            for (int i = 0; i < map->rows; ++i) {
                uchar *data = map->ptr<uchar>(i);
                for (int j = 0; j < map->cols; ++j) {
                    if (gridMap.isFree(i, j)) {         // 空闲为白
                        data[j] = 255;
                    }
                    else if (gridMap.isOccupied(i, j)) {// 占用为黑
                        data[j] = 0;
                    }
                    else{                               // 未知为灰
                        data[j] = 150;
                    }
                }
            }

            lastGetMapUpdateIndex = slamProcessor->getGridMap(0).getUpdateIndex();

            if (mapMutex) {
                mapMutex->unlockMap();
            }

            cv::imshow("map",*map);
        }
        if(cv::waitKey(1000/p_map_pub_period_)=='q'){     // 按q退出程序

            if(save_map_){
                cout<<"Saving map, please wait......"<<endl;
                cv::imwrite(save_map_path_,*map);
                cout<<"Success, save map is finished"<<endl;
            }

            cout<<"Success, successfully shut down!"<<endl;
            shut_down_ = true;
            break;
        }
    }
}

