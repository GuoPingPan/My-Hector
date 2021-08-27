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

#ifndef HECTOR_SLAM_PROCESSOR_H_
#define HECTOR_SLAM_PROCESSOR_H_

#include "../map/GridMap.h"
#include "../map/OccGridMapUtilConfig.h"
#include "../matcher/ScanMatcher.h"

#include "../scan/DataPointContainer.h"

#include "../util/UtilFunctions.h"
#include "../util/MapLockerInterface.h"

#include "MapRepresentationInterface.h"
#include "MapRepMultiMap.h"

#include <float.h>

namespace hectorslam
{

/**
* @details Hector 系统处理核，是Hector Slam的接口。其包含三个部分的函数：
*          1. 关键的 update() 函数，用于处理新的激光数据并更新slam状态，reset()用于重置hector系统；
*          2. get**()函数，获取hector的系统状态，包括pose、map及其他参数；
*          3. set 函数，用于设置地图部分参数，直接操作的是MapRep。
* @param MapRep 地图容器,最主要的一个类
*/
class HectorSlamProcessor
{
public:

    /**
     * @brief HectorSlamProcessor初始化，完成地图容器的构建
     * @param[in] mapResolution           地图分辨率
     * @param[in] mapSizeX                珊格地图尺寸
     * @param[in] mapSizeY 
     * @param[in] startCoords             地图起点坐标比例，一般取0.5，指的是中心点
     * @param[in] multi_res_size          多分辨率地图层数
     * @param[in] prue_localization=false 纯定位
     * @param[in] mapPath=""              地图加载路径
    */

    HectorSlamProcessor(float mapResolution,                 
                        int mapSizeX, int mapSizeY,         
                        const Eigen::Vector2f &startCoords, 
                        int multi_res_size,                  
                        bool prue_localization=false,       
                        std::string mapPath=""              
                        ){
        prueLocalization = prue_localization;

        if(prueLocalization){
            mapRep = new MapRepMultiMap(mapResolution, multi_res_size, startCoords,mapPath);
        }
        else{
            mapRep = new MapRepMultiMap(mapResolution, mapSizeX, mapSizeY, multi_res_size, startCoords);
        }

        this->reset();
        /* 设置进行地图更新的位姿变化初始阈值，在HectorSLAM初始化时会再次更新 */
        this->setMapUpdateMinDistDiff(0.4f * 1.0f);
        this->setMapUpdateMinAngleDiff(0.13f * 1.0f);

    }

    ~HectorSlamProcessor()
    {
        delete mapRep;
    }

    /**
    * @brief 对每一帧的激光数据进行处理
    * @param[in] dataContainer  激光数据存储容器，坐标已转换成地图尺度，为地图中激光系的坐标
    * @param[in] poseHintWorld  激光坐标系在地图中的初始pose，真实世界坐标系下的位姿
    * @param[in] prue_mapping   是否纯建图
    */
    void update(const DataContainer &dataContainer, const Eigen::Vector3f &poseHintWorld, bool prue_mapping = false)
    {

        /* 位姿匹配 */
        // newPoseEstimateWorld 也是真实世界坐标系下的位姿
        Eigen::Vector3f newPoseEstimateWorld;

        if (!prue_mapping){
            // 进行 scan to map 的地方
            newPoseEstimateWorld = (mapRep->matchData(poseHintWorld, dataContainer, lastScanMatchCov));
        }
        else{
            newPoseEstimateWorld = poseHintWorld;
        }

        lastScanMatchPose = newPoseEstimateWorld;

        /* 纯定位 */
        if(prueLocalization){
            lastMapUpdatePose = newPoseEstimateWorld;
            return;
        }

        /* 非纯定位，地图更新 */
        else{

            // newPoseEstimateWorld 没有经过更新
            if (util::poseDifferenceLargerThan(newPoseEstimateWorld, lastMapUpdatePose, paramMinDistanceDiffForMapUpdate, paramMinAngleDiffForMapUpdate) || prue_mapping)
            {
                // 仅在位姿变化大于阈值 或者 prue_mapping为真 的时候进行地图更新
                mapRep->updateByScan(dataContainer, newPoseEstimateWorld);
                mapRep->onMapUpdated();
                lastMapUpdatePose = newPoseEstimateWorld; // =  lastScanMatchPose
            }
        }
    }

    /* slam系统重置 */
    void reset()
    {
        lastMapUpdatePose = Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX); //#include<float.h>
        lastScanMatchPose = Eigen::Vector3f::Zero();
        
        /* 纯定位加载地图后无需重置 */
        if(prueLocalization)    return;

        mapRep->reset();
    }

    // 上一次匹配到的位姿
    const Eigen::Vector3f &getLastScanMatchPose() const { return lastScanMatchPose; };
    // 上一次匹配的协方差
    const Eigen::Matrix3f &getLastScanMatchCovariance() const { return lastScanMatchCov; };
    float getScaleToMap() const { return mapRep->getScaleToMap(); };

    // 获取地图层数
    int getMapLevels() const { return mapRep->getMapLevels(); };
    // 获取指定图层地图的常量引用
    const GridMap &getGridMap(int mapLevel = 0) const { return mapRep->getGridMap(mapLevel); };
    // 给指定图层添加互斥锁
    void addMapMutex(int i, MapLockerInterface *mapMutex) { mapRep->addMapMutex(i, mapMutex); };
    // 获取指定图层的锁
    MapLockerInterface *getMapMutex(int i) { return mapRep->getMapMutex(i); };

    // 设置概率、距离阈值参数
    void setUpdateFactorFree(float free_factor) { mapRep->setUpdateFactorFree(free_factor); };
    void setUpdateFactorOccupied(float occupied_factor) { mapRep->setUpdateFactorOccupied(occupied_factor); };
    void setMapUpdateMinDistDiff(float minDist) { paramMinDistanceDiffForMapUpdate = minDist; };
    void setMapUpdateMinAngleDiff(float angleChange) { paramMinAngleDiffForMapUpdate = angleChange; };

protected:
    MapRepresentationInterface *mapRep; // 地图接口对象--纯虚类

    Eigen::Vector3f lastMapUpdatePose;  // 根据地图更新后得到的位姿
    Eigen::Vector3f lastScanMatchPose;  // 匹配得到的位姿
    Eigen::Matrix3f lastScanMatchCov;   // 地图协方差矩阵

    float paramMinDistanceDiffForMapUpdate;
    float paramMinAngleDiffForMapUpdate;

    bool prueLocalization;

};// class HectorSlamProcessor

}// namespace hectorslam

#endif// HECTOR_SLAM_PROCESSOR_H_
