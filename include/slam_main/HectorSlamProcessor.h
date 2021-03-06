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
* @details Hector ?????????????????????Hector Slam?????????????????????????????????????????????
*          1. ????????? update() ????????????????????????????????????????????????slam?????????reset()????????????hector?????????
*          2. get**()???????????????hector????????????????????????pose???map??????????????????
*          3. set ????????????????????????????????????????????????????????????MapRep???
* @param MapRep ????????????,?????????????????????
*/
class HectorSlamProcessor
{
public:

    /**
     * @brief HectorSlamProcessor???????????????????????????????????????
     * @param[in] mapResolution           ???????????????
     * @param[in] mapSizeX                ??????????????????
     * @param[in] mapSizeY 
     * @param[in] startCoords             ????????????????????????????????????0.5?????????????????????
     * @param[in] multi_res_size          ????????????????????????
     * @param[in] prue_localization=false ?????????
     * @param[in] mapPath=""              ??????????????????
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
        /* ?????????????????????????????????????????????????????????HectorSLAM??????????????????????????? */
        this->setMapUpdateMinDistDiff(0.4f * 1.0f);
        this->setMapUpdateMinAngleDiff(0.13f * 1.0f);

    }

    ~HectorSlamProcessor()
    {
        delete mapRep;
    }

    /**
    * @brief ???????????????????????????????????????
    * @param[in] dataContainer  ??????????????????????????????????????????????????????????????????????????????????????????
    * @param[in] poseHintWorld  ????????????????????????????????????pose????????????????????????????????????
    * @param[in] prue_mapping   ???????????????
    */
    void update(const DataContainer &dataContainer, const Eigen::Vector3f &poseHintWorld, bool prue_mapping = false)
    {

        /* ???????????? */
        // newPoseEstimateWorld ???????????????????????????????????????
        Eigen::Vector3f newPoseEstimateWorld;

        if (!prue_mapping){
            // ?????? scan to map ?????????
            newPoseEstimateWorld = (mapRep->matchData(poseHintWorld, dataContainer, lastScanMatchCov));
        }
        else{
            newPoseEstimateWorld = poseHintWorld;
        }

        lastScanMatchPose = newPoseEstimateWorld;

        /* ????????? */
        if(prueLocalization){
            lastMapUpdatePose = newPoseEstimateWorld;
            return;
        }

        /* ??????????????????????????? */
        else{

            // newPoseEstimateWorld ??????????????????
            if (util::poseDifferenceLargerThan(newPoseEstimateWorld, lastMapUpdatePose, paramMinDistanceDiffForMapUpdate, paramMinAngleDiffForMapUpdate) || prue_mapping)
            {
                // ?????????????????????????????? ?????? prue_mapping?????? ???????????????????????????
                mapRep->updateByScan(dataContainer, newPoseEstimateWorld);
                mapRep->onMapUpdated();
                lastMapUpdatePose = newPoseEstimateWorld; // =  lastScanMatchPose
            }
        }
    }

    /* slam???????????? */
    void reset()
    {
        lastMapUpdatePose = Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX); //#include<float.h>
        lastScanMatchPose = Eigen::Vector3f::Zero();
        
        /* ???????????????????????????????????? */
        if(prueLocalization)    return;

        mapRep->reset();
    }

    // ???????????????????????????
    const Eigen::Vector3f &getLastScanMatchPose() const { return lastScanMatchPose; };
    // ???????????????????????????
    const Eigen::Matrix3f &getLastScanMatchCovariance() const { return lastScanMatchCov; };
    float getScaleToMap() const { return mapRep->getScaleToMap(); };

    // ??????????????????
    int getMapLevels() const { return mapRep->getMapLevels(); };
    // ???????????????????????????????????????
    const GridMap &getGridMap(int mapLevel = 0) const { return mapRep->getGridMap(mapLevel); };
    // ??????????????????????????????
    void addMapMutex(int i, MapLockerInterface *mapMutex) { mapRep->addMapMutex(i, mapMutex); };
    // ????????????????????????
    MapLockerInterface *getMapMutex(int i) { return mapRep->getMapMutex(i); };

    // ?????????????????????????????????
    void setUpdateFactorFree(float free_factor) { mapRep->setUpdateFactorFree(free_factor); };
    void setUpdateFactorOccupied(float occupied_factor) { mapRep->setUpdateFactorOccupied(occupied_factor); };
    void setMapUpdateMinDistDiff(float minDist) { paramMinDistanceDiffForMapUpdate = minDist; };
    void setMapUpdateMinAngleDiff(float angleChange) { paramMinAngleDiffForMapUpdate = angleChange; };

protected:
    MapRepresentationInterface *mapRep; // ??????????????????--?????????

    Eigen::Vector3f lastMapUpdatePose;  // ????????????????????????????????????
    Eigen::Vector3f lastScanMatchPose;  // ?????????????????????
    Eigen::Matrix3f lastScanMatchCov;   // ?????????????????????

    float paramMinDistanceDiffForMapUpdate;
    float paramMinAngleDiffForMapUpdate;

    bool prueLocalization;

};// class HectorSlamProcessor

}// namespace hectorslam

#endif// HECTOR_SLAM_PROCESSOR_H_
