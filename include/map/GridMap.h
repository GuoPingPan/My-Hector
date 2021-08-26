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

#ifndef _LESSON4_GRIDMAP_h_
#define _LESSON4_GRIDMAP_h_

#include "OccGridMapBase.h"

#include <cmath>

/**
 * Provides a log odds of occupancy probability representation for cells in a occupancy grid map.
 * 就是占用珊格地图的 log odds 对数概率
 */
class LogOddsCell
{
public:

    /**
   * Sets the cell value to val.
   * @param val The log odds value.
   */
    void set(float val)
    {
        logOddsVal = val;
    }

    /**
   * Returns the value of the cell.
   * @return The log odds value.
   */
    float getValue() const
    {
        return logOddsVal;
    }

    /**
   * Returns wether the cell is occupied.
   * @return Cell is occupied
   */
    bool isOccupied() const
    {
        return logOddsVal > 0.0f;
    }

    bool isFree() const
    {
        return logOddsVal < 0.0f;
    }

    /**
   * Reset Cell to prior probability.
   */
    void resetGridCell()
    {
        logOddsVal = 0.0f;
        updateIndex = -1;
    }

    //protected:

public:
    float logOddsVal; ///< The log odds representation of occupancy probability.
    int updateIndex;
};

/**
 * Provides functions related to a log odds of occupancy probability respresentation for cells in a occupancy grid map.
 * 提供处理 log odds的工具
 */
class GridMapLogOddsFunctions
{
public:
    /**
   * Constructor, sets parameters like free and occupied log odds ratios.
   */
    GridMapLogOddsFunctions()
    {
        this->setUpdateFreeFactor(0.4f);
        this->setUpdateOccupiedFactor(0.6f);
    }

    /**
   * Update cell as occupied
   * @param cell The cell.
   */
    void updateSetOccupied(LogOddsCell &cell) const
    {
        //yi ge dian zui da shi 50.0f
        // dan zhi yao > 0.0f jiu shi bei zhan yong
        if (cell.logOddsVal < 50.0f)
        {
            cell.logOddsVal += logOddsOccupied;
        }
    }

    /**
   * Update cell as free
   * @param cell The cell.
   */
    void updateSetFree(LogOddsCell &cell) const
    {

        cell.logOddsVal += logOddsFree;
    }

    void updateUnsetFree(LogOddsCell &cell) const
    {
        cell.logOddsVal -= logOddsFree;
    }

    /**
   * Get the probability value represented by the grid cell.
   * 获得该 cell 的占用率
   * @param cell The cell.
   * @return The probability
   */
    float getGridProbability(const LogOddsCell &cell) const
    {
        float odds = exp(cell.logOddsVal);
        return odds / (odds + 1.0f);
    }

    void setUpdateFreeFactor(float factor)
    {
        // factor = 0.4
        // log(0.4/1-0.4) = -0.176
        logOddsFree = probToLogOdds(factor);
    }

    void setUpdateOccupiedFactor(float factor)
    {
        // factor = 0.9
        // log(0.9/1-0.9) = 0.9542
        logOddsOccupied = probToLogOdds(factor);
    }

protected:
    inline float probToLogOdds(float prob)
    {
        float odds = prob / (1.0f - prob);
        return log(odds);
    }

    float logOddsOccupied; /// < The log odds representation of probability used for updating cells as occupied
    float logOddsFree;     /// < The log odds representation of probability used for updating cells as free
};


namespace hectorslam {

typedef OccGridMapBase<LogOddsCell, GridMapLogOddsFunctions> GridMap;
//typedef OccGridMapBase<SimpleCountCell, GridMapSimpleCountFunctions> GridMap;
//typedef OccGridMapBase<ReflectanceCell, GridMapReflectanceFunctions> GridMap;

}

#endif
