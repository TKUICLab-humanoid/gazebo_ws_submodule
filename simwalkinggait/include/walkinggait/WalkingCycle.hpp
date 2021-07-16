/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef WALKINGCYCLE_HPP_
#define WALKINGCYCLE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#include <iostream>
#include <math.h>

#include <ros/ros.h>

#include "walkinggait/parameterinfo.hpp"

#define INCREASE_SLOPE 1
#define WALK_MAX_DISTANCE 2

class WalkingCycle
{  
    public:
        WalkingCycle();
        ~WalkingCycle();

        void walkingKindFunction(int walking_mode);
        void singleStepFunction(int walking_mode);
        void singleStepWalkingProcess();
        void LCWalkingProcess();
        void continuousStepFunction();
        void continuousWalkingProcess();

        bool IsStop;
        
        int slopeCounter_;
        int Sample_points_quater;
        double forwardValue_;
        double forwardCounter_;

        double COM_Y_tmp;
        double Lockrange_tmp;
};

#endif // WALKINGCYCLE_HPP_
