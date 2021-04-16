/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef WALKINGGAIT_NODE_HPP_
#define WALKINGGAIT_NODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/mman.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

#include "tku_msgs/IKinfo_message.h"
#include "tku_msgs/Interface.h"
#include "tku_msgs/parameter.h"
#include "tku_msgs/WalkingGaitParameter.h"

#include "tku_libs/TKU_tool.h"
#include "walkinggait/WalkingCycle.hpp"
#include "walkinggait/WalkingTrajectory.hpp"
#include "walkinggait/WalkingGaitByLIPM.hpp"

using namespace std;

#define PI 3.1415926535897932384626433832795	//pi

class WalkingGait
{
    public:
        WalkingGait(ros::NodeHandle &nh, ros::NodeHandle &nhPrivate);
        ~WalkingGait();

        void changeContinuousValue(const tku_msgs::Interface& msg);
        void continuousBackFunction(const std_msgs::Bool& msg);
        void getParameter(const tku_msgs::Interface& msg);
        void saveParameter(const tku_msgs::parameter& msg);
        void walkAckFunction(const std_msgs::Bool& msg);

        bool loadWalkingGaitParameterFunction(tku_msgs::WalkingGaitParameter::Request &req, tku_msgs::WalkingGaitParameter::Response &res);

        string DtoS(double value);
        void timerCallBack(const ros::TimerEvent&);
        void computerWalkingGaitFunction();

        //----------------- n -----------------//
        ros::Subscriber changeContinuousValue_sub;
        ros::Subscriber continuousBack_sub;
        ros::Subscriber continuousMode_sub;
        ros::Subscriber getParameter_sub;
        ros::Subscriber saveParameter_sub;
        ros::Subscriber walkAck_sub;
        ros::Publisher save_pub;

        ros::ServiceServer loadWalkingGaitParameter_ser;

        ros::Timer timer;

        //------------- nhPrivate -------------//
        ros::Publisher walkingData_pub;

        std_msgs::Bool saveData;
        tku_msgs::IKinfo_message walkingData;

        ToolInstance *tool = ToolInstance::getInstance();

        WalkingCycle walkingCycle;
        WalkingTrajectory walkingTrajectory;
        WalkingGaitByLIPM walkingGaitByLIPM;

        bool continuousBackFlag;
        int pre_theta;
        int pre_x;
};

#endif /* WALKINGGAIT_NODE_HPP_ */
