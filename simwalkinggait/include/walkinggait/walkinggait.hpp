/**
 * @file /qlistener/listener.hpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef LISTENER_NODE_HPP_
#define LISTENER_NODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <stdio.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <std_msgs/String.h>

#include "tku_msgs/IKinfo_message.h"
#include "tku_msgs/Interface.h"
#include "tku_msgs/parameter.h"
#include "tku_msgs/WalkingGaitParameter.h"
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include "tku_msgs/HeadPackage.h"

#include <string>
#include "walkinggait/WalkingCycle.hpp"
#include "walkinggait/WalkingTrajectory.hpp"
#include "tku_libs/TKU_tool.h"
// #include "walkinggait/InverseKinematics.hpp"

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/mman.h>
#include <stdbool.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>

#define SIZE 100
using namespace std;

ros::Publisher WaveValue_Publish;
ros::Subscriber WaveRequest_subscriber;

ros::Subscriber ContinousMode_subscriber;
ros::Subscriber ChangeContinuousValue_subscriber;
ros::Subscriber chatter_subscriber;
ros::Subscriber save_parameter_subscriber;
ros::Subscriber walkack_subscriber;
ros::Subscriber continuousback_subscriber;
ros::ServiceServer LoadWalkinggaitParameter_service;

// ros::Publisher headmotor_pitch_pub;
// ros::Publisher headmotor_yaw_pub;

//---IPC FPGA---//
// ros::ServiceClient walkdata_client;
ros::Publisher walkdata_Pub;

//---//

//ros::Rate loop_rate(300);//

///////////////////////Timer//////////////////////////////////

ros::Timer timer;

///////////////////////Timer//////////////////////////////////

#define PI 3.1415926535897932384626433832795	//pi
//#define shift 100.0
#define gyrolsb 131.0

WalkingCycle walkingcycle;
WalkingTrajectory walkingtrajectory;
// InverseKinematics inversekinematics;

tku_msgs::IKinfo_message walkdata;
std_msgs::Int16 FPGAack;
std_msgs::Bool save;
ros::Publisher save_publish;
ToolInstance *tool = ToolInstance::getInstance();

// ros::Publisher NowStep_publish;

int teststop = 0;
int ContMode = 1;
bool walkack = false;
int strategyname = 8;
bool continuousback_flag = false;
bool sentdata = true;
int Ackcnt = 0;




#endif /* LISTENER_NODE_HPP_ */
