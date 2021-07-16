/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef WALKINGTRAJECTORY_HPP_
#define WALKINGTRAJECTORY_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>
#include <string.h>

#include <ros/ros.h>

#include "tku_libs/TKU_tool.h"
#include "walkinggait/parameterinfo.hpp"

using namespace std;

#define PI 3.1415926535897932384626433832795	//pi
#define PI_2 1.5707963267948966192313216916398	//pi/2

enum MoveState
{
    Straight,
    Right_Shift,
    Left_Shift,
    Right_Protect,
    Left_Protect,
    Right_turn,
    Left_Turn
};

class WalkingTrajectory
{
    public:
        WalkingTrajectory();
        ~WalkingTrajectory();

        void walkingProcess(int walking_mode);
        void walkFunction();
        void LCWalkFunction();
        void longJumpFunction();
        void continuousWalk();
        void inverseKinematicsInfo();

        double OSC_move_x_advance(double range, double period_T_ms, double rho_x, double BASE_delrho_x, double delta_x, int time_t_ms);
        double OSC_move_z(double range, double period_T_ms, double rho_z, double delta_z, int time_t_ms, int div_omega = 1);
        double OSC_COM_X(double range, double period_T_ms, double rho_com_x, double delta_com_x, int time_t_ms);
        double OSC_Rotate(double range, double period_T_ms, double rho_y, double delta_y, int time_t_ms);
        double OSC_move_shift_y(double range, double period_T_ms, double rho_y, double delta_y, int time_t_ms);
        double OSC_COM_Y(double period_T_ms, double rho_com_y, double delta_com_y, int time_t_ms);
        double OSC_COM_Z(double period_T_ms, double rho_com_z, double delta_com_z, int time_t_ms);
        double OSC_COM_Lift_X(double range, double period_T_ms, double rho_com_x, double delta_com_x, int time_t_ms);
        double OSC_lifemove_DOWNz(double range, double period_T_ms, double rho_z, double delta_z, double life_high, int time_t_ms, int Sample_Time);
        double OSC_lifemove_UPz(double range, double period_T_ms, double rho_z, double delta_z, double life_high, int time_t_ms, int Sample_Time);

        void SaveData();
        string DtoS(double value);

        ToolInstance *tool;

        vector<double> osc_move_x_r;
        vector<double> osc_move_x_l;
        vector<double> osc_move_y_r;
        vector<double> osc_move_y_l;
        vector<double> osc_move_z_r;
        vector<double> osc_move_z_l;
        vector<double> osc_move_com_x;
        vector<double> osc_move_com_y;
        vector<double> osc_move_com_z;
        vector<double> right_Theta;
        vector<double> left_Theta;
        vector<double> test;

        double wosc_foot_x_r;
        double wosc_foot_x_l;
        double wosc_foot_x_last;
        double wosc_foot_x_now;
        double lift_lock_x;
};
#endif /* WALKINGTRAJECTORY_HPP_ */
