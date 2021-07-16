/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef WALKINGGAITBYLIPM_HPP_
#define WALKINGGAITBYLIPM_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <sys/time.h>

#include <ros/ros.h>

#include "tku_libs/TKU_tool.h"
#include "walkinggait/parameterinfo.hpp"

using namespace std;

#define PI 3.1415926535897932384626433832795
#define START_STEP_COUNT 2
class WalkingGaitByLIPM
{
    public:
        WalkingGaitByLIPM();
        ~WalkingGaitByLIPM();

        void initialize();
        void resetParameter();
        void process();

        double wComVelocityInit(double x0, double xt, double px, double t, double T);
        double wComPosition(double x0, double vx0, double px, double t, double T);
        double wFootPosition(const double start, const double length, const double t, const double T, const double T_DSP);
        double wFootPositionRepeat(const double start, const double length, const double t, const double T, const double T_DSP);
        double wFootPositionZ(const double height, const double t, const double T, const double T_DSP);
        double wFootTheta(const double theta, bool reverse, const double t, const double T, const double T_DSP);

        double unit_step(double x);
        double sinh(double x);
        double cosh(double x);

        //for test
        string DtoS(double value);
        void saveData();

        ToolInstance *tool;

        bool is_parameter_load_;
        bool ready_to_stop_;
        int moving_state_;
        int period_t_;
        int time_point_, sample_point_, sample_time_;
        int now_step_, pre_step_;
        int StartStepCount_, StartHeight_;
        int g_;
        double T_DSP_;
        int step_, left_step_, right_step_;
        double TT_, t_;
        double Tc_;
        double step_length_, width_size_, lift_height_, now_length_, now_width_;
        double theta_, abs_theta_, last_theta_;
        double last_step_length_, now_left_length_, now_right_length_, last_length_;
        double shift_length_, last_shift_length_, now_shift_, now_left_shift_, now_right_shift_, last_shift_;
        double step_point_lx_, step_point_rx_, step_point_ly_, step_point_ry_;
        double step_point_lz_, step_point_rz_, step_point_ltheta_, step_point_rtheta_;
        double vx0_, vy0_, px_, py_, pz_;
        double lpx_, rpx_, lpy_, rpy_, lpz_, rpz_, lpt_, rpt_;

        bool plot_once_, if_finish_;
        
        int name_cont_;
        std::map<std::string, std::vector<float>> map_walk;
};

#endif /* WALKINGGAITBYLIPM_HPP_ */
