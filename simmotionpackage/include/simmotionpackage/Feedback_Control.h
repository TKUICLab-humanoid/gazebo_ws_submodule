#ifndef FEEDBACK_CONTROL_H_
#define FEEDBACK_CONTROL_H_

#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <fstream>
#include <vector>
#include <map>
#include <eigen3/Eigen/Dense>
#include "simmotionpackage/Inverse_kinematic.h"
#include "simmotionpackage/ZMPProcess.h"
#include "walkinggait/WalkingGaitByLIPM.hpp"


class ZMPProcess;

using namespace std;
#define PI                  3.1415926535897932384626433832795   //pi
#define PI_2                1.5707963267948966192313216916398   //pi/2
#define Angle_2_PI          PI/180
#define PI_2_Angle          180/PI
#define DEGREE2RADIAN		(M_PI / 180.0)
#define RADIAN2DEGREE		(180.0/ M_PI)

/*******************Function Define******************************/
#define Auto_Gyro_offset
/********************************************************/

#define GRAVITATIONAL_ACCELERATION 9.8	//9.8(m/s^2)
#define ANKLE_HEIGHT 2.6 //2.6(cm)
#define COM_HEIGHT_TO_GROUND (COM_HEIGHT + ANKLE_HEIGHT)
// #define IMU_HEIGHT 7.462 //7.462(cm)

/////////////////////////Posture///////////////////////
#define Acc_offset_X            	0
#define Acc_offset_Y            	0
#define Acc_offset_Z            	0
#define Gyro_manual_offset_X    	0
#define Gyro_manual_offset_Y    	0
#define Gyro_manual_offset_Z    	0
#define kalman_manual_offset_Pitch	0
#define kalman_manual_offset_Roll	0
#define kalman_manual_offset_Yaw	0
#define Gyro_LSB_2_Angle        	16.4
#define Acc_LSB_2_G					2048.0
#define index_Pitch             	0
#define index_Roll              	1
#define index_Yaw					2
#define Posture_Gain				0.3
////////////////////////////////////////////////////////

/////////////////////////Zmp Control///////////////////////
#define DOUBLE_FEET_WEIGHT_FAR_Y	10.5	//7.6 cm
#define DOUBLE_FEET_WEIGHT_NEAR_Y	3.0	//3.0 cm
#define DOUBLE_FEET_WEIGHT_X		7.15	//4.4 cm
#define DOUBLE_FEET_BALANCE_POINT_Y	4.5	//4.5 cm
#define SINGLE_FOOT_WEIGHT_FAR_Y	3.1	//3.1 cm
#define SINGLE_FOOT_WEIGHT_NEAR_Y	1.5	//1.5 cm
#define SINGLE_FOOT_WEIGHT_X		7.15	//4.4 cm

#define RIGHT_PRESS_SHIFT	4
#define SINGLE_FOOT_WEIGHT_EQUAL_Y	3.9
///////////////////////////////////////////////////////////

//////////////////////for Fall down & Get up/////////////////////////////
#define RPY_ROLL_LIMIT 		1.133	//65 degree
#define RPY_PITCH_LIMIT 	1.133	//65 degree
#define RPY_STAND_RANGE 	0.52	//30 degree

enum class imu {roll = 0,pitch,yaw};
typedef enum {leftfoot = 0,rightfoot,doublefeet} etSupFoot;

class ButterWorthParam
{
public:
    static ButterWorthParam set(float a1, float a2, float b1, float b2);
    float a_[2];
    float b_[2];
};

class ButterWorthFilter
{
public:
    ButterWorthFilter();
    ~ButterWorthFilter();
    void initialize(ButterWorthParam param);
    float getValue(float present_value);
private:
    ButterWorthParam param_;
    float prev_output_;
    float prev_value_;
};

class PID_Controller
{
public:
    PID_Controller(float Kp, float Ki, float Kd);
    PID_Controller();
    ~PID_Controller();
    void initParam();
    void setKpid(double Kp, double Ki, double Kd);
    void setControlGoal(float x1c = 0, float x2c = 0, float x3c = 0);
    void setValueLimit(float upper_limit, float lower_limit);
    // void setDataValue(float value);
    float calculateExpValue(float value);
    float calculateExpValue_roll(float value);	
    float getError();
    float getErrors();
    float getErrord();
    // void setErrorValue(float error);
    // void setErrorsValue(float errors);
    // void setErrordValue(float errord);
    // float getFixValue();
private:
    double Kp;
    double Ki;
    double Kd;
    float error;
    float pre_error;
    float errors;
    float errord;
    float x1c;
    float x2c;
    float x3c;
    float exp_value;
    float value;
    float pre_value;
    float upper_limit;
    float lower_limit;
};

typedef struct IMUParameter IMUParam;
struct IMUParameter
{
    float pos;
    float vel;
    void initialize()
    {
        pos = 0;
        vel = 0;
    }
    // float acc;
};
class SimIMUData
{
    public:
		double qx,qy,qz,qw,g_x,g_y,g_z,a_x,a_y,a_z;
		double sensor_rpy[3];
        float Roll = 0;
        float Pitch = 0;
        float Yaw = 0;
};

typedef struct BalanceParameter BalanceParam;
struct BalanceParameter
{
    float control_value_total;
    float control_value_once;
    void initialize()
    {
        control_value_total = 0;
        control_value_once = 0;
    }
};

typedef struct ButterWorthIMUParameter ButterWorthIMUParam;
struct ButterWorthIMUParameter
{
    ButterWorthFilter pos;
    ButterWorthFilter vel;
    void initialize()
    {
        pos.initialize(ButterWorthParam::set(1, -0.676819, 0.161590, 0.161590)); //fs = 33 , fc = 2, n = 1;
        vel.initialize(ButterWorthParam::set(1, -0.676819, 0.161590, 0.161590)); //fs = 33 , fc = 2, n = 1;
    }
};
class BalanceLowPassFilter
{
public:
	BalanceLowPassFilter();
	~BalanceLowPassFilter();

	void initialize(double control_cycle_sec, double cut_off_frequency);
	void set_cut_off_frequency(double cut_off_frequency);
	double get_cut_off_frequency(void);
	double get_filtered_output(double present_raw_value);

private:
	double cut_off_freq_;
	double control_cycle_sec_;
	double alpha_;
	double prev_output_;
};

class BalanceControl
{
public:
	BalanceControl();
	~BalanceControl();

	struct timeval timer_start_, timer_end_;
    double timer_dt_;
	
	void initialize(const int control_cycle_msec);
	void balance_control();
	void get_sensor_value();
	void control_after_ik_calculation();

	//LIPM
	void setSupportFoot();
	void resetControlValue();
	void endPointControl();
	float calculateCOMPosbyLIPM(float pos_adj, float vel);

	

	BalanceLowPassFilter roll_imu_lpf_;
	BalanceLowPassFilter pitch_imu_lpf_;

	double roll_imu_filtered_;
	double pitch_imu_filtered_;
	bool two_feet_grounded_;
	bool roll_over_limit_;
	bool pitch_over_limit_;
	double cog_roll_offset_;
	double cog_pitch_offset_;
	double foot_cog_x_;
	double foot_cog_y_;	

	etSupFoot sup_foot_, pre_sup_foot_;

	IMUParam init_imu_value[3];
    IMUParam pres_imu_value[3];
    IMUParam prev_imu_value[3];
    IMUParam ideal_imu_value[3];
    IMUParam passfilter_pres_imu_value[3];
    IMUParam passfilter_prev_imu_value[3];

	ButterWorthIMUParam butterfilter_imu[3];
	//hip
	BalanceParam leftfoot_hip_roll_value;
    BalanceParam leftfoot_hip_pitch_value;
	BalanceParam rightfoot_hip_roll_value;
	BalanceParam rightfoot_hip_pitch_value;
	PID_Controller PIDleftfoot_hip_roll;
    PID_Controller PIDleftfoot_hip_pitch;
	PID_Controller PIDrightfoot_hip_roll;
    PID_Controller PIDrightfoot_hip_pitch;
	//ankle
	BalanceParam leftfoot_ankle_roll_value;
    BalanceParam leftfoot_ankle_pitch_value;
	BalanceParam rightfoot_ankle_roll_value;
	BalanceParam rightfoot_ankle_pitch_value;
	PID_Controller PIDleftfoot_ankle_roll;
    PID_Controller PIDleftfoot_ankle_pitch;
	PID_Controller PIDrightfoot_ankle_roll;
    PID_Controller PIDrightfoot_ankle_pitch;

	BalanceParam leftfoot_EPx_value;	//EP = End point
	BalanceParam leftfoot_EPy_value;
	BalanceParam rightfoot_EPx_value;
	BalanceParam rightfoot_EPy_value;
	PID_Controller PIDleftfoot_zmp_x;
	PID_Controller PIDleftfoot_zmp_y;
	PID_Controller PIDrightfoot_zmp_x;
	PID_Controller PIDrightfoot_zmp_y;

	BalanceParam CoM_EPx_value;
	PID_Controller PIDCoM_x;

	ZMPParam pres_ZMP;
    ZMPParam prev_ZMP;
    ZMPParam ideal_ZMP;

	ZMPProcess *ZMP_process;

	float leftfoot_hip_roll;
    float leftfoot_hip_pitch;
    float leftfoot_ankle_roll;
    float leftfoot_ankle_pitch;
	float rightfoot_hip_roll;
    float rightfoot_hip_pitch;
    float rightfoot_ankle_roll;
    float rightfoot_ankle_pitch;

	void saveData();
	string DtoS(double value);



	double original_ik_point_rz_, original_ik_point_lz_;

	std::map<std::string, float> map_param;
    std::map<std::string, std::vector<float>> map_roll;
    std::map<std::string, std::vector<float>> map_pitch;
	std::map<std::string, std::vector<float>> map_ZMP;
	std::map<std::string, std::vector<float>> map_CoM;
	std::map<std::string, std::vector<float>> map_Accel;

	int name_cont_;

};

#endif /*FEEDBACK_CONTROL_H_*/

