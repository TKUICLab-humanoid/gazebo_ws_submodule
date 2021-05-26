#ifndef INVERSE_KINEMATIC_H_
#define INVERSE_KINEMATIC_H_

/******************* Include libarary*********************/
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdio.h>
#include <unistd.h>
/********************************************************/
#include "simmotionpackage/Feedback_Control.h"
#include "walkinggait/parameterinfo.hpp"
#include "tku_libs/TKU_tool.h"



/******************* Define******************************/
#define PI      3.1415926535897932384626433832795       //pi
#define PI_2    1.5707963267948966192313216916398       //pi/2
#define PI_3_2  4.7123889803846898576939650749193       //3*pi/2
#define PI_3    1.0471975511965977461542144610932       //pi/3
#define PI_TO_OUTPUT 651.8986469044032953093479120548   //360/(2pi)*4096/360
#define PI_6	PI / 6.0

#define Position_Zero   1024                            //Zero of Position    180 degree
#define Position_PI_2   2047                            //pi/2 of Position
#define Position_PI     3072                            //pi of Position
#define Max_value       4095
#define Min_value       0
#define Max_speed       1023
#define Gain_Min_speed  3000.0
#define Min_speed       1
#define ALL_Speed_Gain  1
#define ALL_Angle_Gain  1

#define Max_CPG_X		 30
#define Min_CPG_X		-30
#define Max_CPG_Y		 10
#define Min_CPG_Y		-10
#define Max_CPG_Z		 50
#define Min_CPG_Z		 30

#define SPEED_TRANS 32.0303030303030303030303030303     // (32767 / 1023)
#define COM_HEIGHT 24.3
#define STAND_OFFSET_RX 0
#define STAND_OFFSET_RY 0
#define STAND_OFFSET_RZ 0
#define STAND_OFFSET_LX 0
#define STAND_OFFSET_LY 0
#define STAND_OFFSET_LZ 0

#define SHOULDER_TO_COM 19.5

#define MID 0
#define STAND 0
#define RIGHT 1
#define LEFT 2

struct Points_Struct
{
	int P_Table[21];
	double UThta[21];
	double Thta[21];
	double Old_Thta[21];
    double Right_Thta;          //-pi/2~pi/2
    double Left_Thta;           //-pi/2~pi/2
    double X_Right_foot;        //cm
    double X_Left_foot;         //cm
    double X_COM;               //cm
    double Y_Right_foot;        //cm
    double Y_Left_foot;         //cm
    double Y_COM;               //cm
    double Z_Right_foot;        //cm
    double Z_Left_foot;         //cm
    double Z_COM;               //cm
    double Inverse_PointR_X;    //cm
    double Last_Inverse_PointR_X;//cm
    double Inverse_PointR_Y;    //cm
    double Last_Inverse_PointR_Y;//cm
    double Inverse_PointR_Z;    //cm
    double Inverse_PiontR_Thta; //-pi/2~pi/2
    double Inverse_PointL_X;    //cm
    double Inverse_PointL_Y;    //cm
    double Inverse_PointL_Z;    //cm
    double Inverse_PiontL_Thta; //-pi/2~pi/2
    double Inverse_Pointbody_X; //cm
    double Inverse_Pointbody_Y; //cm
    double Inverse_Pointbody_Z; //cm
    double Inverse_Piontbody_Thta;//-pi/2~pi/2
    
    //===================UNCONTROL===============================
    double Inverse_Uncontrol_PointR_X;    //cm
    double Inverse_Uncontrol_PointR_Y;    //cm
    double Inverse_Uncontrol_PointR_Z;    //cm
    double Inverse_Uncontrol_PointL_X;    //cm
    double Inverse_Uncontrol_PointL_Y;    //cm
    double Inverse_Uncontrol_PointL_Z;    //cm
};

struct Parameters_Struct
{
    double Phase_Shift;     //-pi~pi
    double X_Swing_Range;   //cm
    double Y_Swing_Range;   //cm
    double COM_Height;      //cm
    double l1;              //cm
    double l2;              //cm
    double R_X_Offset;      //cm
    double R_Y_Offset;      //cm
    double R_Z_Offset;      //cm
    double L_X_Offset;      //cm
    double L_Y_Offset;      //cm
    double L_Z_Offset;      //cm
    double COM_X_Offset;    //cm
    double COM_Y_Offset;    //cm
    double COM_Z_Offset;    //cm
    double R_Open;
    double L_Open;
    double Body_Pitch;      //-pi~pi
    double Body_Pitch_tmp;  //-pi~pi
    double Body_Roll;       //-pi~pi
    double Body_Yaw;        //-pi~pi
   //---------------Period_parameters---------------//
    int Period_T;           //ms
    int Period_T2;          //ms
    int Sample_Time;
    double Phase;
    double Phase_2; 
    double Open_Phase;
    //--------------Walk_Parameters------------------//
    double Push_Rate;       //?%
    //--------------Foot_Motor_Compensate------------------//
    double Thta[12];

};

struct Status_Struct
{
	int LoR;
	int Turn_LoR;
	double Pre_Y;
	int Move_ok_Y;
	int Up_FooT;
};


class InverseKinematic
{
    public:
        InverseKinematic();
        ~InverseKinematic();

        void initial_angle_gain();
        void initial_speed_gain();
        void initial_inverse_kinematic();
        void initial_points();
        void initial_parameters();
        void initial_points_process();
        void calculate_inverse_kinematic(int);
        void saveData();

        ToolInstance *tool;

    public:
        double speed_gain_[21];
        double angle_gain_[21];
        int angle_[21];
        int output_base_[21];
        int thta_base_[21];
        double past_thta_[21];
        double delay_time_[21];
        double past_delay_time_[21];
        unsigned int output_speed_[21];
        unsigned int output_angle_[21];
        double rotate_body_l_;
        bool flag_;

        int name_cont_;        
        std::map<std::string, std::vector<double>> map_motor;



};

#endif /*INVERSE_KINEMATIC_H_*/
