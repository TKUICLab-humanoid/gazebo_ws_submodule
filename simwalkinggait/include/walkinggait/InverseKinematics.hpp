#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include "walkinggait/parameterinfo.hpp"
#include <vector>

using namespace std;

//-----Link-----//
#define d0 -3.35
#define a0 4.0
//#define a2 12.5
//#define a3 12.5
#define a5 3.5
////-------Right-------//
#define R_nz -1.0
#define R_oy 1.0
#define R_ax 1.0
//-------Left-------//
#define L_nz -1.0
#define L_oy 1.0
#define L_ax 1.0

#define PI_TO_OUTPUT 651.8986469044032953093479120548 //360/(2pi)*1024/300
#define Position_Zero   1024                       //Zero of Position    180«×
#define Max_value       4095
#define PI 3.1415926535897932384626433832795	//pi
#define PI_2 1.5707963267948966192313216916398	//pi/2
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.v = to explain what is being done
 */
class InverseKinematics
{

public:

    vector<float> theta_r0;
    vector<float> theta_r1;
    vector<float> theta_r2;
    vector<float> theta_r3;
    vector<float> theta_r4;

    vector<float> theta_l0;
    vector<float> theta_l1;
    vector<float> theta_l2;
    vector<float> theta_l3;
    vector<float> theta_l4;

    InverseKinematics();
    ~InverseKinematics();
    void Initail_IK();
    void R_IK(float *Goal);
    void L_IK(float *Goal);
    void DoIK(int Motion_Delay);
    void SaveData();
    string FtoS(float value);

    int *temp_motion;

private:
    int Thta_Base[21];
    bool P_Table[21];
    float Past_Thta[21];
    float UThta[21];

    unsigned int Output_Angle[21];
    unsigned int Output_Speed[21];
    int Output_Base[21];

    float Delay_time[21];
    float Past_Delay_time[21];
    double test[20][100];
    int testnum;
    void Initail_Parameters();

};
//extern InverseKinematics *inversekinematics;
#endif // INVERSEKINEMATICS_H
