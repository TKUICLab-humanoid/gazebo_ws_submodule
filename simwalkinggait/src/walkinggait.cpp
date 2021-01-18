/**
 * @file /qlistener/listener.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "walkinggait/walkinggait.hpp"

// #include "tku_libs/modelinfo.h"

// ros::Publisher chatter_publisher1;
// ros::Publisher chatter_publisher2;
// ros::Publisher chatter_publisher3;
// ros::Publisher chatter_publisher4;
// ros::Publisher chatter_publisher5;
// ros::Publisher chatter_publisher6;
// ros::Publisher chatter_publisher7;
// ros::Publisher chatter_publisher8;
// ros::Publisher chatter_publisher9;
// ros::Publisher chatter_publisher10;
// ros::Publisher chatter_publisher11;
// ros::Publisher chatter_publisher12;

// ros::Publisher Left_H[4];
// ros::Publisher Right_H[4];
// ros::Publisher Head[2];

// ros::Publisher Waist;

// bool initail_flag = true;
// std_msgs::Float64 data13;
// std_msgs::Float64 data1,data2,data3,data4,data5,data6,data7,data8,data9,data10,data11,data12;
// float error=0, errors=0, errord=0, olderror=0, Ladd=0, Radd=0, Kp=0,Ki=0,Kd=0;//0.8 0.0 0.01


void ChangeContinuousValue(const tku_msgs::Interface& msg)
{
    ROS_INFO("change");
    parameterinfo->X = (double)msg.x/1000;
    parameterinfo->Y = (double)msg.y/1000;
    parameterinfo->Z = (double)msg.z/1000;
    parameterinfo->THTA = (double)msg.theta/180*PI;
    parameterinfo->sensor_mode = msg.sensor_mode;
    // if (parameterinfo->X >= 0.0)
    //     parameterinfo->walking_state = 0;
    // else
    //     parameterinfo->walking_state = 1;
}
void ContinousMode(const std_msgs::Int16& msg)
{
    ContMode = msg.data;
}
string DtoS(double value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();

    return str;
}
void save_parameter(const tku_msgs::parameter& msg)
{
    string savedText;
    fstream fp;
    char path[200];
    std::string PATH = tool->getPackagePath("strategy");
    strcpy(path, PATH.c_str());
    switch (msg.mode)
    {
    case 4:
    case 0:
        strcat(path, "/Single_Parameter.ini");
        fp.open(path, ios::out);
        savedText = "[General]\n";
        fp<<savedText;
        savedText = "X_Swing_Range = " + DtoS(msg.X_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Z_Swing_Range = " + DtoS(msg.Z_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
        fp<<savedText;
        savedText = "Period_T2 = " + DtoS(msg.Period_T2) + "\n";
        fp<<savedText;
        savedText = "Sample_Time = " + DtoS(msg.Sample_Time) + "\n";
        fp<<savedText;
        savedText = "OSC_LockRange = " + DtoS(msg.OSC_LockRange) + "\n";
        fp<<savedText;
        savedText = "BASE_Default_Z = " + DtoS(msg.BASE_Default_Z) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Shift = " + DtoS(msg.Y_Swing_Shift);
        fp<<savedText;
        break;
    case 1:
        if(continuousback_flag)
            strcat(path, "/Continuous_Back.ini");
        else
            strcat(path, "/Continuous_Parameter.ini");
        fp.open(path, ios::out);
        savedText = "[General]\n";
        fp<<savedText;
        savedText = "X_Swing_Range = " + DtoS(msg.X_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Z_Swing_Range = " + DtoS(msg.Z_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
        fp<<savedText;
        savedText = "Period_T2 = " + DtoS(msg.Period_T2) + "\n";
        fp<<savedText;
        savedText = "Sample_Time = " + DtoS(msg.Sample_Time) + "\n";
        fp<<savedText;
        savedText = "OSC_LockRange = " + DtoS(msg.OSC_LockRange) + "\n";
        fp<<savedText;
        savedText = "BASE_Default_Z = " + DtoS(msg.BASE_Default_Z);
        fp<<savedText;
        break;
    case 7:
        if(continuousback_flag)
            strcat(path, "/Continuous_Second_Back.ini");
        else
            strcat(path, "/Continuous_Second_Parameter.ini");
        fp.open(path, ios::out);
        savedText = "[General]\n";
        fp<<savedText;
        savedText = "X_Swing_Range = " + DtoS(msg.X_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Z_Swing_Range = " + DtoS(msg.Z_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
        fp<<savedText;
        savedText = "Period_T2 = " + DtoS(msg.Period_T2) + "\n";
        fp<<savedText;
        savedText = "Sample_Time = " + DtoS(msg.Sample_Time) + "\n";
        fp<<savedText;
        savedText = "OSC_LockRange = " + DtoS(msg.OSC_LockRange) + "\n";
        fp<<savedText;
        savedText = "BASE_Default_Z = " + DtoS(msg.BASE_Default_Z);
        fp<<savedText;
        break;
    case 8:
        if(continuousback_flag)
            strcat(path, "/Continuous_Third_Back.ini");
        else
            strcat(path, "/Continuous_Third_Parameter.ini");
        fp.open(path, ios::out);
        savedText = "[General]\n";
        fp<<savedText;
        savedText = "X_Swing_Range = " + DtoS(msg.X_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Z_Swing_Range = " + DtoS(msg.Z_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
        fp<<savedText;
        savedText = "Period_T2 = " + DtoS(msg.Period_T2) + "\n";
        fp<<savedText;
        savedText = "Sample_Time = " + DtoS(msg.Sample_Time) + "\n";
        fp<<savedText;
        savedText = "OSC_LockRange = " + DtoS(msg.OSC_LockRange) + "\n";
        fp<<savedText;
        savedText = "BASE_Default_Z = " + DtoS(msg.BASE_Default_Z);
        fp<<savedText;
        break;
    case 2:
        strcat(path, "/LCstep_Parameter.ini");
        fp.open(path, ios::out);
        savedText = "[General]\n";
        fp<<savedText;
        savedText = "X_Swing_Range = " + DtoS(msg.X_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Z_Swing_Range = " + DtoS(msg.Z_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
        fp<<savedText;
        savedText = "Period_T2 = " + DtoS(msg.Period_T2) + "\n";
        fp<<savedText;
        savedText = "Sample_Time = " + DtoS(msg.Sample_Time) + "\n";
        fp<<savedText;
        savedText = "OSC_LockRange = " + DtoS(msg.OSC_LockRange) + "\n";
        fp<<savedText;
        savedText = "BASE_Default_Z = " + DtoS(msg.BASE_Default_Z) + "\n";
        fp<<savedText;
        savedText = "X_Swing_COM = " + DtoS(msg.X_Swing_COM) + "\n";
        fp<<savedText;
        savedText = "BASE_LIFT_Z = " + DtoS(msg.BASE_LIFT_Z);
        fp<<savedText;
        break;
    case 3:
        strcat(path, "/LCdown_Parameter.ini");
        fp.open(path, ios::out);
        savedText = "[General]\n";
        fp<<savedText;
        savedText = "X_Swing_Range = " + DtoS(msg.X_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Z_Swing_Range = " + DtoS(msg.Z_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
        fp<<savedText;
        savedText = "Period_T2 = " + DtoS(msg.Period_T2) + "\n";
        fp<<savedText;
        savedText = "Sample_Time = " + DtoS(msg.Sample_Time) + "\n";
        fp<<savedText;
        savedText = "OSC_LockRange = " + DtoS(msg.OSC_LockRange) + "\n";
        fp<<savedText;
        savedText = "BASE_Default_Z = " + DtoS(msg.BASE_Default_Z) + "\n";
        fp<<savedText;
        savedText = "X_Swing_COM = " + DtoS(msg.X_Swing_COM) + "\n";
        fp<<savedText;
        savedText = "BASE_LIFT_Z = " + DtoS(msg.BASE_LIFT_Z);
        fp<<savedText;
        break;
    case 5:
        strcat(path, "/Single_wood.ini");
        fp.open(path, ios::out);
        savedText = "[General]\n";
        fp<<savedText;
        savedText = "X_Swing_Range = " + DtoS(msg.X_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Z_Swing_Range = " + DtoS(msg.Z_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
        fp<<savedText;
        savedText = "Period_T2 = " + DtoS(msg.Period_T2) + "\n";
        fp<<savedText;
        savedText = "Sample_Time = " + DtoS(msg.Sample_Time) + "\n";
        fp<<savedText;
        savedText = "OSC_LockRange = " + DtoS(msg.OSC_LockRange) + "\n";
        fp<<savedText;
        savedText = "BASE_Default_Z = " + DtoS(msg.BASE_Default_Z) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Shift = " + DtoS(msg.Y_Swing_Shift);
        fp<<savedText;
        break;
    case 6:
        strcat(path, "/Single_third.ini");
        fp.open(path, ios::out);
        savedText = "[General]\n";
        fp<<savedText;
        savedText = "X_Swing_Range = " + DtoS(msg.X_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Z_Swing_Range = " + DtoS(msg.Z_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
        fp<<savedText;
        savedText = "Period_T2 = " + DtoS(msg.Period_T2) + "\n";
        fp<<savedText;
        savedText = "Sample_Time = " + DtoS(msg.Sample_Time) + "\n";
        fp<<savedText;
        savedText = "OSC_LockRange = " + DtoS(msg.OSC_LockRange) + "\n";
        fp<<savedText;
        savedText = "BASE_Default_Z = " + DtoS(msg.BASE_Default_Z) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Shift = " + DtoS(msg.Y_Swing_Shift);
        fp<<savedText;
        break;
    }
    fp.close();
}
void Getparameter(const tku_msgs::Interface& msg)
{
    ROS_INFO("Getparameter");
    char line[SIZE];
    fstream fin;
    parameterinfo->X = (double)msg.x/1000;
    parameterinfo->Y = (double)msg.y/1000;
    parameterinfo->Z = (double)msg.z/1000;
    parameterinfo->THTA = (double)msg.theta/180*PI;
    parameterinfo->walking_mode = msg.walking_mode;
    parameterinfo->sensor_mode = msg.sensor_mode;
    // if (parameterinfo->X >= 0.0)
    //     parameterinfo->walking_state = 0;
    // else
    //     parameterinfo->walking_state = 1;
    walkack = true;
    char path[200];
    std::string PATH = tool->getPackagePath("strategy");
    strcpy(path, PATH.c_str());

    if( parameterinfo->walking_mode == 1 )//reload
    {
        if(parameterinfo->X >= 0)
        {
            strcat(path, "/Continuous_Parameter.ini");
        }
        else
        {
            strcat(path, "/Continuous_Back.ini");
        }
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        parameterinfo->parameters.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        parameterinfo->parameters.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        parameterinfo->parameters.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
    }
    else if( parameterinfo->walking_mode == 2 )
    {
        strcat(path, "/LCstep_Parameter.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        parameterinfo->parameters.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        parameterinfo->parameters.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        parameterinfo->parameters.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);

        parameterinfo->parameters.X_Swing_COM = tool->readvalue(fin,"X_Swing_COM",1);
        parameterinfo->parameters.BASE_LIFT_Z = tool->readvalue(fin,"BASE_LIFT_Z",1);
    }
    else if( parameterinfo->walking_mode == 3 )
    {
        strcat(path, "/LCdown_Parameter.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        parameterinfo->parameters.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        parameterinfo->parameters.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        parameterinfo->parameters.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);

        parameterinfo->parameters.X_Swing_COM = tool->readvalue(fin,"X_Swing_COM",1);
        parameterinfo->parameters.BASE_LIFT_Z = tool->readvalue(fin,"BASE_LIFT_Z",1);
    }
    else if(parameterinfo->walking_mode == 5)
    {
        strcat(path, "/Single_wood.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        parameterinfo->parameters.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        parameterinfo->parameters.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        parameterinfo->parameters.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        parameterinfo->parameters.Y_Swing_Shift = tool->readvalue(fin,"Y_Swing_Shift",1);
        if((parameterinfo->Y >= 1) || (parameterinfo->Y <= -1))
            parameterinfo->parameters.Y_Swing_Range -= parameterinfo->parameters.Y_Swing_Shift;
    }
    else if(parameterinfo->walking_mode == 6)
    {
        strcat(path, "/Single_third.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        parameterinfo->parameters.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        parameterinfo->parameters.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        parameterinfo->parameters.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        parameterinfo->parameters.Y_Swing_Shift = tool->readvalue(fin,"Y_Swing_Shift",1);
        if((parameterinfo->Y >= 1) || (parameterinfo->Y <= -1))
            parameterinfo->parameters.Y_Swing_Range -= parameterinfo->parameters.Y_Swing_Shift;
    }
    else if( parameterinfo->walking_mode == 7 )
    {
        if(parameterinfo->X >= 0)
        {
            strcat(path, "/Continuous_Second_Parameter.ini");
        }
        else
        {
            strcat(path, "/Continuous_Second_Back.ini");
        }
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        parameterinfo->parameters.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        parameterinfo->parameters.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        parameterinfo->parameters.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
    }
    else if( parameterinfo->walking_mode == 8 )
    {
        if(parameterinfo->X >= 0)
        {
            strcat(path, "/Continuous_Third_Parameter.ini");
        }
        else
        {
            strcat(path, "/Continuous_Third_Back.ini");
        }
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        parameterinfo->parameters.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        parameterinfo->parameters.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        parameterinfo->parameters.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
    }
    else
    {
        strcat(path, "/Single_Parameter.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        parameterinfo->parameters.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        parameterinfo->parameters.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        parameterinfo->parameters.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        parameterinfo->parameters.Y_Swing_Shift = tool->readvalue(fin,"Y_Swing_Shift",1);
        if((parameterinfo->Y >= 1) || (parameterinfo->Y <= -1))
            parameterinfo->parameters.Y_Swing_Range -= parameterinfo->parameters.Y_Swing_Shift;

    }

    if(parameterinfo->complan.walking_state == StopStep)
    {
        parameterinfo->complan.walking_stop = false;
        sentdata = true;
        parameterinfo->complan.walking_state = StartStep;
        parameterinfo->WalkFlag = true;
        parameterinfo->counter = 0;
        //walkingcycle.walkingkindfunction(parameterinfo->walking_mode);
        //walkingtrajectory.walkingprocess(parameterinfo->walking_mode);
        parameterinfo->Repeat = true;
    }
    else
    {
        parameterinfo->WalkFlag = false;
        parameterinfo->complan.walking_state = StopStep;
        parameterinfo->Repeat = false;
    }
    if(teststop && parameterinfo->walking_mode == 1)
    {
        parameterinfo->complan.walking_state = StopStep;
        teststop = 0;
    }
    else if(parameterinfo->walking_mode == 1)
    {
        teststop = 1;
        parameterinfo->parameters.abswaistx = 0;
    }
    if(teststop && parameterinfo->walking_mode == 7)
    {
        parameterinfo->complan.walking_state = StopStep;
        teststop = 0;
    }
    else if(parameterinfo->walking_mode == 7)
    {
        teststop = 1;
        parameterinfo->parameters.abswaistx = 0;
    }
    if(teststop && parameterinfo->walking_mode == 8)
    {
        parameterinfo->complan.walking_state = StopStep;
        teststop = 0;
    }
    else if(parameterinfo->walking_mode == 8)
    {
        teststop = 1;
        parameterinfo->parameters.abswaistx = 0;
    }
    parameterinfo->cpgack = true;
}
void walkackFunction(const std_msgs::Bool& msg)
{
    if(msg.data == true)
    {
        walkack = true;
    }
}

void IPCWalkingGaitFunction()
{
    walkingcycle.walkingkindfunction(parameterinfo->walking_mode);
    walkingtrajectory.walkingprocess(parameterinfo->walking_mode);
    // inversekinematics.DoIK((parameterinfo->parameters.Period_T / parameterinfo->parameters.Sample_Time));
    walkdata.IK_Point_RX = parameterinfo->points.IK_Point_RX;
    walkdata.IK_Point_RY = parameterinfo->points.IK_Point_RY;
    walkdata.IK_Point_RZ = parameterinfo->points.IK_Point_RZ;
    walkdata.IK_Point_RThta = parameterinfo->points.IK_Point_RThta;
    walkdata.IK_Point_LX = parameterinfo->points.IK_Point_LX;
    walkdata.IK_Point_LY = parameterinfo->points.IK_Point_LY;
    walkdata.IK_Point_LZ  = parameterinfo->points.IK_Point_LZ;
    walkdata.IK_Point_LThta = parameterinfo->points.IK_Point_LThta;
    walkdata.Sampletime = parameterinfo->parameters.Sample_Time;
    walkdata.Period_T = parameterinfo->parameters.Period_T;

    walkdata.Walking_State = parameterinfo->walking_state;
    walkdata.Sensor_Mode = parameterinfo->sensor_mode;
    parameterinfo->CPGalready = true;

    if(parameterinfo->complan.walking_stop)
    {
        save.data = true;
        save_publish.publish(save);
    }
}
bool LoadWalkingGaitParameterFunction(tku_msgs::WalkingGaitParameter::Request &req, tku_msgs::WalkingGaitParameter::Response &res)
{
    ROS_INFO("Load %d", req.mode);
    char line[SIZE];
    fstream fin;
    char path[200];
    std::string PATH = tool->getPackagePath("strategy");
    strcpy(path, PATH.c_str());

    switch(req.mode)
    {
    case 4:
    case 0:
        strcat(path, "/Single_Parameter.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        res.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        res.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        res.Period_T = tool->readvalue(fin,"Period_T",0);
        res.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        res.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        res.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        res.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        res.Y_Swing_Shift = tool->readvalue(fin,"Y_Swing_Shift",1);
        break;
    case 1:
        if(continuousback_flag)
            strcat(path, "/Continuous_Back.ini");
        else
            strcat(path, "/Continuous_Parameter.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        res.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        res.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        res.Period_T = tool->readvalue(fin,"Period_T",0);
        res.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        res.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        res.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        res.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        break;
    case 7:
        if(continuousback_flag)
            strcat(path, "/Continuous_Second_Back.ini");
        else
            strcat(path, "/Continuous_Second_Parameter.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        res.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        res.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        res.Period_T = tool->readvalue(fin,"Period_T",0);
        res.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        res.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        res.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        res.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        break;
    case 8:
        if(continuousback_flag)
            strcat(path, "/Continuous_Third_Back.ini");
        else
            strcat(path, "/Continuous_Third_Parameter.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        res.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        res.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        res.Period_T = tool->readvalue(fin,"Period_T",0);
        res.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        res.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        res.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        res.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        break;
    case 2:
        strcat(path, "/LCstep_Parameter.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        res.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        res.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        res.Period_T = tool->readvalue(fin,"Period_T",0);
        res.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        res.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        res.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        res.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        res.X_Swing_COM = tool->readvalue(fin,"X_Swing_COM",1);
        res.BASE_LIFT_Z = tool->readvalue(fin,"BASE_LIFT_Z",1);
        break;
    case 3:
        strcat(path, "/LCdown_Parameter.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        res.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        res.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        res.Period_T = tool->readvalue(fin,"Period_T",0);
        res.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        res.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        res.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        res.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        res.X_Swing_COM = tool->readvalue(fin,"X_Swing_COM",1);
        res.BASE_LIFT_Z = tool->readvalue(fin,"BASE_LIFT_Z",1);
        break;
    case 5:
        strcat(path, "/Single_wood.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        res.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        res.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        res.Period_T = tool->readvalue(fin,"Period_T",0);
        res.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        res.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        res.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        res.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        res.Y_Swing_Shift = tool->readvalue(fin,"Y_Swing_Shift",1);
        break;
    case 6:
        strcat(path, "/Single_third.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        res.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        res.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        res.Period_T = tool->readvalue(fin,"Period_T",0);
        res.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        res.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        res.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        res.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        res.Y_Swing_Shift = tool->readvalue(fin,"Y_Swing_Shift",1);
        break;
    }

    return true;
}
void ContinuousbackFunction(const std_msgs::Bool& msg)
{
    continuousback_flag = msg.data;
}

void SendWaveData(const std_msgs::Bool& msg)
{
    if(msg.data)
    {
        WaveValue_Publish.publish(walkingtrajectory.Wavelist);
    }
}

void timercallback(const ros::TimerEvent&)
{
    if(!parameterinfo->complan.walking_stop)
    {
        IPCWalkingGaitFunction();
        // data1.data = parameterinfo->ik_parameters.Ltest_theta[5];
        // data2.data = parameterinfo->ik_parameters.Ltest_theta[0]+Ladd;
        // data3.data = parameterinfo->ik_parameters.Ltest_theta[1]-0.105 + 0.25;//-0.105;
        // data4.data = parameterinfo->ik_parameters.Ltest_theta[2]+0.104 - 0.5;
        // data5.data = parameterinfo->ik_parameters.Ltest_theta[3]-0.045 + 0.25;
        // data6.data = -parameterinfo->ik_parameters.Ltest_theta[4]-Ladd;

        // data7.data = -parameterinfo->ik_parameters.Rtest_theta[5];
        // data8.data = parameterinfo->ik_parameters.Rtest_theta[0]+Radd;
        // data9.data = -(parameterinfo->ik_parameters.Rtest_theta[1]-0.105 + 0.25);//+0.105;
        // data10.data = -(parameterinfo->ik_parameters.Rtest_theta[2]+0.104 - 0.5);
        // data11.data = -(parameterinfo->ik_parameters.Rtest_theta[3]-0.045 + 0.25);
        // data12.data = -parameterinfo->ik_parameters.Rtest_theta[4]-0.1-Radd;

        // // data1.data = parameterinfo->ik_parameters.Ltest_theta[5];
        // // data2.data = parameterinfo->ik_parameters.Ltest_theta[0]+Ladd;
        // // data3.data = parameterinfo->ik_parameters.Ltest_theta[1]-0.105+0.38;//-0.105;
        // // data4.data = parameterinfo->ik_parameters.Ltest_theta[2]+0.104-0.45*2;
        // // data5.data = parameterinfo->ik_parameters.Ltest_theta[3]-0.045+0.45;
        // // data6.data = -parameterinfo->ik_parameters.Ltest_theta[4]-Ladd;

        // // data7.data = -parameterinfo->ik_parameters.Rtest_theta[5];
        // // data8.data = parameterinfo->ik_parameters.Rtest_theta[0]+Radd;
        // // data9.data = -(parameterinfo->ik_parameters.Rtest_theta[1]-0.105+0.38);//+0.105;
        // // data10.data = -(parameterinfo->ik_parameters.Rtest_theta[2]+0.104-0.45*2);
        // // data11.data = -(parameterinfo->ik_parameters.Rtest_theta[3]-0.045+0.45);
        // // data12.data = -parameterinfo->ik_parameters.Rtest_theta[4]-0.1-Radd;

        // chatter_publisher1.publish(data1);
        // chatter_publisher2.publish(data2);
        // chatter_publisher3.publish(data3);
        // chatter_publisher4.publish(data4);
        // chatter_publisher5.publish(data5);
        // chatter_publisher6.publish(data6);

        // chatter_publisher7.publish(data7);
        // chatter_publisher8.publish(data8);
        // chatter_publisher9.publish(data9);
        // chatter_publisher10.publish(data10);
        // chatter_publisher11.publish(data11);
        // chatter_publisher12.publish(data12);

        // parameterinfo->CPGalready = false;
        walkdata_Pub.publish(walkdata);
    }
}
// void GetHeadMotor(const tku_msgs::HeadPackage &msg)
// {
//     std_msgs::Float64 aa;
//     aa.data = (msg.Position-511)/512.0*3.14159265;
//     ROS_INFO("%f",aa.data);
//     if(msg.ID == 0)
//     {
//         headmotor_yaw_pub.publish(aa);
//     }
//     else
//     {
//             //ROS_ERROR("11");
//         headmotor_pitch_pub.publish(aa);
//     }
// }
int main(int argc, char **argv)
{
    ros::init(argc, argv, "WalkingGait");
    ros::NodeHandle n;
    ros::NodeHandle nhPrivate("~");
    ros::CallbackQueue my_queue;
    nhPrivate.setCallbackQueue(&my_queue);
    ros::AsyncSpinner spinner(2, &my_queue);
    spinner.start();
    ros::AsyncSpinner s(4);
    s.start();
    // tool_gz->set_Client(n);
    string robot_ganeration;
    robot_ganeration = "11th";
    nhPrivate.getParam("robot", robot_ganeration);
    nhPrivate.deleteParam("robot");
    // if(robot_ganeration == "10th")
    // {
    //     parameterinfo->parameters.COM_Height = 21.7;
    // }
    // else //if(robot_ganeration == "11th")
    // {
    //     parameterinfo->parameters.COM_Height = 17.9;
    // }
    parameterinfo->parameters.COM_Height = 24.3;

    WaveRequest_subscriber = n.subscribe("/WaveRequest_Topic", 100, SendWaveData);
    ContinousMode_subscriber = n.subscribe("/ContinousMode_Topic", 100, ContinousMode);
    ChangeContinuousValue_subscriber = n.subscribe("/ChangeContinuousValue_Topic", 100, ChangeContinuousValue);
    chatter_subscriber = n.subscribe("SendBodyAuto_Topic", 1000, Getparameter);
    save_parameter_subscriber = n.subscribe("web/parameter_Topic", 100, save_parameter);
    walkack_subscriber = n.subscribe("/package/walkack", 100, walkackFunction);
    continuousback_subscriber = n.subscribe("/walkinggait/Continuousback", 100, ContinuousbackFunction);
    LoadWalkinggaitParameter_service = n.advertiseService("web/LoadWalkingGaitParameter", LoadWalkingGaitParameterFunction);
///----------------------Gazebo---------------------------///
    // ros::Subscriber headmotor_sub = n.subscribe("/package/HeadMotor", 1, GetHeadMotor);
    // headmotor_pitch_pub = n.advertise<std_msgs::Float64>("/kidsize/head_pitch_position_controller/command", 1);
    // headmotor_yaw_pub = n.advertise<std_msgs::Float64>("/kidsize/neck_yaw_position_controller/command", 1);

    // chatter_publisher1 = nhPrivate.advertise<std_msgs::Float64>("/kidsize/left_hip_yaw_position_controller/command", 1);
    // chatter_publisher2 = nhPrivate.advertise<std_msgs::Float64>("/kidsize/left_hip_roll_position_controller/command", 1);
    // chatter_publisher3 = nhPrivate.advertise<std_msgs::Float64>("/kidsize/left_hip_pitch_position_controller/command", 1);
    // chatter_publisher4 = nhPrivate.advertise<std_msgs::Float64>("/kidsize/left_knee_pitch_position_controller/command", 1);
    // chatter_publisher5 = nhPrivate.advertise<std_msgs::Float64>("/kidsize/left_ankle_pitch_position_controller/command", 1);
    // chatter_publisher6 = nhPrivate.advertise<std_msgs::Float64>("/kidsize/left_ankle_roll_position_controller/command", 1);

    // chatter_publisher7 = nhPrivate.advertise<std_msgs::Float64>("/kidsize/right_hip_yaw_position_controller/command", 1);
    // chatter_publisher8 = nhPrivate.advertise<std_msgs::Float64>("/kidsize/right_hip_roll_position_controller/command", 1);
    // chatter_publisher9 = nhPrivate.advertise<std_msgs::Float64>("/kidsize/right_hip_pitch_position_controller/command", 1);
    // chatter_publisher10 = nhPrivate.advertise<std_msgs::Float64>("/kidsize/right_knee_pitch_position_controller/command", 1);
    // chatter_publisher11 = nhPrivate.advertise<std_msgs::Float64>("/kidsize/right_ankle_pitch_position_controller/command", 1);
    // chatter_publisher12 = nhPrivate.advertise<std_msgs::Float64>("/kidsize/right_ankle_roll_position_controller/command", 1);

    // Left_H[0] = n.advertise<std_msgs::Float64>("/kidsize/left_shoulder_pitch_position_controller/command", 1);
    // Left_H[1] = n.advertise<std_msgs::Float64>("/kidsize/left_shoulder_roll_position_controller/command", 1);
    // Left_H[2] = n.advertise<std_msgs::Float64>("/kidsize/left_middle_yaw_position_controller/command", 1);
    // Left_H[3] = n.advertise<std_msgs::Float64>("/kidsize/left_elbow_pitch_position_controller/command", 1);
    // Right_H[0] = n.advertise<std_msgs::Float64>("/kidsize/right_shoulder_pitch_position_controller/command", 1);
    // Right_H[1] = n.advertise<std_msgs::Float64>("/kidsize/right_shoulder_roll_position_controller/command", 1);
    // Right_H[2] = n.advertise<std_msgs::Float64>("/kidsize/right_middle_yaw_position_controller/command", 1);
    // Right_H[3] = n.advertise<std_msgs::Float64>("/kidsize/right_elbow_pitch_position_controller/command", 1);

    // Waist = n.advertise<std_msgs::Float64>("/kidsize/waist_yaw_position_controller/command", 1);
    //     //Head[0] = nhPrivate.advertise<std_msgs::Float64>("/kidsize/head_pitch_position_controller/command", 1);
    //     //Head[1] = nhPrivate.advertise<std_msgs::Float64>("/kidsize/neck_yaw_position_controller/command", 1);
///----------------------Gazebo---------------------------///

    //---IPC FPGA---//
    //walkdata_client = n.serviceClient<tku_msgs::IKinfo>("/package/walkingdata");
    WaveValue_Publish = n.advertise<tku_msgs::wavelist>("/wave", 1000);

    walkdata_Pub = nhPrivate.advertise<tku_msgs::IKinfo_message>("/package/walkingdata", 1000);
    save_publish = n.advertise<std_msgs::Bool>("/package/save", 1000);
    
    //walkingtrajectory.initRosNodeHandle(n);
    //---//
    // int i;

    ros::Timer timer = n.createTimer(ros::Duration(0.0333), timercallback);

    // nhPrivate.getParam("Kp",Kp);
    // nhPrivate.getParam("Ki",Ki);
    // nhPrivate.getParam("Kd",Kd);
    // ROS_INFO("Kp=%f Ki=%f Kd=%f", Kp, Ki, Kd);
    
    ros::Rate loop_rate(50);//

    while (ros::ok())
    {
        // if (parameterinfo->walking_mode == 0 || parameterinfo->walking_mode == 1)
        // {
        //     tool_gz->get_modelinfo("kidsize");
        //     tool_gz->get_simtime();

        //     olderror = error;
        //     error = 0.0-(tool_gz->mapModelInfo["kidsize"].Angular.x);
        //     if(abs(tool_gz->Current_Sim_Time_sec-tool_gz->OldCurrent_Sim_Time_sec) < 0.05)
        //     {
        //         errors = errors+error*(tool_gz->Current_Sim_Time_sec-tool_gz->OldCurrent_Sim_Time_sec);
        //         errord = (olderror-error)/(tool_gz->Current_Sim_Time_sec-tool_gz->OldCurrent_Sim_Time_sec);
        //     }

        //     Ladd = 0;
        //     Radd = 0;
        //     if(tool_gz->get_linkposeZ("left_foot_link") > 0.0355 && tool_gz->get_linkposeZ("left_foot_link") < 0.075 && tool_gz->get_linkposeZ("right_foot_link") <= 0.0355 && abs(tool_gz->mapModelInfo["kidsize"].Angular.x*180/3.14159265) < 10)
        //     {
        //         Radd = -(Kp*error+Ki*errors+Kd*errord);
        //     }
        //     else if(tool_gz->get_linkposeZ("right_foot_link") > 0.0355 && tool_gz->get_linkposeZ("right_foot_link") < 0.075 && tool_gz->get_linkposeZ("left_foot_link") <= 0.0355 && abs(tool_gz->mapModelInfo["kidsize"].Angular.x*180/3.14159265) < 10)
        //     {
        //         Ladd = -(Kp*error+Ki*errors+Kd*errord);
        //     }
        //     else
        //     {
        //         Ladd = 0;
        //         Radd = 0;
        //     }
        //     //ROS_WARN("Radd = %f", Radd);
        //     //ROS_WARN("Ladd = %f", Ladd);
        //     //ROS_WARN("-------------------");
        // }
        //     //ros::spinOnce();

        //---
        // ROS_INFO("%f",tool_gz->get_jointproperties("left_middle_yaw"));
        // ROS_INFO("%d", initail_flag);
        // ROS_INFO("%f",tool_gz->get_linkposeZ("left_foot_link")-tool_gz->get_linkposeZ("left_thigh_link"));
        // if(initail_flag)
        // {
        //     if(tool_gz->get_jointproperties("left_middle_yaw") < 0.002 && tool_gz->get_jointproperties("left_middle_yaw") > -0.002)initail_flag = false;
        //     data13.data = 0.0;
        //     Left_H[0].publish(data13);
        //     Left_H[1].publish(data13);
        //     Left_H[3].publish(data13);
        //     Right_H[0].publish(data13);
        //     Right_H[1].publish(data13);
        //     Right_H[3].publish(data13);
        //     data13.data = 0;
        //     Waist.publish(data13);

        //     data13.data = 0;
        //     Left_H[2].publish(data13);
        //     data13.data = 0;
        //     Right_H[2].publish(data13);
        // }
        loop_rate.sleep();
    }

    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    // nhPrivate.setParam("Kp",0);
    // nhPrivate.setParam("Ki",0);
    // nhPrivate.setParam("Kd",0);
    return 0;
}
