#include "walkinggait/walkinggait.hpp"

WalkingGait::WalkingGait(ros::NodeHandle &nh, ros::NodeHandle &nhPrivate)
{
    parameterinfo->parameters.COM_Height = 24.3;

    changeContinuousValue_sub = nh.subscribe("/ChangeContinuousValue_Topic", 100, &WalkingGait::changeContinuousValue, this);
    continuousBack_sub = nh.subscribe("/walkinggait/Continuousback", 100, &WalkingGait::continuousBackFunction, this);
    getParameter_sub = nh.subscribe("SendBodyAuto_Topic", 1000, &WalkingGait::getParameter, this);
    saveParameter_sub = nh.subscribe("web/parameter_Topic", 100, &WalkingGait::saveParameter, this);
    save_pub = nh.advertise<std_msgs::Bool>("/package/save", 1000);

    loadWalkingGaitParameter_ser = nh.advertiseService("web/LoadWalkingGaitParameter", &WalkingGait::loadWalkingGaitParameterFunction, this);

    timer = nh.createTimer(ros::Duration(0.03), &WalkingGait::timerCallBack, this);

    walkingData_pub = nhPrivate.advertise<tku_msgs::IKinfo_message>("/package/walkingdata", 1000);

    continuousBackFlag = false;
    pre_theta = 0;
    pre_x = 0;
}

WalkingGait::~WalkingGait()
{

}

void WalkingGait::changeContinuousValue(const tku_msgs::Interface& msg)
{
    parameterinfo->X = (double)msg.x/1000.0;
    parameterinfo->Y = (double)msg.y/1000.0;
    parameterinfo->Z = (double)msg.z/1000.0;
    if(abs(msg.theta) > 20)
    {
        parameterinfo->THETA = (double)pre_theta/180.0*PI;
    }
    else
    {
        parameterinfo->THETA = (double)msg.theta/180.0*PI;
        pre_theta = msg.theta;
    }
    parameterinfo->sensor_mode = msg.sensor_mode;

    std::string sensorMode;
    if(parameterinfo->sensor_mode == 0)
    {
        sensorMode = "None";
    }
    else if(parameterinfo->sensor_mode == 1)
    {
        sensorMode = "Pitch";
    }
    else if(parameterinfo->sensor_mode == 2)
    {
        sensorMode = "Roll";
    }
    else if(parameterinfo->sensor_mode == 3)
    {
        sensorMode = "RollPitch";
    }
    ROS_INFO("Change Value [%-6d %-6d %-4d; %s]\n", parameterinfo->X, parameterinfo->Y, parameterinfo->THETA, sensorMode.c_str());

    if((pre_x < 0 && parameterinfo->X >= 0) || (pre_x >= 0 && parameterinfo->X < 0))
    {
        std::printf("pre_x:%d\tx:%d\n", pre_x, parameterinfo->X);
        fstream fin;
        char path[200];
        char line[100];
        
        strcpy(path, tool->parameterPath.c_str());
        if(parameterinfo->walking_mode == 1)
        {
            if(parameterinfo->X >= 0)
            {
                strcat(path, "/Continuous_Parameter.ini");
            }
            else
            {
                strcat(path, "/Continuous_Back.ini");
            }
        }
        else if(parameterinfo->walking_mode == 7)
        {
            if(parameterinfo->X >= 0)
            {
                strcat(path, "/Continuous_Second_Parameter.ini");
            }
            else
            {
                strcat(path, "/Continuous_Second_Back.ini");
            }
        }
        else if(parameterinfo->walking_mode == 8)
        {
            if(parameterinfo->X >= 0)
            {
                strcat(path, "/Continuous_Third_Parameter.ini");
            }
            else
            {
                strcat(path, "/Continuous_Third_Back.ini");
            }
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

        std::printf("YSWING:%f\n", parameterinfo->parameters.Y_Swing_Range);

        fin.close();
    }
    pre_x = parameterinfo->X;
}

void WalkingGait::continuousBackFunction(const std_msgs::Bool& msg)
{
    continuousBackFlag = msg.data;
}

void WalkingGait::getParameter(const tku_msgs::Interface& msg)
{
    parameterinfo->X = (double)msg.x/1000.0;
    parameterinfo->Y = (double)msg.y/1000.0;
    parameterinfo->Z = (double)msg.z/1000.0;
    parameterinfo->THETA = (double)msg.theta/180.0*PI;
    parameterinfo->walking_mode = msg.walking_mode;
    parameterinfo->sensor_mode = msg.sensor_mode;

    pre_x = parameterinfo->X;

    char path[200];
    char line[100];
    fstream fin;

    strcpy(path, tool->parameterPath.c_str());
    if(parameterinfo->walking_mode == 1)
    {
        ROS_INFO("Generate Continuous");
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
    else if(parameterinfo->walking_mode == 2)
    {
        ROS_INFO("Generate LCstep");
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
    else if(parameterinfo->walking_mode == 3)
    {
        ROS_INFO("Generate LCdown");
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
        ROS_INFO("Generate Single_wood");
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
        ROS_INFO("Generate Single_third");
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
    else if(parameterinfo->walking_mode == 7)
    {
        ROS_INFO("Generate Continuous_Second");
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
    else if(parameterinfo->walking_mode == 8)
    {
        ROS_INFO("Generate Continuous_Third");
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
    else if(parameterinfo->walking_mode == 9)
    {
        ROS_INFO("Generate Right_Kick_Ball");
        strcat(path, "/Right_Kick_Ball.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Kick_Point_X = tool->readvalue(fin,"Kick_Point_X",1);
        parameterinfo->parameters.Kick_Point_Y = tool->readvalue(fin,"Kick_Point_Y",1);
        parameterinfo->parameters.Kick_Point_Z = tool->readvalue(fin,"Kick_Point_Z",1);
        parameterinfo->parameters.Back_Point_X = tool->readvalue(fin,"Back_Point_X",1);
        parameterinfo->parameters.Back_Point_Z = tool->readvalue(fin,"Back_Point_Z",1);
        parameterinfo->parameters.Support_Foot_Hip_Upper_Pitch = tool->readvalue(fin,"Support_Foot_Hip_Upper_Pitch",1);
        parameterinfo->parameters.Kick_Foot_Ankle_Upper_Pitch = tool->readvalue(fin,"Kick_Foot_Ankle_Upper_Pitch",1);
        parameterinfo->parameters.Support_Foot_Ankle_Upper_Pitch = tool->readvalue(fin,"Support_Foot_Ankle_Upper_Pitch",1);
    }
    else if(parameterinfo->walking_mode == 10)
    {
        ROS_INFO("Generate Left_Kick_Ball");
        strcat(path, "/Left_Kick_Ball.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Kick_Point_X = tool->readvalue(fin,"Kick_Point_X",1);
        parameterinfo->parameters.Kick_Point_Y = tool->readvalue(fin,"Kick_Point_Y",1);
        parameterinfo->parameters.Kick_Point_Z = tool->readvalue(fin,"Kick_Point_Z",1);
        parameterinfo->parameters.Back_Point_X = tool->readvalue(fin,"Back_Point_X",1);
        parameterinfo->parameters.Back_Point_Z = tool->readvalue(fin,"Back_Point_Z",1);
        parameterinfo->parameters.Support_Foot_Hip_Upper_Pitch = tool->readvalue(fin,"Support_Foot_Hip_Upper_Pitch",1);
        parameterinfo->parameters.Kick_Foot_Ankle_Upper_Pitch = tool->readvalue(fin,"Kick_Foot_Ankle_Upper_Pitch",1);
        parameterinfo->parameters.Support_Foot_Ankle_Upper_Pitch = tool->readvalue(fin,"Support_Foot_Ankle_Upper_Pitch",1);
    }
    else
    {
        ROS_INFO("Generate Single");
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
    fin.close();

    if(parameterinfo->complan.walking_state == StopStep)
    {
        parameterinfo->complan.walking_state = StartStep;
        parameterinfo->complan.walking_stop = false;
        parameterinfo->WalkFlag = true;
        parameterinfo->counter = 0;
    }
    else
    {
        parameterinfo->complan.walking_state = StopStep;
        parameterinfo->WalkFlag = false;
    }

    parameterinfo->cpgack = true;
}

void WalkingGait::saveParameter(const tku_msgs::parameter& msg)
{
    char path[200];
    fstream fp;
    string savedText;

    strcpy(path, tool->parameterPath.c_str());
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
            if(continuousBackFlag)
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
            if(continuousBackFlag)
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
            if(continuousBackFlag)
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
        case 9:
            strcat(path, "/Right_Kick_Ball.ini");
            fp.open(path, ios::out);
            savedText = "[General]\n";
            fp<<savedText;
            savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
            fp<<savedText;
            savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
            fp<<savedText;
            savedText = "Kick_Point_X = " + DtoS(msg.B_SplineParam.Kick_Point_X) + "\n";
            fp<<savedText;
            savedText = "Kick_Point_Y = " + DtoS(msg.B_SplineParam.Kick_Point_Y) + "\n";
            fp<<savedText;
            savedText = "Kick_Point_Z = " + DtoS(msg.B_SplineParam.Kick_Point_Z) + "\n";
            fp<<savedText;
            savedText = "Back_Point_X = " + DtoS(msg.B_SplineParam.Back_Point_X) + "\n";
            fp<<savedText;
            savedText = "Back_Point_Z = " + DtoS(msg.B_SplineParam.Back_Point_Z) + "\n";
            fp<<savedText;
            savedText = "Support_Foot_Hip_Upper_Pitch = " + DtoS(msg.B_SplineParam.Support_Foot_Hip_Upper_Pitch) + "\n";
            fp<<savedText;
            savedText = "Kick_Foot_Ankle_Upper_Pitch = " + DtoS(msg.B_SplineParam.Kick_Foot_Ankle_Upper_Pitch) + "\n";
            fp<<savedText;
            savedText = "Support_Foot_Ankle_Upper_Pitch = " + DtoS(msg.B_SplineParam.Support_Foot_Ankle_Upper_Pitch) + "\n";
            fp<<savedText;
            break;
        case 10:
            strcat(path, "/Left_Kick_Ball.ini");
            fp.open(path, ios::out);
            savedText = "[General]\n";
            fp<<savedText;
            savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
            fp<<savedText;
            savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
            fp<<savedText;
            savedText = "Kick_Point_X = " + DtoS(msg.B_SplineParam.Kick_Point_X) + "\n";
            fp<<savedText;
            savedText = "Kick_Point_Y = " + DtoS(msg.B_SplineParam.Kick_Point_Y) + "\n";
            fp<<savedText;
            savedText = "Kick_Point_Z = " + DtoS(msg.B_SplineParam.Kick_Point_Z) + "\n";
            fp<<savedText;
            savedText = "Back_Point_X = " + DtoS(msg.B_SplineParam.Back_Point_X) + "\n";
            fp<<savedText;
            savedText = "Back_Point_Z = " + DtoS(msg.B_SplineParam.Back_Point_Z) + "\n";
            fp<<savedText;
            savedText = "Support_Foot_Hip_Upper_Pitch = " + DtoS(msg.B_SplineParam.Support_Foot_Hip_Upper_Pitch) + "\n";
            fp<<savedText;
            savedText = "Kick_Foot_Ankle_Upper_Pitch = " + DtoS(msg.B_SplineParam.Kick_Foot_Ankle_Upper_Pitch) + "\n";
            fp<<savedText;
            savedText = "Support_Foot_Ankle_Upper_Pitch = " + DtoS(msg.B_SplineParam.Support_Foot_Ankle_Upper_Pitch) + "\n";
            fp<<savedText;
            break;
    }
    fp.close();
}

bool WalkingGait::loadWalkingGaitParameterFunction(tku_msgs::WalkingGaitParameter::Request &req, tku_msgs::WalkingGaitParameter::Response &res)
{
    char path[200];
    char line[100];
    fstream fin;

    strcpy(path, tool->parameterPath.c_str());
    switch(req.mode)
    {
        case 4:
        case 0:
            ROS_INFO("Load Single_Parameter");
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
            if(continuousBackFlag)
            {
                ROS_INFO("Load Continuous_Back");
                strcat(path, "/Continuous_Back.ini");
            }
            else
            {
                ROS_INFO("Load Continuous_Parameter");
                strcat(path, "/Continuous_Parameter.ini");
            }
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
            if(continuousBackFlag)
            {
                ROS_INFO("Load Continuous_Second_Back");
                strcat(path, "/Continuous_Second_Back.ini");
            }
            else
            {
                ROS_INFO("Load Continuous_Second_Parameter");
                strcat(path, "/Continuous_Second_Parameter.ini");
            }
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
            if(continuousBackFlag)
            {
                ROS_INFO("Load Continuous_Third_Back");
                strcat(path, "/Continuous_Third_Back.ini");
            }
            else
            {
                ROS_INFO("Load Continuous_Third_Parameter");
                strcat(path, "/Continuous_Third_Parameter.ini");
            }
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
            ROS_INFO("Load LCstep_Parameter");
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
            ROS_INFO("Load LCdown_Parameter");
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
            ROS_INFO("Load Single_wood");
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
            ROS_INFO("Load Single_third");
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
        case 9:
            ROS_INFO("Load Right_Kick_Ball");
            strcat(path, "/Right_Kick_Ball.ini");
            fin.open(path,ios::in);
            fin.getline(line,sizeof(line),'\n');

            res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
            res.Period_T = tool->readvalue(fin,"Period_T",0);
            res.B_SplineParam.Kick_Point_X = tool->readvalue(fin,"Kick_Point_X",1);
            res.B_SplineParam.Kick_Point_Y = tool->readvalue(fin,"Kick_Point_Y",1);
            res.B_SplineParam.Kick_Point_Z = tool->readvalue(fin,"Kick_Point_Z",1);
            res.B_SplineParam.Back_Point_X = tool->readvalue(fin,"Back_Point_X",1);
            res.B_SplineParam.Back_Point_Z = tool->readvalue(fin,"Back_Point_Z",1);
            res.B_SplineParam.Support_Foot_Hip_Upper_Pitch = tool->readvalue(fin,"Support_Foot_Hip_Upper_Pitch",1);
            res.B_SplineParam.Kick_Foot_Ankle_Upper_Pitch = tool->readvalue(fin,"Kick_Foot_Ankle_Upper_Pitch",1);
            res.B_SplineParam.Support_Foot_Ankle_Upper_Pitch = tool->readvalue(fin,"Support_Foot_Ankle_Upper_Pitch",1);
            break;
        case 10:
            ROS_INFO("Load Left_Kick_Ball");
            strcat(path, "/Left_Kick_Ball.ini");
            fin.open(path,ios::in);
            fin.getline(line,sizeof(line),'\n');

            res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
            res.Period_T = tool->readvalue(fin,"Period_T",0);
            res.B_SplineParam.Kick_Point_X = tool->readvalue(fin,"Kick_Point_X",1);
            res.B_SplineParam.Kick_Point_Y = tool->readvalue(fin,"Kick_Point_Y",1);
            res.B_SplineParam.Kick_Point_Z = tool->readvalue(fin,"Kick_Point_Z",1);
            res.B_SplineParam.Back_Point_X = tool->readvalue(fin,"Back_Point_X",1);
            res.B_SplineParam.Back_Point_Z = tool->readvalue(fin,"Back_Point_Z",1);
            res.B_SplineParam.Support_Foot_Hip_Upper_Pitch = tool->readvalue(fin,"Support_Foot_Hip_Upper_Pitch",1);
            res.B_SplineParam.Kick_Foot_Ankle_Upper_Pitch = tool->readvalue(fin,"Kick_Foot_Ankle_Upper_Pitch",1);
            res.B_SplineParam.Support_Foot_Ankle_Upper_Pitch = tool->readvalue(fin,"Support_Foot_Ankle_Upper_Pitch",1);
            break;
    }
    fin.close();

    return true;
}

string WalkingGait::DtoS(double value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();

    return str;
}

void WalkingGait::timerCallBack(const ros::TimerEvent&)
{
    if(!parameterinfo->complan.walking_stop)
    {
        computerWalkingGaitFunction();
        walkingData_pub.publish(walkingData);
    }
}

void WalkingGait::computerWalkingGaitFunction()
{
    switch(parameterinfo->walking_mode)
    {
        case 0://Single Step
        case 2://LC Up
        case 3://LC Down
        case 4://Long Jump
        case 5://Single Wood
        case 6://Single Third
            walkingCycle.walkingKindFunction(parameterinfo->walking_mode);
            walkingTrajectory.walkingProcess(parameterinfo->walking_mode);
            break;
        case 1://Continuous
        case 7://Continuous Second
        case 8://Continuous Third
            walkingGaitByLIPM.process();
            break;
        case 9://Right Kick Ball
        case 10://Left Kick Ball
            // kickinggait.kickingCycle(parameterinfo->walking_mode);
            return;
            break;
        default:
            break;
    }

    walkingData.IK_Point_RX = parameterinfo->points.IK_Point_RX;
    walkingData.IK_Point_RY = parameterinfo->points.IK_Point_RY;
    walkingData.IK_Point_RZ = parameterinfo->points.IK_Point_RZ;
    walkingData.IK_Point_RThta = parameterinfo->points.IK_Point_RTheta;
    walkingData.IK_Point_LX = parameterinfo->points.IK_Point_LX;
    walkingData.IK_Point_LY = parameterinfo->points.IK_Point_LY;
    walkingData.IK_Point_LZ  = parameterinfo->points.IK_Point_LZ;
    walkingData.IK_Point_LThta = parameterinfo->points.IK_Point_LTheta;
    walkingData.Sampletime = parameterinfo->parameters.Sample_Time;
    walkingData.Period_T = parameterinfo->parameters.Period_T;

    walkingData.Walking_State = parameterinfo->walking_state;
    walkingData.Sensor_Mode = parameterinfo->sensor_mode;

    parameterinfo->CPGalready = true;

    if(parameterinfo->complan.walking_stop)
    {
        saveData.data = true;
        save_pub.publish(saveData);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "WalkingGait");

    //----------------- nh -----------------//
    ros::NodeHandle nh;

    ros::AsyncSpinner s(4);
    s.start();

    //------------- nhPrivate -------------//
    ros::NodeHandle nhPrivate("~");

    ros::CallbackQueue my_queue;
    nhPrivate.setCallbackQueue(&my_queue);

    ros::AsyncSpinner spinner(2, &my_queue);
    spinner.start();

    WalkingGait walkingGait(nh, nhPrivate);
    
    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        loop_rate.sleep();
    }

    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    return 0;
}
