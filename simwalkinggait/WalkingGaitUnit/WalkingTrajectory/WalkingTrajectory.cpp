#include "walkinggait/WalkingTrajectory.hpp"

WalkingTrajectory::WalkingTrajectory()
{
    tool = ToolInstance::getInstance();

    osc_move_x_r.clear();
    osc_move_x_l.clear();
    osc_move_y_r.clear();
    osc_move_y_l.clear();
    osc_move_z_r.clear();
    osc_move_z_l.clear();
    osc_move_com_x.clear();
    osc_move_com_y.clear();
    osc_move_com_z.clear();
    right_Theta.clear();
    left_Theta.clear();
    test.clear();

    wosc_foot_x_r = 0.0;
    wosc_foot_x_l = 0.0;
    wosc_foot_x_last = 0.0;
    wosc_foot_x_now = 0.0;
}

WalkingTrajectory::~WalkingTrajectory()
{

}

void WalkingTrajectory::walkingProcess(int walking_mode)
{
    switch(walking_mode)
    {
        case 0://Single Step
        case 5://Single Wood
        case 6://Single Third
            walkFunction();
            break;
        case 1://Continuous
        case 7://Continuous Second
        case 8://Continuous Third
            continuousWalk();
            break;
        case 2://LC Up
        case 3://LC Down
            LCWalkFunction();
            break;
        case 4://Long Jump
            longJumpFunction();
            break;
        default:
            break;
    }
    inverseKinematicsInfo();
}

void WalkingTrajectory::walkFunction()
{
    int time_shift = 0;
    int time_point_ = parameterinfo->complan.time_point_;

    time_shift =  parameterinfo->complan.time_point_ + 0.5 * parameterinfo->parameters.Period_T;

    int moving_mode;
    if(parameterinfo->YUpdate > 0)
    {
        if(parameterinfo->THETAUpdate == 0)
        {
            moving_mode = Left_Shift;
        }
        else
        {
            moving_mode = Left_Protect;
        }
    }
    else if(parameterinfo->YUpdate < 0)
    {
        if(parameterinfo->THETAUpdate == 0)
        {
            moving_mode = Right_Shift;
        }
        else
        {
            moving_mode = Right_Protect;
        }
    }
    else
    {
        if(parameterinfo->THETAUpdate > 0)
        {
            moving_mode = Left_Turn;
        }
        else if (parameterinfo->THETAUpdate < 0)
        {
            moving_mode = Right_turn;
        }
        else
        {
            moving_mode = Straight;
        }
    }

    switch (moving_mode)
    {
        case Right_turn:
            parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_)
                    + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);
            parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift)
                    + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);
            parameterinfo->points.Y_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);
            parameterinfo->points.Y_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);
            parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
            parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);

            parameterinfo->points.X_COM = OSC_COM_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, 0, time_point_);
            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T * 1/4);
            parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2 * 1/4);

            if(parameterinfo->complan.isfirststep || parameterinfo->complan.islaststep)
            {
                parameterinfo->points.Right_Theta = 0;
                parameterinfo->points.Left_Theta = 0;
            }
            else
            {
                parameterinfo->points.Right_Theta =  OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.3 * parameterinfo->THETAUpdate * -1, 0, time_shift); //Swing leg turn right
                parameterinfo->points.Left_Theta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.7 * parameterinfo->THETAUpdate, 0, time_shift);
            }
            break;
        case Right_Protect:
            parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_)
                    + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);
            parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift);
            
            if(parameterinfo->complan.isfirststep)
            {
                parameterinfo->points.Y_Right_foot = 0;
                parameterinfo->points.Y_Left_foot = 0;
            }
            else
            {
                parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_shift);
                parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_shift);

            }
            
            parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
            parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
            
            parameterinfo->points.X_COM = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->parameters.X_Swing_Range, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T2 * 1/4);
            
            if(parameterinfo->complan.isfirststep)
            {
                parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T * 1/4);
            }
            else
            {
                parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range-parameterinfo->YUpdate * 0.25, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T * 1/4);
            }
            
            parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2 * 1/4);
            
            if(parameterinfo->complan.isfirststep)
            {
                parameterinfo->points.Right_Theta = 0;
                parameterinfo->points.Left_Theta = 0;
            }
            else if(parameterinfo->THETAUpdate > 0) //Turn Left
            {
                parameterinfo->points.Right_Theta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.7 * parameterinfo->THETAUpdate * -1, 0, time_shift);
                parameterinfo->points.Left_Theta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.3 * parameterinfo->THETAUpdate, 0, time_shift); //Swing leg turn left;
            }
            else if(parameterinfo->THETAUpdate < 0) //Turn Right
            {
                parameterinfo->points.Right_Theta =  OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.3 * parameterinfo->THETAUpdate * -1, 0, time_shift); //Swing leg turn right
                parameterinfo->points.Left_Theta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.7 * parameterinfo->THETAUpdate, 0, time_shift);
            }
            else
            {
                parameterinfo->points.Right_Theta = 0;
                parameterinfo->points.Left_Theta = 0;
            }
            break;
        case Left_Turn:
            parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift)
                    + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);
            parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_)
                    + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);
            parameterinfo->points.Y_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);
            parameterinfo->points.Y_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);
            parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
            parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
            
            parameterinfo->points.X_COM = OSC_COM_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, 0, time_point_);
            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_shift + parameterinfo->parameters.Period_T * 1/4);
            parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2 * 1/4);
            
            if(parameterinfo->complan.isfirststep)
            {
                parameterinfo->points.Right_Theta = 0;
                parameterinfo->points.Left_Theta = 0;
            }
            else
            {
                parameterinfo->points.Right_Theta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.7 * parameterinfo->THETAUpdate, 0, time_shift);
                parameterinfo->points.Left_Theta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.3 * parameterinfo->THETAUpdate * -1, 0, time_shift); //Swing leg turn left;
            }
            break;
        case Left_Protect:
            parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift)
                    + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);
            parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_)
                    + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);
            
            if(parameterinfo->complan.isfirststep)
            {
                parameterinfo->points.Y_Right_foot = 0;
                parameterinfo->points.Y_Left_foot = 0;
            }
            else
            {
                parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_shift);
                parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_shift);
            }
            
            parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
            parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
            
            parameterinfo->points.X_COM = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->parameters.X_Swing_Range, 0, PI_2, time_shift + parameterinfo->parameters.Period_T2 * 1/4);
            
            if(parameterinfo->complan.isfirststep)
            {
                parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_shift + parameterinfo->parameters.Period_T * 1/4);
            }
            else
            {
                parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range+parameterinfo->YUpdate * 0.25, 0, PI_2, time_shift + parameterinfo->parameters.Period_T * 1/4);
            }
            
            parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2 * 1/4);
            
            if(parameterinfo->complan.isfirststep)
            {
                parameterinfo->points.Right_Theta = 0;
                parameterinfo->points.Left_Theta = 0;
            }
            else if(parameterinfo->THETAUpdate > 0) //Turn Left
            {
                parameterinfo->points.Right_Theta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.7 * parameterinfo->THETAUpdate, 0, time_shift);
                parameterinfo->points.Left_Theta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.3 * parameterinfo->THETAUpdate * -1, 0, time_shift); //Swing leg turn left;
            }
            else if(parameterinfo->THETAUpdate < 0) //Turn Right
            {

                parameterinfo->points.Right_Theta =  OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.3 * parameterinfo->THETAUpdate, 0, time_shift); //Swing leg turn right
                parameterinfo->points.Left_Theta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.7 * parameterinfo->THETAUpdate * -1, 0, time_shift);
            }
            else
            {
                parameterinfo->points.Right_Theta = 0;
                parameterinfo->points.Left_Theta = 0;
            }
            break;
        case Right_Shift:
            parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_)
                    + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);
            parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T,parameterinfo->XUpdate, 0, PI_2, time_shift)
                    + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);
            
            if(parameterinfo->complan.isfirststep)
            {
                parameterinfo->points.Y_Right_foot = 0;
                parameterinfo->points.Y_Left_foot = 0;
            }
            else
            {
                parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_shift);
                parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_shift);
            }
            
            parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
            parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);

            parameterinfo->points.X_COM = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->parameters.X_Swing_Range, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T2 * 1/4);
            
            if(parameterinfo->complan.isfirststep)
            {
                parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T * 1/4);
            }
            else
            {
                parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range-parameterinfo->YUpdate * 0.25, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T * 1/4);
            }
            
            parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2 * 1/4);

            if(parameterinfo->complan.isfirststep)
            {
                parameterinfo->points.Right_Theta = 0;
                parameterinfo->points.Left_Theta = 0;
            }
            else
            {
                parameterinfo->points.Right_Theta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, -0/180*PI, 0, time_point_); //Swing leg turn right
                parameterinfo->points.Left_Theta = 0;
            }
            break;
        case Left_Shift:
            parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift)
                    + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);
            parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_)
                    + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);
            
            if(parameterinfo->complan.isfirststep)
            {
                parameterinfo->points.Y_Right_foot = 0;
                parameterinfo->points.Y_Left_foot = 0;
            }
            else
            {
                parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_shift);
                parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_shift);
            }
            
            parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
            parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
            
            parameterinfo->points.X_COM = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->parameters.X_Swing_Range, 0, PI_2, time_shift + parameterinfo->parameters.Period_T2 * 1/4);
            
            if(parameterinfo->complan.isfirststep)
            {
                parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_shift + parameterinfo->parameters.Period_T * 1/4);
            }
            else
            {
                parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range+parameterinfo->YUpdate * 0.25, 0, PI_2, time_shift + parameterinfo->parameters.Period_T * 1/4);
            }
            
            parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2 * 1/4);

            if(parameterinfo->complan.isfirststep)
            {
                parameterinfo->points.Right_Theta = 0;
                parameterinfo->points.Left_Theta = 0;
            }
            else
            {
                parameterinfo->points.Right_Theta = 0;
                parameterinfo->points.Left_Theta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0/180*PI, PI, time_shift);
            }
            break;
        case Straight:
            parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange , parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift)
                    + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);
            parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_)
                    + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);
            parameterinfo->points.Y_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);
            parameterinfo->points.Y_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);
            parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
            parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
            
            if(parameterinfo->complan.islaststep)
            {
                parameterinfo->points.X_COM = 0;
            }
            else
            {
                parameterinfo->points.X_COM = OSC_COM_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, 0, time_point_);
            }

            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_shift+parameterinfo->parameters.Period_T * 1/4);
            parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2 * 1/4);
            
            if(parameterinfo->complan.islaststep)
            {
                parameterinfo->points.Right_Theta = 0;
                parameterinfo->points.Left_Theta = 0;
            }
            else
            {
                parameterinfo->points.Right_Theta = 0;
                parameterinfo->points.Left_Theta  = 0;
            }
            break;
        default:
            parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift);
            parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_);
            parameterinfo->points.Y_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);
            parameterinfo->points.Y_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);
            parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
            parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
            
            parameterinfo->points.X_COM = OSC_COM_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, 0, time_point_);
            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_point_+parameterinfo->parameters.Period_T * 1/4);
            parameterinfo->points.Z_COM = 0;
            
            parameterinfo->points.Right_Theta = 0;
            parameterinfo->points.Left_Theta = 0;
            break;
    }

    // osc_move_x_r.push_back(parameterinfo->points.X_Right_foot);
    // osc_move_x_l.push_back(parameterinfo->points.X_Left_foot);
    // osc_move_y_r.push_back(parameterinfo->points.Y_Right_foot);
    // osc_move_y_l.push_back(parameterinfo->points.Y_Left_foot);
    // osc_move_z_r.push_back(parameterinfo->points.Z_Right_foot);
    // osc_move_z_l.push_back(parameterinfo->points.Z_Left_foot);
    // osc_move_com_x.push_back(parameterinfo->points.X_COM);
    // osc_move_com_y.push_back(parameterinfo->points.Y_COM);
    // osc_move_com_z.push_back(parameterinfo->points.Z_COM);
    // right_Theta.push_back(parameterinfo->points.Right_Theta);
    // left_Theta.push_back(parameterinfo->points.Left_Theta);
    // test.push_back(parameterinfo->counter);

    // if(parameterinfo->complan.walking_stop)
    // {
    //     SaveData();
    // }
}


void WalkingTrajectory::continuousWalk()
{
    int time_shift = 0;
    int time_point_ = parameterinfo->complan.time_point_;

    time_shift =  parameterinfo->complan.time_point_ + 0.5 * parameterinfo->parameters.Period_T;

    parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift);
    parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_);
    parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
    parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);

    parameterinfo->points.X_COM = OSC_COM_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, 0, time_point_);
    parameterinfo->points.Y_COM = OSC_COM_Y(parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, time_point_);
    parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_COM_Z(parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0, time_point_);
    
    //Turn
    if(parameterinfo->THETAUpdate > 0) //Turn Left
    {
        parameterinfo->points.Right_Theta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THETAUpdate, 0, time_shift);
        parameterinfo->points.Left_Theta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THETAUpdate * -1, 0, time_shift); //Swing leg turn left

    }
    else if(parameterinfo->THETAUpdate < 0) //Turn Right
    {
        parameterinfo->points.Right_Theta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THETAUpdate * -1, 0, time_point_); //Swing leg turn right
        parameterinfo->points.Left_Theta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THETAUpdate, 0, time_point_);
    }
    else
    {
        parameterinfo->points.Right_Theta = 0;
        parameterinfo->points.Left_Theta = 0;
    }

    //Shift
    if(parameterinfo->complan.isfirststep)
    {
        parameterinfo->points.Y_Right_foot = 0;
        parameterinfo->points.Y_Left_foot = 0;
    }
    else if(parameterinfo->YUpdate < 0 && !parameterinfo->Isfrist_right_shift)
    {
        parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_point_);
        parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_point_);
        parameterinfo->points.Right_Theta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0*PI/180, 0, time_shift); //Turn fix for shift
        parameterinfo->points.Left_Theta = 0;
    }
    else if(parameterinfo->YUpdate > 0)
    {
        parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_shift);
        parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_shift);
        parameterinfo->points.Right_Theta = 0;
        parameterinfo->points.Left_Theta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0*PI/180, 0, time_point_); //Turn fix for shift
    }

    // osc_move_x_r.push_back(parameterinfo->points.X_Right_foot);
    // osc_move_x_l.push_back(parameterinfo->points.X_Left_foot);
    // osc_move_y_r.push_back(parameterinfo->points.Y_Right_foot);
    // osc_move_y_l.push_back(parameterinfo->points.Y_Left_foot);
    // osc_move_z_r.push_back(parameterinfo->points.Z_Right_foot);
    // osc_move_z_l.push_back(parameterinfo->points.Z_Left_foot);
    // osc_move_com_x.push_back(parameterinfo->points.X_COM);
    // osc_move_com_y.push_back(parameterinfo->points.Y_COM);
    // osc_move_com_z.push_back(parameterinfo->points.Z_COM);
    // right_Theta.push_back(parameterinfo->points.Right_Theta);
    // left_Theta.push_back(parameterinfo->points.Left_Theta);
    // test.push_back(parameterinfo->counter);

    // if(parameterinfo->complan.walking_stop)
    // {
    //     SaveData();
    // }
}

void WalkingTrajectory::LCWalkFunction()
{
    int time_shift = 0;

    time_shift =  parameterinfo->complan.time_point_ + 0.5 * parameterinfo->parameters.Period_T;

    parameterinfo->points.Y_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, parameterinfo->complan.time_point_);
    parameterinfo->points.Y_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);
    
    if(parameterinfo->complan.lift_lock_x != 0)
    {
        parameterinfo->points.X_Right_foot = (OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate ,0 , PI_2, time_shift) + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift));
        if(parameterinfo->points.X_Right_foot > parameterinfo->complan.lift_lock_x)
        {
            parameterinfo->points.X_Right_foot = parameterinfo->complan.lift_lock_x;
        }
        else
        {
            parameterinfo->points.X_Left_foot = (OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate ,0 , PI_2, parameterinfo->complan.time_point_) + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, parameterinfo->complan.time_point_));
            parameterinfo->complan.lift_lock_x = 0;
        }
    }
    else
    {
        parameterinfo->points.X_Right_foot = (OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate ,0 , PI_2, time_shift) + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift));
        parameterinfo->points.X_Left_foot = (OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate ,0 , PI_2, parameterinfo->complan.time_point_) + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, parameterinfo->complan.time_point_));
        parameterinfo->complan.lift_lock_x = parameterinfo->points.X_Right_foot;
    }

    if(parameterinfo->parameters.BASE_LIFT_Z > 0)
    {
        parameterinfo->points.Z_Right_foot = OSC_lifemove_UPz(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, parameterinfo->parameters.BASE_LIFT_Z, parameterinfo->complan.time_point_-parameterinfo->parameters.Period_T/parameterinfo->parameters.Sample_Time * 2, parameterinfo->parameters.Sample_Time);
        parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, parameterinfo->complan.time_point_+parameterinfo->parameters.Period_T/parameterinfo->parameters.Sample_Time * 2);
    }
    else if(parameterinfo->parameters.BASE_LIFT_Z < 0)
    {
        parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift-parameterinfo->parameters.Period_T/parameterinfo->parameters.Sample_Time * 3);
        parameterinfo->points.Z_Left_foot = OSC_lifemove_DOWNz(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, parameterinfo->parameters.BASE_LIFT_Z, parameterinfo->complan.time_point_-parameterinfo->parameters.Period_T/parameterinfo->parameters.Sample_Time * 3, parameterinfo->parameters.Sample_Time);
    }

    if(parameterinfo->complan.islaststep)
    {
        parameterinfo->points.X_COM = 0;
    }
    else
    {
        parameterinfo->points.X_COM = OSC_COM_Lift_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, parameterinfo->parameters.X_Swing_COM, parameterinfo->complan.time_point_);
    }

    parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_shift+parameterinfo->parameters.Period_T * 1/4);
    parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, parameterinfo->complan.time_point_);

    if(parameterinfo->complan.islaststep)
    {
        parameterinfo->points.Right_Theta = 0;
        parameterinfo->points.Left_Theta = 0;
    }
    else
    {
        parameterinfo->points.Right_Theta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, time_shift);
        parameterinfo->points.Left_Theta  += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, parameterinfo->complan.time_point_);
    }

    // osc_move_x_r.push_back(parameterinfo->points.X_Right_foot);
    // osc_move_x_l.push_back(parameterinfo->points.X_Left_foot);
    // osc_move_y_r.push_back(parameterinfo->points.Y_Right_foot);
    // osc_move_y_l.push_back(parameterinfo->points.Y_Left_foot);
    // osc_move_z_r.push_back(parameterinfo->points.Z_Right_foot);
    // osc_move_z_l.push_back(parameterinfo->points.Z_Left_foot);
    // osc_move_com_x.push_back(parameterinfo->points.X_COM);
    // osc_move_com_y.push_back(parameterinfo->points.Y_COM);
    // osc_move_com_z.push_back(parameterinfo->points.Z_COM);
    // right_Theta.push_back(parameterinfo->points.Right_Theta);
    // left_Theta.push_back(parameterinfo->points.Left_Theta);
    // test.push_back(parameterinfo->counter);

    // if(parameterinfo->complan.walking_stop)
    // {
    //     SaveData();
    // }
}

void WalkingTrajectory::longJumpFunction()
{
    int time_shift = 0;
    int time_point_ = parameterinfo->complan.time_point_;

    time_shift =  parameterinfo->complan.time_point_ + 0.5 * parameterinfo->parameters.Period_T;

    parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange , parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_);
    parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_);
    parameterinfo->points.Y_Right_foot = 0.0;
    parameterinfo->points.Y_Left_foot = 0.0;
    parameterinfo->points.Z_Right_foot = -OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
    parameterinfo->points.Z_Left_foot = -OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
    
    if (parameterinfo->complan.islaststep)
    {
        parameterinfo->points.X_COM = 0;
    }
    else
    {
        parameterinfo->points.X_COM = OSC_COM_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, 0, time_point_);
    }
    
    parameterinfo->points.Y_COM = 0.0;
    parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2 * 1/4);
    
    if (parameterinfo->complan.islaststep)
    {
        parameterinfo->points.Right_Theta = 0;
        parameterinfo->points.Left_Theta = 0;
    }
    else
    {
        parameterinfo->points.Right_Theta = 0;
        parameterinfo->points.Left_Theta  = 0;
    }

    // osc_move_x_r.push_back(parameterinfo->points.X_Right_foot);
    // osc_move_x_l.push_back(parameterinfo->points.X_Left_foot);
    // osc_move_y_r.push_back(parameterinfo->points.Y_Right_foot);
    // osc_move_y_l.push_back(parameterinfo->points.Y_Left_foot);
    // osc_move_z_r.push_back(parameterinfo->points.Z_Right_foot);
    // osc_move_z_l.push_back(parameterinfo->points.Z_Left_foot);
    // osc_move_com_x.push_back(parameterinfo->points.X_COM);
    // osc_move_com_y.push_back(parameterinfo->points.Y_COM);
    // osc_move_com_z.push_back(parameterinfo->points.Z_COM);
    // right_Theta.push_back(parameterinfo->points.Right_Theta);
    // left_Theta.push_back(parameterinfo->points.Left_Theta);
    // test.push_back(parameterinfo->counter);

    // if(parameterinfo->complan.walking_stop)
    // {
    //     SaveData();
    // }
}

void WalkingTrajectory::inverseKinematicsInfo()
{
    int time_shift = 0;
    int time_point_ = parameterinfo->complan.time_point_;
    time_shift =  parameterinfo->complan.time_point_ + 0.5 * parameterinfo->parameters.Period_T;

    parameterinfo->points.IK_Point_RX = parameterinfo->points.X_COM + parameterinfo->points.X_Right_foot;
    parameterinfo->points.IK_Point_RY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Right_foot;
    parameterinfo->points.IK_Point_RZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Right_foot;
    parameterinfo->points.IK_Point_RTheta = parameterinfo->points.Right_Theta;
    parameterinfo->points.IK_Point_LX = parameterinfo->points.X_COM + parameterinfo->points.X_Left_foot;
    parameterinfo->points.IK_Point_LY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Left_foot;
    parameterinfo->points.IK_Point_LZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Left_foot;
    parameterinfo->points.IK_Point_LTheta = parameterinfo->points.Left_Theta;
}

double WalkingTrajectory::OSC_COM_Lift_X(double range, double period_T_ms, double rho_com_x, double delta_com_x, int time_t_ms)
{
    double rho_x;
    double period_T = period_T_ms * 0.001;
    double T_divbylock = period_T * (range / 2);
    double T_divbyunlock = period_T * (1-range) / 2;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t =(double)time_t_temp / 1000;

    rho_x = rho_com_x * (range) * delta_com_x;

    if(time_t >= 0 && time_t < T_divbylock)
    {
        return rho_x * sin((2 * PI / (T_divbylock * 4)) * time_t + PI);
    }
    else if(time_t < period_T && time_t > (period_T - T_divbylock))
    {
        return rho_x * sin((1.5*PI) + (2 * PI / (T_divbyunlock * 4)) * (time_t - (period_T - T_divbylock))+PI);
    }
    else
    {
        return rho_x * sin((PI / 2) + (2 * PI / (T_divbyunlock * 4)) * (time_t - T_divbylock) + PI);
    }
}

double WalkingTrajectory::OSC_move_x_advance(double range, double period_T_ms, double rho_x, double BASE_delrho_x, double delta_x, int time_t_ms)
{
    double omega_x;
    double period_T = period_T_ms * 0.001;//sec
    double T_divby2 = period_T / 2;
    double T_divby4 = period_T / 4;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    omega_x = 2 * PI / period_T / (1-range);

    if(time_t >= 0 && time_t < range * T_divby4)
    {
        return rho_x *(1 + BASE_delrho_x);
    }
    else if(time_t >= range * T_divby4 && time_t < (T_divby2 - (range * T_divby4)) )
    {
        if(time_t <= T_divby4)
        {
            return rho_x * (1 + BASE_delrho_x) * sin(omega_x * (time_t - range * T_divby4)+delta_x);
        }
        else
        {
            return rho_x* (1 - BASE_delrho_x) * sin(omega_x * (time_t - range * T_divby4)+delta_x);
        }
    }
    else if(time_t >= T_divby2 - (range * T_divby4) && time_t < T_divby2 + (range * T_divby4))
    {
        return -rho_x * (1 - BASE_delrho_x);
    }
    else if(time_t >= T_divby2 + (range * T_divby4) && time_t < period_T - (range * T_divby4))
    {
        if(time_t <= period_T - T_divby4)
        {
            return rho_x * (1 - BASE_delrho_x) * sin(omega_x * (time_t + (range * T_divby4) - period_T) + delta_x);
        }
        else
        {
            return rho_x * (1 + BASE_delrho_x) * sin(omega_x * (time_t + (range * T_divby4) - period_T) + delta_x);
        }
    }
    else
    {
        return rho_x *(1 + BASE_delrho_x) ;

    }
}

double WalkingTrajectory::OSC_move_z(double range, double period_T_ms, double rho_z, double delta_z, int time_t_ms, int div_omega)
{
    double omega_z;
    double period_T = period_T_ms * 0.001;
    double T_divby2 = period_T / 2;
    double T_divby4 = period_T / 4;
    int time_t_temp = time_t_ms % ( int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    omega_z = 2 * PI / period_T / (1-range);
    omega_z = omega_z / (double)div_omega;

    if(time_t >= 0 && time_t < T_divby2 + range * T_divby4)
    {
        return 0;
    }
    else if(time_t >= T_divby2 + range * T_divby4 && time_t < period_T-(range * T_divby4))
    {
        return rho_z * sin(omega_z * (time_t - range * 3 * T_divby4) + delta_z);
    }
    else
    {
        return 0;
    }
}

double WalkingTrajectory::OSC_COM_X(double range, double period_T_ms, double rho_com_x, double delta_com_x, int time_t_ms)
{
    double rho_x;
    double period_T = period_T_ms * 0.001;
    double T_divbylock = period_T * (range/2);
    double T_divbyunlock = period_T * (1-range)/2;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    rho_x = rho_com_x * (range) * (parameterinfo->parameters.X_Swing_Range);

    if(time_t >= 0 && time_t <  T_divbylock)
    {
        return rho_x * sin((2 * PI / (T_divbylock * 4)) * time_t + PI );
    }
    else if(time_t < period_T && time_t >  (period_T - T_divbylock))
    {
        return rho_x * sin( (1.5 * PI) + (2 * PI / (T_divbylock * 4)) * (time_t - (period_T - T_divbylock))+ PI );
    }
    else
    {
        return rho_x * sin( (PI / 2) + (2 * PI / (T_divbyunlock * 4)) * (time_t - T_divbylock)+ PI );
    }
}

double WalkingTrajectory::OSC_move_shift_y(double range, double period_T_ms, double rho_y, double delta_y, int time_t_ms)
{
    double omega_y;
    double period_T = period_T_ms * 0.001;
    double T_divby2 = period_T / 2;
    double T_divby4 = period_T / 4;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;
    
    // new
    double r1 = T_divby4;
    double r2 = T_divby2 - (range * T_divby4);
    double r3 = T_divby2 + (range * T_divby4);
    double r4 = 3 * T_divby4;
    
    omega_y = 2 * PI / period_T / (1-range);
    
    if(time_t >= 0 && time_t < range * T_divby4)
    {
        return 0;
    }
    else if(time_t >=  range * T_divby4  && time_t < r1)
    {
        return rho_y * sin(omega_y * (time_t - range * T_divby4) + delta_y);
    }
    else if(time_t >= r1 && time_t < r3)
    {
        return rho_y;
    }
    else if(time_t >= r3 && time_t <= r4)
    {
        return rho_y * sin(omega_y * (time_t - 3*T_divby4) + delta_y + PI);
    }
    else
    {
        return 0;
    }
}

double WalkingTrajectory::OSC_COM_Y(double period_T_ms, double rho_com_y, double delta_com_y, int time_t_ms)
{
    double omega_com_y;
    double period_T = period_T_ms * 0.001;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    omega_com_y = 2 * PI / period_T;

    return rho_com_y * sin(omega_com_y * time_t + delta_com_y);
}

double WalkingTrajectory::OSC_COM_Z(double period_T_ms, double rho_com_z, double delta_com_z, int time_t_ms)
{
    double omega_com_z;
    double period_T = period_T_ms * 0.001;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    omega_com_z =  2 * PI / period_T;

    return rho_com_z * sin(omega_com_z * time_t + delta_com_z);
}

double WalkingTrajectory::OSC_Rotate(double range, double period_T_ms, double rho_y, double delta_y, int time_t_ms)
{
    return OSC_move_shift_y(range,  period_T_ms,  rho_y,  delta_y,  time_t_ms);
}

double WalkingTrajectory::OSC_lifemove_DOWNz(double range, double period_T_ms, double rho_z, double delta_z, double life_high, int time_t_ms, int Sample_Time)
{
    double omega_z;
    double outputZ;
    double period_T = period_T_ms * 0.001;
    double T_divby2 = period_T / 2;
    double T_divby4 = period_T / 4;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    if(life_high < 0)
    {
        time_t = abs(time_t -(period_T_ms / 1000));
        parameterinfo->complan.lift_lock_x = 0;
        omega_z = 2 * PI / period_T / (1-range);
    }

    omega_z = 2 * PI / period_T / (1-range);

    if(time_t >= 0 && time_t < range * T_divby4)
    {
        return 0;
    } 
    else if(time_t >= range * T_divby4 && time_t < T_divby2 - range * T_divby4)
    {
        outputZ = rho_z * sin(omega_z * (time_t - range * T_divby4));
        if(time_t >= T_divby4 && outputZ <= abs(life_high))
        {
            return abs(life_high);
        }

        parameterinfo->complan.lift_lock_x = 0;

        return outputZ;
    }
    else if(time_t >= T_divby2 - range * T_divby4 && time_t < T_divby2 + range * T_divby4)
    {
        return abs(life_high);
    }
    else if(time_t >= T_divby2 + range * T_divby4 && time_t < T_divby2 + T_divby4)
    {
        outputZ = rho_z * sin(omega_z * (time_t - range * 3 * T_divby4) + delta_z);
        outputZ = -(outputZ - abs(life_high));

        if(outputZ < abs(life_high) && outputZ >= 0)
        {
            return outputZ;
        }
        else if(outputZ < 0)
        {
            return 0;
        }
        else
        {
            return abs(life_high);
        }
    }
    else
    {
        return 0;
    }

}

double WalkingTrajectory::OSC_lifemove_UPz(double range, double period_T_ms, double rho_z, double delta_z, double life_high, int time_t_ms, int Sample_Time)
{
    double omega_z;
    double outputZ;
    double period_T = period_T_ms * 0.001;
    double T_divby2 = period_T / 2;
    double T_divby4 = period_T / 4;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    if(life_high < 0)
    {
        time_t = abs(time_t -(period_T_ms / 1000));
        parameterinfo->complan.lift_lock_x = 0;
        omega_z = 2 * PI / period_T / (1-range);
    }

    omega_z = 2 * PI / period_T / (1-range);
    
    if(time_t >= 0 && time_t < range * T_divby4)
    {
        return 0;
    } 
    else if(time_t >= range * T_divby4 && time_t < T_divby2 - range * T_divby4)
    {
        outputZ = rho_z * sin(omega_z * (time_t - range * T_divby4));
        if (time_t >= T_divby4 && outputZ <= abs(life_high))
        {
            return abs(life_high);
        }
        
        parameterinfo->complan.lift_lock_x = 0;
        
        return outputZ;
    }
    else if(time_t >= T_divby2 - range * T_divby4 && time_t < T_divby2 + range * T_divby4 - period_T/Sample_Time * 3)
    {
        return abs(life_high);
    }
    else if(time_t >= T_divby2 + range * T_divby4 - period_T/Sample_Time * 3 && time_t < T_divby2 + T_divby4 - period_T/Sample_Time * 3)
    {
        time_t = time_t + period_T/Sample_Time * 3;
        outputZ = rho_z * sin(omega_z * (time_t - range * 3 * T_divby4) + delta_z);
        outputZ = -(outputZ - abs(life_high));
        
        if (outputZ < abs(life_high) && outputZ >= 0)
        {
            return outputZ;
        }
        else if (outputZ < 0)
        {
            return 0;
        }
        else
        {
            return abs(life_high);
        }
    }
    else
    {
        return 0;
    }

}

void WalkingTrajectory::SaveData()
{
    string savedText = "R_move_X\tL_move_X\t"
                       "R_move_Y\tL_move_Y\t"
                       "R_move_Z\tL_move_Z\t"
                       "move_COM_X\tmove_COM_Y\tmove_COM_Z\t"
                       "R_Theta\tL_Theta\tPoint\n";
    char path[200];
    strcpy(path, tool->parameterPath.c_str());
    strcat(path, "/Trajectory_Record.ods");

    fstream fp;
    fp.open(path, ios::out);

    fp<<savedText;

    for(int i = 0; i < test.size(); i++)
    {
        savedText = DtoS(osc_move_x_r[i]) + "\t"
                + DtoS(osc_move_x_l[i]) + "\t"
                + DtoS(osc_move_y_r[i]) + "\t"
                + DtoS(osc_move_y_l[i]) + "\t"
                + DtoS(osc_move_z_r[i]) + "\t"
                + DtoS(osc_move_z_l[i]) + "\t"
                + DtoS(osc_move_com_x[i]) + "\t"
                + DtoS(osc_move_com_y[i]) + "\t"
                + DtoS(osc_move_com_z[i]) + "\t"
                + DtoS(right_Theta[i]) + "\t"
                + DtoS(left_Theta[i]) + "\t"
                + DtoS(test[i]) + "\n";
        fp<<savedText;
    }

    osc_move_x_r.clear();
    osc_move_x_l.clear();
    osc_move_y_r.clear();
    osc_move_y_l.clear();
    osc_move_z_r.clear();
    osc_move_z_l.clear();
    osc_move_com_x.clear();
    osc_move_com_y.clear();
    osc_move_com_z.clear();
    right_Theta.clear();
    left_Theta.clear();
    test.clear();
    fp.close();
}

string WalkingTrajectory::DtoS(double value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();

    return str;
}
