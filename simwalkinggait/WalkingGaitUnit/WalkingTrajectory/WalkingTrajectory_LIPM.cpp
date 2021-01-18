/**
 * @file  src/qt_ros_pkg/ClassEample/classtestbb.cpp
 *
 * @brief Class Task_B
 *
 * @date 2015/10/16
 *
 * @author shengru
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "walkinggait/WalkingTrajectory.hpp"
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/
WalkingTrajectory::WalkingTrajectory()
{
    osc_move_x_r.clear();
    osc_move_x_l.clear();
    osc_move_y_r.clear();
    osc_move_y_l.clear();
    osc_move_z_r.clear();
    osc_move_z_l.clear();
    osc_move_com_x.clear();
    osc_move_com_y.clear();
    osc_move_com_z.clear();
    right_Thta.clear();
    left_Thta.clear();
    test.clear();
    wosc_foot_x_r = 0;
    wosc_foot_x_l = 0;
    wosc_foot_x_last = 0;
    wosc_foot_x_now = 0;
}

WalkingTrajectory::~WalkingTrajectory()
{

}

void WalkingTrajectory::walkingprocess(int walking_mode)
{
//-----------------------------------------------------------
    Wperiod_T = parameterinfo->parameters.Period_T/2;
    Wsample_point++;
    Wtime_point = Wsample_point * Wsample_time;
    //---------
    TT = (double)Wperiod_T/1000;
    t = (double)((Wtime_point-(int)Wsample_time)%Wperiod_T+Wsample_time)/1000;
    qq = -Wlength/2;
    now_step = (Wsample_point-1)/(int)(Wperiod_T/Wsample_time);
    if (Wmode == 1)
    {
        WTc = sqrt(parameterinfo->parameters.COM_Height/Wg);
    }
    else if(Wmode == 2)
    {
        WTc = sqrt((WM*parameterinfo->parameters.COM_Height+WM*Wl+Wm*parameterinfo->parameters.COM_Height/4)/((WM+Wm/2)*Wg));
    }
    else if(Wmode == 3)
    {
        WTc = sqrt((WM*parameterinfo->parameters.COM_Height+WM*Wl+Wmc*parameterinfo->parameters.COM_Height/16+9*Wmt*parameterinfo->parameters.COM_Height/16)/((WM+Wmc/4+3*Wmt/4)*Wg));
    }
    else if(Wmode == 4)
    {
        dz = 0.5*COM_rho_Z*(2*PI/TT)*(sin(2*t*PI/TT));
        ddz = 0.5*COM_rho_Z*(2*PI/TT)*(2*PI/TT)*(cos(2*t*PI/TT));
        WTc = sqrt((WM*parameterinfo->parameters.COM_Height+WM*Wl+Wmc*parameterinfo->parameters.COM_Height/16+9*Wmt*parameterinfo->parameters.COM_Height/16)/((WM+Wmc/4+3*Wmt/4)*Wg+(WM+Wmc/16+9*Wmt/16)*ddz));
        C = sqrt((WM*parameterinfo->parameters.COM_Height+WM*Wl+Wmc*parameterinfo->parameters.COM_Height/16+9*Wmt*parameterinfo->parameters.COM_Height/16));  
    }
//------------------------------------------------
    switch(walking_mode)
    {
    case 5:
    case 0://Single Step
        walkfunction();
        break;
    case 1://Continuous
        //osccontinuouswalk();
        continuouswalk();
        break;
    case 2:
    case 3:
        LC_walkfunction();
        break;
    case 4:
        longjump_function();
        break;
    default:
        break;
    }
    inversekinmaticsinfo();
}

void WalkingTrajectory::walkfunction()
{
    int time_shift = 0;
    int time_point_ = parameterinfo->complan.time_point_;

    //    double open_value = sin(Parameters->Open_Phase);
    time_shift =  parameterinfo->complan.time_point_ + 0.5 * parameterinfo->parameters.Period_T;
    //    double Open_value_R = Parameters->R_Open;
    //    double Open_value_L = Parameters->L_Open;
    // ROS_INFO("%d", parameterinfo->complan.time_point_);
    /************************************************************************/
    /* Set flags of moving cases                                            */
    /************************************************************************/
    int moving_mode;
    if (parameterinfo->YUpdate > 0)
    {
        if (parameterinfo->THTAUpdate == 0)
        {
            moving_mode = Left_Shift;
        }
        else
        {
            moving_mode = Left_Protect;
        }
    }
    else if (parameterinfo->YUpdate < 0)
    {
        if (parameterinfo->THTAUpdate == 0)
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
        if (parameterinfo->THTAUpdate > 0)
        {
            moving_mode = Left_Turn;
        }
        else if (parameterinfo->THTAUpdate < 0)
        {
            moving_mode = Right_turn;
        }
        else
        {
            moving_mode = Straight;
        }
    }
    ///************************************************************************/
    ///* Switch moving cases from flags                                       */
    ///************************************************************************/
    //    parameterinfo->complan.isfirststep = msg.isfirststep;
    //    parameterinfo->complan.islaststep = msg.islaststep;
    switch (moving_mode)
    {
    case Right_turn:

        parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_ )//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Parameters->Shift_RX_fix
        parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_LX_fix



        parameterinfo->points.Y_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Open_value_R
        parameterinfo->points.Y_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Open_value_L
        parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
        parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
        parameterinfo->points.X_COM = OSC_COM_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, 0, time_point_);

        parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange

        if (parameterinfo->complan.isfirststep || parameterinfo->complan.islaststep)
        {
            parameterinfo->points.Right_Thta = 0;
            parameterinfo->points.Left_Thta = 0;
        }
        else
        {
            parameterinfo->points.Right_Thta =  OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.3 * parameterinfo->THTAUpdate * -1, 0, time_shift); //Swing leg turn right
            parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.7 * parameterinfo->THTAUpdate, 0, time_shift);
        }
        break;
    case Right_Protect:
        parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Parameters->Shift_RX_fix
        parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_LX_fix
        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Y_Right_foot = 0;
            parameterinfo->points.Y_Left_foot = 0;
        }
        else
        {
            parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_shift);//Parameters->Shift_delPower
            parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_shift);//Parameters->Shift_delPower

        }
        parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
        parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
        parameterinfo->points.X_COM = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->parameters.X_Swing_Range, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);
        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }
        else
        {
            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range-parameterinfo->YUpdate*0.25, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }
        parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange
        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Right_Thta = 0;
            parameterinfo->points.Left_Thta = 0;
        }
        else if (parameterinfo->THTAUpdate > 0) // Turn Left
        {
            parameterinfo->points.Right_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.7 * parameterinfo->THTAUpdate * -1, 0, time_shift);
            parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.3 * parameterinfo->THTAUpdate, 0, time_shift);	// Swing leg turn left;
        }
        else if (parameterinfo->THTAUpdate < 0) // Turn Right
        {
            parameterinfo->points.Right_Thta =  OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.3 * parameterinfo->THTAUpdate * -1, 0, time_shift); //Swing leg turn right
            parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.7 * parameterinfo->THTAUpdate, 0, time_shift);
        }
        else
        {
            parameterinfo->points.Right_Thta = 0;
            parameterinfo->points.Left_Thta = 0;
        }
        break;

    case Left_Turn:

        parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_RX_fix
        parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Parameters->Shift_LX_fix

        parameterinfo->points.Y_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Open_value_R
        parameterinfo->points.Y_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Open_value_L
        parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
        parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
        parameterinfo->points.X_COM = OSC_COM_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, 0, time_point_);
        parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_shift + parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange
        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Right_Thta = 0;
            parameterinfo->points.Left_Thta = 0;	// Swing leg turn left
        }
        else
        {
            parameterinfo->points.Right_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.7 * parameterinfo->THTAUpdate, 0, time_shift);
            parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.3 * parameterinfo->THTAUpdate * -1, 0, time_shift);	// Swing leg turn left;
        }
        break;
    case Left_Protect:
        parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate , 0, PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_RX_fix
        parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Parameters->Shift_LX_fix

        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Y_Right_foot = 0;
            parameterinfo->points.Y_Left_foot = 0;
        }
        else
        {
            parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_shift);//Parameters->Shift_delPower
            parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_shift);//Parameters->Shift_delPower
        }

        parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
        parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
        parameterinfo->points.X_COM = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->parameters.X_Swing_Range, 0, PI_2, time_shift + parameterinfo->parameters.Period_T2/4);
        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_shift + parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }
        else
        {
            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range+parameterinfo->YUpdate*0.25, 0, PI_2, time_shift + parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }

        parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange
        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Right_Thta = 0;
            parameterinfo->points.Left_Thta = 0;
        }
        else if (parameterinfo->THTAUpdate > 0) // Turn Left
        {
            parameterinfo->points.Right_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.7 * parameterinfo->THTAUpdate, 0, time_shift);
            parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.3 * parameterinfo->THTAUpdate * -1, 0, time_shift);	// Swing leg turn left;
        }
        else if (parameterinfo->THTAUpdate < 0) // Turn Right
        {

            parameterinfo->points.Right_Thta =  OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.3 * parameterinfo->THTAUpdate, 0, time_shift); //Swing leg turn right
            parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.7 * parameterinfo->THTAUpdate * -1, 0, time_shift);
        }
        else
        {
            parameterinfo->points.Right_Thta = 0;
            parameterinfo->points.Left_Thta = 0;
        }
        break;

    case Right_Shift:
        parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Parameters->Shift_RX_fix
        parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T,parameterinfo->XUpdate, 0, PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_LX_fix

        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Y_Right_foot = 0;
            parameterinfo->points.Y_Left_foot = 0;
        }
        else
        {
            parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_shift);//Parameters->Shift_delPower
            parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_shift);//Parameters->Shift_delPower
        }
        parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
        parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
        parameterinfo->points.X_COM = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->parameters.X_Swing_Range, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);

        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }
        else
        {
            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range-parameterinfo->YUpdate*0.25, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }
        parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange

        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Right_Thta = 0;
            parameterinfo->points.Left_Thta = 0;
        }
        else
        {
            parameterinfo->points.Right_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, -0/180*PI, 0, time_point_); //Swing leg turn right //Parameters->Shift_Lfoot_RTurn
            parameterinfo->points.Left_Thta = 0;
        }

        break;
    case Left_Shift:

        parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_RX_fix
        parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Parameters->Shift_LX_fix

        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Y_Right_foot = 0;
            parameterinfo->points.Y_Left_foot = 0;
        }
        else
        {
            parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_shift);//Parameters->Shift_delPower
            parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_shift);//Parameters->Shift_delPower
        }
        parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
        parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
        parameterinfo->points.X_COM = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->parameters.X_Swing_Range, 0, PI_2, time_shift + parameterinfo->parameters.Period_T2/4);

        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_shift + parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }
        else
        {
            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range+parameterinfo->YUpdate*0.25, 0, PI_2, time_shift + parameterinfo->parameters.Period_T/4);//OSC_COM_LockRange
        }

        parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange

        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Right_Thta = 0;
            parameterinfo->points.Left_Thta = 0;
        }
        else
        {
            parameterinfo->points.Right_Thta = 0;
            parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0/180*PI, PI, time_shift);//Parameters->Shift_Rfoot_LTurn
        }

        break;
    case Straight:

        parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange , parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0 , PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_RX_fix
        parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0 , PI_2, time_point_)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0 , PI, time_point_);//Parameters->Shift_LX_fix

        parameterinfo->points.Y_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Open_value_R
        parameterinfo->points.Y_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Open_value_L
        parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
        parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
        if (parameterinfo->complan.islaststep)
        {
            parameterinfo->points.X_COM = 0;
        }
        else
        {
            parameterinfo->points.X_COM = OSC_COM_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, 0, time_point_);
        }
        parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range,0, PI_2, time_shift+parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange

        parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange,
        if (parameterinfo->complan.islaststep)
        {
            parameterinfo->points.Right_Thta = 0;//821
            parameterinfo->points.Left_Thta = 0;//821
        }
        else
        {
            parameterinfo->points.Right_Thta = 0;//+= OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, time_shift);//Parameters->Str_Rfoot_Lturn
            parameterinfo->points.Left_Thta  = 0;//+= OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, time_point_);//-Parameters->Str_Lfoot_Rturn
        }
        break;
    default:
        parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift);//Parameters->BASE_delrho_x
        parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_);//Parameters->BASE_delrho_x
        parameterinfo->points.Y_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Open_value_R
        parameterinfo->points.Y_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Open_value_L
        parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
        parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
        parameterinfo->points.X_COM = OSC_COM_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate , 0, time_point_);
        parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range,0, PI_2, time_point_+parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        parameterinfo->points.Z_COM = 0;//parameterinfo->parameters.COM_Height + OSC_move_x_advance(parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0, time_point_);
        parameterinfo->points.Right_Thta = 0;
        parameterinfo->points.Left_Thta = 0;
    }

    osc_move_x_r.push_back(parameterinfo->points.X_Right_foot);
    osc_move_x_l.push_back(parameterinfo->points.X_Left_foot);
    osc_move_y_r.push_back(parameterinfo->points.Y_Right_foot);
    osc_move_y_l.push_back(parameterinfo->points.Y_Left_foot);
    osc_move_z_r.push_back(parameterinfo->points.Z_Right_foot);
    osc_move_z_l.push_back(parameterinfo->points.Z_Left_foot);
    osc_move_com_x.push_back(parameterinfo->points.X_COM);
    osc_move_com_y.push_back(parameterinfo->points.Y_COM);
    osc_move_com_z.push_back(parameterinfo->points.Z_COM);
    right_Thta.push_back(parameterinfo->points.Right_Thta);
    left_Thta.push_back(parameterinfo->points.Left_Thta);
    test.push_back(parameterinfo->counter);

    if(parameterinfo->complan.walking_stop)
    {
        //        ROS_INFO("Save");
        SaveData();
    }

}


void WalkingTrajectory::continuouswalk()
{
    int time_shift = 0;
    int time_point_ = parameterinfo->complan.time_point_;

    //    double open_value = sin(Parameters->Open_Phase);
    time_shift =  parameterinfo->complan.time_point_ + 0.5 * parameterinfo->parameters.Period_T;
    //    double Open_value_R = Parameters->R_Open;
    //    double Open_value_L = Parameters->L_Open;

    parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift);
    parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_);
    parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
    parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);

    parameterinfo->points.X_COM = OSC_COM_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, 0, time_point_);
    parameterinfo->points.Y_COM = OSC_COM_Y(parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, time_point_);
    parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_COM_Z(parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0, time_point_);
    // Turn
    if (parameterinfo->THTAUpdate > 0) // Turn Left
    {

        parameterinfo->points.Right_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THTAUpdate, 0, time_shift);
        parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THTAUpdate * -1, 0, time_shift);	// Swing leg turn left

    }
    else if (parameterinfo->THTAUpdate < 0) // Turn Right
    {
        parameterinfo->points.Right_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THTAUpdate * -1, 0, time_point_);//Swing leg turn right
        parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THTAUpdate, 0, time_point_);
    }
    else
    {
        parameterinfo->points.Right_Thta = 0;
        parameterinfo->points.Left_Thta = 0;
    }
    // Shift
    if(parameterinfo->complan.isfirststep)
    {
        parameterinfo->points.Y_Right_foot = 0;
        parameterinfo->points.Y_Left_foot = 0;
    }
    else if (parameterinfo->YUpdate < 0 && !parameterinfo->Isfrist_right_shift)
    {
        parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_point_);//Parameters->Shift_delPower
        parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_point_);//Parameters->Shift_delPower
        parameterinfo->points.Right_Thta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0*PI/180, 0, time_shift);	// Turn fix for shift  //Parameters->Shift_Rfoot_LTurn
        parameterinfo->points.Left_Thta = 0;
    }
    else if (parameterinfo->YUpdate > 0)
    {
        parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_shift);//Parameters->Shift_delPower
        parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_shift);//Parameters->Shift_delPower
        parameterinfo->points.Right_Thta = 0;
        parameterinfo->points.Left_Thta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0*PI/180, 0, time_point_); // Turn fix for shift  //Parameters->Shift_Lfoot_RTurn
    }
    else
    {
        //parameterinfo->points.Y_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T,0, PI, time_shift);// Open_value_R
        //parameterinfo->points.Y_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Open_value_L
    }
    // Add turn offset
    //parameterinfo->points.Y_Right_foot += 0;//Y_Right_foot_tmp_;
    //parameterinfo->points.Y_Left_foot += 0;//Y_Left_foot_tmp_;
    // Forward modify
    // if (parameterinfo->XUpdate != 0)
    // {
    //     parameterinfo->points.Right_Thta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, time_point_); //817(Lturn) //Parameters->Str_Rfoot_Lturn
    //     parameterinfo->points.Left_Thta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, time_shift); //-Parameters->Str_Lfoot_Rturn
    // }
//----------------------------------------------------------------------------------------------------
    // ROS_INFO("%d",now_step);
    // if(now_step == 0)//init
    // {
    //     Vx0 = WLIPM_com_Vx0(0, Wlength/2, 0, parameterinfo->parameters.COM_Height, TT, WTc, C, 0);
    //     Px = WLIPM_com_Px(0, Vx0, 0, parameterinfo->parameters.COM_Height, t, WTc, C, 0);
    //     Vx = WLIPM_com_Vx(0, Vx0, 0, parameterinfo->parameters.COM_Height, t, WTc, C, 0);
    //     COM_Z = WLIPM_com_Pz(COM_rho_Z, t, TT, T_DSP, now_step);
    //     LPx = WLIPM_Foot_Px_init(Wlength, t, TT, T_DSP, now_step);
    //     RPx = 0;
    //     LPz = WLIPM_Foot_Pz(Wheight, t, TT, T_DSP);
    //     RPz = 0;
    //     // WZMPY_data = Set_ZMPY_data(t, TT, T_DSP, 0);
    //     // WZMPX_data = Set_ZMPX_data_init(Wlength, t, TT, T_DSP, now_step);
    // }
    // else if(now_step == WN)//fin
    // {
    //     qq = now_step*Wlength;
        
    //     Vx0 = WLIPM_com_Vx0(-Wlength/2+qq, 0+qq, qq, parameterinfo->parameters.COM_Height, TT, WTc, C, 0);
    //     Px = WLIPM_com_Px(-Wlength/2+qq, Vx0, qq, parameterinfo->parameters.COM_Height, t, WTc, C, 0);
    //     Vx = WLIPM_com_Vx(-Wlength/2+qq, Vx0, qq, parameterinfo->parameters.COM_Height, t, WTc, C, 0);
    //     COM_Z = WLIPM_com_Pz(COM_rho_Z, t, TT, T_DSP, now_step);
    //     // WZMPX_data = Set_ZMPX_data_fin(Wlength, t, TT, T_DSP, now_step);
    //     if((now_step)%2 == 1)
    //     {
    //         LPx = Wlength*now_step;
    //         RPx = WLIPM_Foot_Px_fin(Wlength, t, TT, T_DSP, now_step);
    //         LPz = 0;
    //         RPz = WLIPM_Foot_Pz(Wheight, t, TT, T_DSP);
    //         // WZMPY_data = Set_ZMPY_data(t, TT, T_DSP, 1);
    //     }
    //     else if((now_step)%2 == 0)
    //     {
    //         LPx = WLIPM_Foot_Px_fin(Wlength, t, TT, T_DSP, now_step);
    //         RPx = Wlength*now_step;
    //         LPz = WLIPM_Foot_Pz(Wheight, t, TT, T_DSP);
    //         RPz = 0;
    //         // WZMPY_data = Set_ZMPY_data(t, TT, T_DSP, 0);
    //     }
    // }
    // else//repeat
    // {
    //     qq = now_step*Wlength;
        
    //     Vx0 = WLIPM_com_Vx0(-Wlength/2+qq, Wlength/2+qq, qq, parameterinfo->parameters.COM_Height, TT, WTc, C, 0);
    //     Px = WLIPM_com_Px(-Wlength/2+qq, Vx0, qq, parameterinfo->parameters.COM_Height, t, WTc, C, 0);
    //     Vx = WLIPM_com_Vx(-Wlength/2+qq, Vx0, qq, parameterinfo->parameters.COM_Height, t, WTc, C, 0);
    //     COM_Z = WLIPM_com_Pz(COM_rho_Z, t, TT, T_DSP, now_step);
    //     // WZMPX_data = Set_ZMPX_data_repeat(Wlength, t, TT, T_DSP, now_step);
    //     if((now_step)%2 == 1)
    //     {
    //         LPx = Wlength*now_step;
    //         RPx = WLIPM_Foot_Px_repeat(Wlength, t, TT, T_DSP, now_step);
    //         LPz = 0;
    //         RPz = WLIPM_Foot_Pz(Wheight, t, TT, T_DSP);
    //         // WZMPY_data = Set_ZMPY_data(t, TT, T_DSP, 1);
    //     }
    //     else if((now_step)%2 == 0)
    //     {
    //         LPx = WLIPM_Foot_Px_repeat(Wlength, t, TT, T_DSP, now_step);
    //         RPx = Wlength*now_step;
    //         LPz = WLIPM_Foot_Pz(Wheight, t, TT, T_DSP);
    //         RPz = 0;
    //         // WZMPY_data = Set_ZMPY_data(t, TT, T_DSP, 0);
    //     }
    // }
    
    // if(now_step%2 == 1)
    // {
    //     Wwidth = -Wwidth_size;
    // }
    // else if(now_step%2 == 0)
    // {
    //     Wwidth = Wwidth_size;
    // }
    // Vy0 = WLIPM_com_Vy0(0, -Wwidth, parameterinfo->parameters.COM_Height, TT, WTc);
    // Py = WLIPM_com_Py(0, Vy0, -Wwidth, parameterinfo->parameters.COM_Height, t, WTc);
    // Vy = WLIPM_com_Vy(0, Vy0, -Wwidth, parameterinfo->parameters.COM_Height, t, WTc);
    
    // // Wstep_pointL_X = LPx - Px;
    // // Wstep_pointR_X = RPx - Px;
    // // Wstep_pointL_Y = 0 - Py;
    // // Wstep_pointR_Y = 0 - Py;
    // // if(Wmode != 4)
    // // {
    // //     Wstep_pointL_Z = parameterinfo->parameters.COM_Height - LPz;
    // //     Wstep_pointR_Z = parameterinfo->parameters.COM_Height - RPz;
    // // }
    // // else if(now_step%2 == 1)
    // // {
    // //     Wstep_pointL_Z = parameterinfo->parameters.COM_Height - LPz + COM_Z;
    // //     Wstep_pointR_Z = parameterinfo->parameters.COM_Height - RPz;
    // // }
    // // else if(now_step%2 == 0)
    // // {
    // //     Wstep_pointL_Z = parameterinfo->parameters.COM_Height - LPz;
    // //     Wstep_pointR_Z = parameterinfo->parameters.COM_Height - RPz + COM_Z;
    // // }
    // // Wstep_pointL_theta = 0;
    // // Wstep_pointR_theta = 0;
    
    // // if(now_step > WN)
    // // {
    // //     Wstep_pointL_X = 0;
    // //     Wstep_pointR_X = 0;
    // //     Wstep_pointL_Y = 0;
    // //     Wstep_pointR_Y = 0;
    // //     Wstep_pointL_Z = parameterinfo->parameters.COM_Height;
    // //     Wstep_pointR_Z = parameterinfo->parameters.COM_Height;
    // //     Wstep_pointL_theta = 0;
    // //     Wstep_pointR_theta = 0;
    // // }
//----------------------------------------------------------------------------------------------------
    osc_move_x_r.push_back(RPx);
    osc_move_x_l.push_back(LPx);
    osc_move_y_r.push_back(0);
    osc_move_y_l.push_back(0);
    osc_move_z_r.push_back(RPz);
    osc_move_z_l.push_back(LPz);
    osc_move_com_x.push_back(Px);
    osc_move_com_y.push_back(Py);
    osc_move_com_z.push_back(COM_Z);
    right_Thta.push_back(parameterinfo->points.Right_Thta);
    left_Thta.push_back(parameterinfo->points.Left_Thta);
    test.push_back(parameterinfo->counter);

    if(parameterinfo->complan.walking_stop)
    {
        ROS_INFO("save");
        SaveData();
    }
}

void WalkingTrajectory::osccontinuouswalk()//JJ
{
    int time_shift = 0;
    int time_point_ = parameterinfo->complan.time_point_;

    //    double open_value = sin(Parameters->Open_Phase);
    time_shift =  parameterinfo->complan.time_point_ + 0.5 * parameterinfo->parameters.Period_T;
    //    double Open_value_R = Parameters->R_Open;
    //    double Open_value_L = Parameters->L_Open;
    double wv_x = WOSC_Waist_V(parameterinfo->parameters.Ts, parameterinfo->parameters.Vmin, parameterinfo->parameters.Vmax, parameterinfo->parameters.Period_T, 0, time_point_, parameterinfo->parameters.Sample_Time);//- (1.5*parameterinfo->parameters.Ts+(parameterinfo->parameters.Period_T/parameterinfo->parameters.Sample_Time))
    //double rx = WOSC_Foot_X(parameterinfo->parameters.Ts, parameterinfo->parameters.Vmin, parameterinfo->parameters.Vmax, parameterinfo->parameters.Period_T, 0, time_point_, parameterinfo->parameters.Sample_Time);//- 1.5*parameterinfo->parameters.Ts
    double absx = WOSC_Foot_X(parameterinfo->parameters.Ts, parameterinfo->parameters.Vmin, parameterinfo->parameters.Vmax, parameterinfo->parameters.Period_T, PI, time_point_, parameterinfo->parameters.Sample_Time);//- 1.5*parameterinfo->parameters.Ts
    
    if(time_point_ <= parameterinfo->parameters.Ts*4*3)
    {
        parameterinfo->points.X_Right_foot = 0;
        parameterinfo->points.X_Left_foot = 0;
    }
    else
    {
        parameterinfo->points.X_Right_foot = wosc_foot_x_r-wv_x-(200*(parameterinfo->parameters.Vmax-parameterinfo->parameters.Vmin));
        parameterinfo->points.X_Left_foot = wosc_foot_x_l-wv_x-(200*(parameterinfo->parameters.Vmax-parameterinfo->parameters.Vmin));
    }
    parameterinfo->points.Z_Right_foot = WOSC_Foot_Z(parameterinfo->parameters.Ts, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, 0, time_point_,1);
    parameterinfo->points.Z_Left_foot = WOSC_Foot_Z(parameterinfo->parameters.Ts, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, 0, time_point_,0);
    //    parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
    //    parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);

    parameterinfo->points.X_COM = 0;//wv_x;//OSC_COM_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, 0, time_point_);
    parameterinfo->points.Y_COM = WOSC_Waist_Y(parameterinfo->parameters.Ts, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, time_point_);//OSC_COM_Y(parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, time_point_);
    parameterinfo->points.Z_COM =  parameterinfo->parameters.COM_Height + WOSC_Waist_Z(parameterinfo->parameters.Ts, parameterinfo->parameters.Period_T, parameterinfo->parameters.Z_Swing_Range, 0, time_point_);

    parameterinfo->points.wv_x_COM = wv_x;
    parameterinfo->points.abs_x_feet = absx;
    parameterinfo->points.foot_x_r = wosc_foot_x_r;
    parameterinfo->points.foot_x_l = wosc_foot_x_l;

    //OSC_COM_Z(parameterinfo->parameters.Period_T, parameterinfo->parameters.Z_Swing_Range, PI_2, time_point_);
    // Turn

    if (parameterinfo->THTAUpdate > 0) // Turn Left
    {

        parameterinfo->points.Right_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THTAUpdate, 0, time_shift);
        parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THTAUpdate * -1, 0, time_shift);	// Swing leg turn left

    }
    else if (parameterinfo->THTAUpdate < 0) // Turn Right
    {
        parameterinfo->points.Right_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THTAUpdate * -1, 0, time_point_);//Swing leg turn right
        parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THTAUpdate, 0, time_point_);
    }
    else
    {
        parameterinfo->points.Right_Thta = 0;
        parameterinfo->points.Left_Thta = 0;
    }
    // Shift
    if (parameterinfo->YUpdate > 0)
    {
        parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate, 0, time_shift);//Parameters->Shift_delPower
        parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate * -1, 0, time_shift);//Parameters->Shift_delPower
        parameterinfo->points.Right_Thta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0*PI/180, 0, time_shift);	// Turn fix for shift  //Parameters->Shift_Rfoot_LTurn
        parameterinfo->points.Left_Thta = 0;
    }
    else if (parameterinfo->YUpdate < 0)
    {
        parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_shift);//Parameters->Shift_delPower
        parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_shift);//Parameters->Shift_delPower
        parameterinfo->points.Right_Thta = 0;
        parameterinfo->points.Left_Thta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0*PI/180, 0, time_point_); // Turn fix for shift  //Parameters->Shift_Lfoot_RTurn
    }
    else
    {
        parameterinfo->points.Y_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T,0, PI, time_shift);// Open_value_R
        parameterinfo->points.Y_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Open_value_L
    }
    // Add turn offset
    parameterinfo->points.Y_Right_foot += 0;//Y_Right_foot_tmp_;
    parameterinfo->points.Y_Left_foot += 0;//Y_Left_foot_tmp_;
    // Forward modify
    if (parameterinfo->XUpdate != 0)
    {
        parameterinfo->points.Right_Thta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, time_point_); //817(Lturn) //Parameters->Str_Rfoot_Lturn
        parameterinfo->points.Left_Thta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, time_shift); //-Parameters->Str_Lfoot_Rturn
    }

    osc_move_x_r.push_back(parameterinfo->points.X_Right_foot);
    osc_move_x_l.push_back(parameterinfo->points.X_Left_foot);
    osc_move_y_r.push_back(parameterinfo->points.Y_Right_foot);
    osc_move_y_l.push_back(parameterinfo->points.Y_Left_foot);
    osc_move_z_r.push_back(parameterinfo->points.Z_Right_foot);
    osc_move_z_l.push_back(parameterinfo->points.Z_Left_foot);
    osc_move_com_x.push_back(parameterinfo->points.X_COM);
    osc_move_com_y.push_back(parameterinfo->points.Y_COM);
    osc_move_com_z.push_back(parameterinfo->points.Z_COM);
    right_Thta.push_back(parameterinfo->points.Right_Thta);
    left_Thta.push_back(parameterinfo->points.Left_Thta);
    test.push_back(parameterinfo->counter);

    if(parameterinfo->complan.walking_stop)
    {
        SaveData();
    }
}

void WalkingTrajectory::LC_walkfunction()
{
    int time_shift = 0;
  //  int time_point_ = parameterinfo->complan.time_point_;

    //    double open_value = sin(Parameters->Open_Phase);
    time_shift =  parameterinfo->complan.time_point_ + 0.5 * parameterinfo->parameters.Period_T;
    //    double Open_value_R = Parameters->R_Open;
    //    double Open_value_L = Parameters->L_Open;

//    case Straight:

    parameterinfo->points.Y_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, parameterinfo->complan.time_point_);//Open_value_R
    parameterinfo->points.Y_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Open_value_L
    
    if (parameterinfo->complan.lift_lock_x != 0)
    {
        parameterinfo->points.X_Right_foot = (OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate ,0 , PI_2, time_shift)+OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift));//2.4462*7;
        ROS_INFO("0%f", parameterinfo->points.X_Right_foot);
        if (parameterinfo->points.X_Right_foot > parameterinfo->complan.lift_lock_x)
        {
            parameterinfo->points.X_Right_foot = parameterinfo->complan.lift_lock_x;
            ROS_INFO("1%f", parameterinfo->points.X_Right_foot);
        }else{
            parameterinfo->points.X_Left_foot = (OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate ,0 , PI_2, parameterinfo->complan.time_point_)+OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, parameterinfo->complan.time_point_));//2.4462*7;
            parameterinfo->complan.lift_lock_x = 0;
            ROS_INFO("L%f", parameterinfo->points.X_Right_foot);
        }
        ROS_INFO("2%f", parameterinfo->points.X_Right_foot);

    }
    else
    {
        parameterinfo->points.X_Right_foot = (OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate ,0 , PI_2, time_shift)+OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift));//2.4462*7;
        parameterinfo->points.X_Left_foot = (OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate ,0 , PI_2, parameterinfo->complan.time_point_)+OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, parameterinfo->complan.time_point_));//2.4462*7;
        parameterinfo->complan.lift_lock_x = parameterinfo->points.X_Right_foot;
        ROS_INFO("%f", parameterinfo->points.X_Right_foot);
    }


    if (parameterinfo->parameters.BASE_LIFT_Z > 0)
    {
        parameterinfo->points.Z_Right_foot = OSC_lifemove_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, parameterinfo->parameters.BASE_LIFT_Z, parameterinfo->complan.time_point_-parameterinfo->parameters.Period_T/16);
        parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, parameterinfo->complan.time_point_-parameterinfo->parameters.Period_T/32);

    }else if (parameterinfo->parameters.BASE_LIFT_Z < 0)
    {
        parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
        parameterinfo->points.Z_Left_foot = OSC_lifemove_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, parameterinfo->parameters.BASE_LIFT_Z, parameterinfo->complan.time_point_);
    }

    if (parameterinfo->complan.islaststep)
    {
        parameterinfo->points.X_COM = 0;
    }else{
        parameterinfo->points.X_COM = OSC_COM_Lift_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, parameterinfo->parameters.X_Swing_COM, parameterinfo->complan.time_point_);
    }
    parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_shift+parameterinfo->parameters.Period_T/4);
    parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, parameterinfo->complan.time_point_);

    if (parameterinfo->complan.islaststep)
    {
        parameterinfo->points.Right_Thta = 0;//821
        parameterinfo->points.Left_Thta = 0;//821
    }else{
        parameterinfo->points.Right_Thta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, time_shift);//Parameters->Str_Rfoot_Lturn
        parameterinfo->points.Left_Thta  += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, parameterinfo->complan.time_point_);//-Parameters->Str_Lfoot_Rturn
    }
    

    osc_move_x_r.push_back(parameterinfo->points.X_Right_foot);
    osc_move_x_l.push_back(parameterinfo->points.X_Left_foot);
    osc_move_y_r.push_back(parameterinfo->points.Y_Right_foot);
    osc_move_y_l.push_back(parameterinfo->points.Y_Left_foot);
    osc_move_z_r.push_back(parameterinfo->points.Z_Right_foot);
    osc_move_z_l.push_back(parameterinfo->points.Z_Left_foot);
    osc_move_com_x.push_back(parameterinfo->points.X_COM);
    osc_move_com_y.push_back(parameterinfo->points.Y_COM);
    osc_move_com_z.push_back(parameterinfo->points.Z_COM);
    right_Thta.push_back(parameterinfo->points.Right_Thta);
    left_Thta.push_back(parameterinfo->points.Left_Thta);
    test.push_back(parameterinfo->counter);

    if(parameterinfo->complan.walking_stop)
    {
        //        ROS_INFO("Save");
        SaveData();
    }
}

void WalkingTrajectory::longjump_function()
{
    int time_shift = 0;
    int time_point_ = parameterinfo->complan.time_point_;

    //    double open_value = sin(Parameters->Open_Phase);
    time_shift =  parameterinfo->complan.time_point_ + 0.5 * parameterinfo->parameters.Period_T;
    //    double Open_value_R = Parameters->R_Open;
    //    double Open_value_L = Parameters->L_Open;

    /************************************************************************/
    /* Set flags of moving cases                                            */
    /************************************************************************/

    parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange , parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0 , PI_2, time_point_);
    parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0 , PI_2, time_point_);
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
    parameterinfo->points.Y_COM = 0.0;//OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range,0, PI_2, time_shift+parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
    parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange,
    if (parameterinfo->complan.islaststep)
    {
        parameterinfo->points.Right_Thta = 0;//821
        parameterinfo->points.Left_Thta = 0;//821
    }
    else
    {
        parameterinfo->points.Right_Thta = 0;//+= OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, time_shift);//Parameters->Str_Rfoot_Lturn
        parameterinfo->points.Left_Thta  = 0;//+= OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, time_point_);//-Parameters->Str_Lfoot_Rturn
    }


    osc_move_x_r.push_back(parameterinfo->points.X_Right_foot);
    osc_move_x_l.push_back(parameterinfo->points.X_Left_foot);
    osc_move_y_r.push_back(parameterinfo->points.Y_Right_foot);
    osc_move_y_l.push_back(parameterinfo->points.Y_Left_foot);
    osc_move_z_r.push_back(parameterinfo->points.Z_Right_foot);
    osc_move_z_l.push_back(parameterinfo->points.Z_Left_foot);
    osc_move_com_x.push_back(parameterinfo->points.X_COM);
    osc_move_com_y.push_back(parameterinfo->points.Y_COM);
    osc_move_com_z.push_back(parameterinfo->points.Z_COM);
    right_Thta.push_back(parameterinfo->points.Right_Thta);
    left_Thta.push_back(parameterinfo->points.Left_Thta);
    test.push_back(parameterinfo->counter);

    if(parameterinfo->complan.walking_stop)
    {
        // ROS_INFO("Save");
        SaveData();
    }
}

void WalkingTrajectory::inversekinmaticsinfo()
{
    //----------------------------------------------------
    // // Wstep_pointL_X = LPx - Px;
    // // Wstep_pointR_X = RPx - Px;
    // // Wstep_pointL_Y = 0 - Py;
    // // Wstep_pointR_Y = 0 - Py;
    // // if(Wmode != 4)
    // // {
    // //     Wstep_pointL_Z = parameterinfo->parameters.COM_Height - LPz;
    // //     Wstep_pointR_Z = parameterinfo->parameters.COM_Height - RPz;
    // // }
    // // else if(now_step%2 == 1)
    // // {
    // //     Wstep_pointL_Z = parameterinfo->parameters.COM_Height - LPz + COM_Z;
    // //     Wstep_pointR_Z = parameterinfo->parameters.COM_Height - RPz;
    // // }
    // // else if(now_step%2 == 0)
    // // {
    // //     Wstep_pointL_Z = parameterinfo->parameters.COM_Height - LPz;
    // //     Wstep_pointR_Z = parameterinfo->parameters.COM_Height - RPz + COM_Z;
    // // }
    // // Wstep_pointL_theta = 0;
    // // Wstep_pointR_theta = 0;
    
    // // if(now_step > WN)
    // // {
    // //     Wstep_pointL_X = 0;
    // //     Wstep_pointR_X = 0;
    // //     Wstep_pointL_Y = 0;
    // //     Wstep_pointR_Y = 0;
    // //     Wstep_pointL_Z = parameterinfo->parameters.COM_Height;
    // //     Wstep_pointR_Z = parameterinfo->parameters.COM_Height;
    // //     Wstep_pointL_theta = 0;
    // //     Wstep_pointR_theta = 0;
    // // }
    // parameterinfo->points.IK_Point_RX = RPx - Px;
    // parameterinfo->points.IK_Point_RY = 0 - Py - 4;
    // parameterinfo->points.IK_Point_RZ =  RPz;
    // parameterinfo->points.IK_Point_RThta = 0;
    // parameterinfo->points.IK_Point_LX = LPx - Px;
    // parameterinfo->points.IK_Point_LY = 0 - Py + 4;
    // parameterinfo->points.IK_Point_LZ =  LPz;
    // parameterinfo->points.IK_Point_LThta = 0;

    // if(now_step > WN)
    // {
    //     parameterinfo->points.IK_Point_RX = 0;
    //     parameterinfo->points.IK_Point_RY = -4;
    //     parameterinfo->points.IK_Point_RZ = 0;
    //     parameterinfo->points.IK_Point_RThta = 0;
    //     parameterinfo->points.IK_Point_LX = 0;
    //     parameterinfo->points.IK_Point_LY = 4;
    //     parameterinfo->points.IK_Point_LZ = 0;
    //     parameterinfo->points.IK_Point_LThta = 0;
    // }

    // parameterinfo->ik_parameters.R_Goal[0] = -parameterinfo->points.IK_Point_RX;
    // parameterinfo->ik_parameters.R_Goal[1] = parameterinfo->points.IK_Point_RY;
    // parameterinfo->ik_parameters.R_Goal[2] = parameterinfo->points.IK_Point_RZ;
    // parameterinfo->ik_parameters.R_Goal[3] = parameterinfo->points.IK_Point_RThta;
    // parameterinfo->ik_parameters.L_Goal[0] = parameterinfo->points.IK_Point_LX;
    // parameterinfo->ik_parameters.L_Goal[1] = parameterinfo->points.IK_Point_LY;
    // parameterinfo->ik_parameters.L_Goal[2] = parameterinfo->points.IK_Point_LZ;
    // parameterinfo->ik_parameters.L_Goal[3] = parameterinfo->points.IK_Point_LThta;
    //----------------------------------------------------------------
    parameterinfo->points.IK_Point_RX = parameterinfo->points.X_COM + parameterinfo->points.X_Right_foot;
    parameterinfo->points.IK_Point_RY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Right_foot;
    parameterinfo->points.IK_Point_RZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Right_foot;
    parameterinfo->points.IK_Point_RThta = parameterinfo->points.Right_Thta;
    parameterinfo->points.IK_Point_LX = parameterinfo->points.X_COM + parameterinfo->points.X_Left_foot;
    parameterinfo->points.IK_Point_LY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Left_foot;
    parameterinfo->points.IK_Point_LZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Left_foot;
    parameterinfo->points.IK_Point_LThta = parameterinfo->points.Left_Thta;

    parameterinfo->ik_parameters.R_Goal[0] = -parameterinfo->points.IK_Point_RX;
    parameterinfo->ik_parameters.R_Goal[1] = parameterinfo->points.IK_Point_RY - 4;
    parameterinfo->ik_parameters.R_Goal[2] = 21.7 - parameterinfo->points.IK_Point_RZ;
    parameterinfo->ik_parameters.R_Goal[3] = parameterinfo->points.IK_Point_RThta;
    parameterinfo->ik_parameters.L_Goal[0] = parameterinfo->points.IK_Point_LX;
    parameterinfo->ik_parameters.L_Goal[1] = parameterinfo->points.IK_Point_LY + 4;
    parameterinfo->ik_parameters.L_Goal[2] = 21.7 - parameterinfo->points.IK_Point_LZ;
    parameterinfo->ik_parameters.L_Goal[3] = parameterinfo->points.IK_Point_LThta;
}

string WalkingTrajectory::DtoS(double value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();

    return str;
}

void WalkingTrajectory::SaveData()
{
    string savedText = "R_move_X\tL_move_X\t"
                       "R_move_Y\tL_move_Y\t"
                       "R_move_Z\tL_move_Z\t"
                       "move_COM_X\tmove_COM_Y\tmove_COM_Z\t"
                       "R_Thta\tL_Thta\tPoint\n";
    char path[200];
    string PATH = ros::package::getPath("strategy") + "/Parameter";
    strcpy(path, PATH.c_str());
    strcat(path, "/Trajectory_Record.ods");
    //char filename[]="/home/iclab/Desktop/OBS0722SLOW/newsystem/src/strategy/src/spartanrace/Parameter/Trajectory_Record.ods";

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
                + DtoS(right_Thta[i]) + "\t"
                + DtoS(left_Thta[i]) + "\t"
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
    right_Thta.clear();
    left_Thta.clear();
    test.clear();

    fp.close();
}
double WalkingTrajectory::OSC_COM_Lift_X(double range, double period_T_ms, double rho_com_x, double delta_com_x, int time_t_ms)
{
    double omega_com_x;
    double rho_x;
    double period_T = period_T_ms * 0.001;
    double T_divbylock = period_T * (range / 2);
    double T_divbyunlock = period_T * (1-range) / 2;
    omega_com_x = 2 * PI / period_T;
    rho_x = rho_com_x * (range) * delta_com_x;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t =(double)time_t_temp / 1000;

    if (time_t >= 0 && time_t < T_divbylock)
    {
        return rho_x * sin((2 * PI / (T_divbylock * 4)) * time_t + PI);
    }
    else if (time_t < period_T && time_t > (period_T - T_divbylock))
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
    //int ms_period_T = (int)(period_T*1000);
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    omega_x = 2 * PI / period_T / (1-range);

    if(time_t >= 0 && time_t < range * T_divby4)
    {
        return rho_x *(1 + BASE_delrho_x);
    }
    else if(time_t >= range * T_divby4 && time_t < (T_divby2 - (range * T_divby4)) )
    {
        if (time_t<=T_divby4)
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
        if (time_t <= period_T - T_divby4)
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

double WalkingTrajectory::OSC_move_z(double range, double period_T_ms, double rho_z, double delta_z, int time_t_ms)
{
    double omega_z;
    double period_T = period_T_ms * 0.001;
    double T_divby2 = period_T / 2;
    double T_divby4 = period_T / 4;
    //int ms_period_T = (int)(period_T*1000);
    int time_t_temp = time_t_ms % ( int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    omega_z = 2 * PI / period_T / (1-range);

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
    double omega_com_x;
    double rho_x;
    double period_T = period_T_ms * 0.001;
    double T_divbylock = period_T * (range/2);
    double T_divbyunlock = period_T * (1-range)/2;
    omega_com_x = 2 * PI / period_T;
    rho_x = rho_com_x * (range) * (parameterinfo->parameters.X_Swing_Range);
    //int ms_period_T = (int)(period_T*1000);
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;
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
    double r4 = 3*T_divby4;
    omega_y = 2 * PI / period_T / (1-range);
    if (time_t >= 0 && time_t < range * T_divby4  )
    {
        return 0;
    }
    else if(time_t >=  range * T_divby4  && time_t < r1)
    {
        return rho_y * sin(omega_y * (time_t - range * T_divby4) + delta_y);
    }
    else if (time_t >= r1 && time_t < r3)
    {
        return rho_y;
    }
    else if (time_t >= r3 && time_t <= r4)
    {
        //return rho_y * sin(omega_y * (time_t - range * 3 * T_divby4 - period_T) + delta_y);
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
    omega_com_y = 2 * PI / period_T;
    //int ms_period_T = (int)(period_T*1000);
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    return rho_com_y * sin(omega_com_y * time_t + delta_com_y);
}

double WalkingTrajectory::OSC_COM_Z(double period_T_ms, double rho_com_z, double delta_com_z, int time_t_ms)
{
    double omega_com_z;
    double period_T = period_T_ms * 0.001;
    omega_com_z =  2*PI / period_T;
    //int ms_period_T = (int)(period_T*1000);
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    return rho_com_z * sin(omega_com_z * time_t + delta_com_z);
}

double WalkingTrajectory::OSC_Rotate(double range, double period_T_ms, double rho_y, double delta_y, int time_t_ms)
{
    return OSC_move_shift_y( range,  period_T_ms,  rho_y,  delta_y,  time_t_ms);
}

double WalkingTrajectory::WOSC_Waist_V(int Ts, double Vmin, double Vmax, double period_T_ms, double delta, int time_t_ms, int Sample_Time)
{

    int Td = period_T_ms-Ts;
    int Tssd = 2*Ts;
    double period_T = period_T_ms * 0.001;
    int time_t_temp = time_t_ms % (int)period_T_ms;//parameterinfo->parameters.abswaistx +
    double time_t = (double)time_t_temp / 1000;

    if(time_t_temp >= 0 && time_t_temp < Ts)
        parameterinfo->parameters.abswaistx += (0.5*(Vmax-Vmin)*(1-cos(PI*time_t_temp/(double)Ts))+Vmin)*(period_T_ms/(double)(Sample_Time));
    else if(time_t_temp >= Ts && time_t_temp < Td)
        parameterinfo->parameters.abswaistx += (Vmax)*(period_T_ms/(double)(Sample_Time));
    else if(time_t_temp >= Td && time_t_temp < period_T_ms)
        parameterinfo->parameters.abswaistx += (0.5*(Vmax-Vmin)*(1-cos(PI*(time_t_temp-Td-Ts)/(double)Ts))+Vmin)*(period_T_ms/(double)(Sample_Time));

    return parameterinfo->parameters.abswaistx;
}

double WalkingTrajectory::WOSC_Foot_X(int Ts, double Vmin, double Vmax, double period_T_ms, double delta, int time_t_ms, int Sample_Time)
{
    wosc_foot_x_last = wosc_foot_x_now;

    int Td = period_T_ms-Ts;
    double a = (Vmax*Td+Vmin*Ts)/PI/(period_T_ms/Ts)*4;
    int Tssd = 2*Ts;

    double period_T = period_T_ms * 0.001;
    int time_t_temp = time_t_ms % (int)(period_T_ms*4);
    double time_t = (double)time_t_temp / 1000;

    double q = 2*PI*time_t_ms/(Tssd*2.0);
    wosc_foot_x_now = a*(q-sin(q+delta));

    double x = wosc_foot_x_now - wosc_foot_x_last;

    if((time_t_temp >= 0 && time_t_temp < period_T_ms) || (time_t_temp >= period_T_ms*3 && time_t_temp < period_T_ms*4))
    {
        wosc_foot_x_r += x;
        wosc_foot_x_l = wosc_foot_x_l;
    }
    else if(time_t_temp >= period_T_ms && time_t_temp < period_T_ms*3)
    {
        wosc_foot_x_l += x;
        wosc_foot_x_r = wosc_foot_x_r;
    }

    return wosc_foot_x_now;

}

double WalkingTrajectory::WOSC_Foot_Z(int Ts, double period_T_ms, double Hfoot, double delta, int time_t_ms, int state)
{
    int Tssd = 2*Ts;
    double q = PI*time_t_ms/Tssd;
    double period_T = period_T_ms * 0.001;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;
    int aa = (time_t_ms-period_T_ms)/(period_T_ms/(period_T_ms/2.0/period_T_ms));

    if(time_t_ms < Ts*2+Ts*4+Ts*4)
        return 0;
    else
        if( (aa%2) == state)
            return Hfoot/2*(1-cos(q+PI));
        else if( (aa%2) == state)
            return Hfoot/2*(1-cos(q+PI));
        else
            return 0;
}

double WalkingTrajectory::WOSC_Waist_Y(int Ts, double period_T_ms, double Ay, double delta_z, int time_t_ms)
{
    int Tssd = 2*Ts;
    double q = 2*PI*time_t_ms/Tssd/2.0/2.0;
    double period_T = period_T_ms * 0.001;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;
    int aa = (time_t_ms/Ts)/(period_T_ms/(period_T_ms/2/Ts));

    if(time_t_ms < Ts*2)
        return 0;
    else if(time_t_ms < Ts*2+Ts*4+Ts*4)
        return (Ay+0.2)*(cos(q));//+PI/2+PI/2
    else
        return Ay*(cos(q));//+PI/2+PI/2
}

double WalkingTrajectory::WOSC_Waist_Z(int Ts, double period_T_ms, double Az, double delta_z, int time_t_ms)
{
    int Tssd = 2*Ts;
    double q = 2*PI*time_t_ms/Tssd/2.0;
    double period_T = period_T_ms * 0.001;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    if(time_t_ms < Ts*2+Ts*4+Ts*4)
        return 0;
    else
        return Az*(cos(q+PI/4+PI/4));

}

double WalkingTrajectory::OSC_lifemove_z(double range, double period_T_ms, double rho_z, double delta_z, double life_high, int time_t_ms)
{
    double omega_z;
    double outputZ;
    double period_T = period_T_ms * 0.001;
    double T_divby2 = period_T / 2;
    double T_divby4 = period_T / 4;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    if (life_high < 0)
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
    else if(time_t >= T_divby2 - range * T_divby4 && time_t < T_divby2 + range * T_divby4)
    {
        return abs(life_high);
    }
    else if(time_t >= T_divby2 + range * T_divby4 && time_t < T_divby2 + T_divby4)
    {
        outputZ = rho_z * sin(omega_z * (time_t - range * 3 * T_divby4) + delta_z);
        outputZ = -(outputZ - abs(life_high));
        if (outputZ < abs(life_high) && outputZ >= 0)
        {
            return outputZ;
        }else if (outputZ < 0)
        {
            return 0;
        }else{
            return abs(life_high);
        }
    }else{
        return 0;
    }

}
//----------------------------------------------------
double WalkingTrajectory::WLIPM_com_Vx0(double X0, double Xt, double Px, double z, double t, double T, double C, double q)
{
    return (Xt-X0*cosh(t/T)+(Px+(pow(T,2)*q/pow(C,2)))*(cosh(t/T)-1))/(T*sinh(t/T));
}
double WalkingTrajectory::WLIPM_com_Px(double X0, double dX0, double Px, double z, double t, double T, double C, double q)
{
    return  X0*cosh(t/T)+T*dX0*sinh(t/T)-(Px+(pow(T,2)*q/pow(C,2)))*(cosh(t/T)-1);
}
double WalkingTrajectory::WLIPM_com_Vx(double X0, double dX0, double Px, double z, double t, double T, double C, double q)
{
    return X0*sinh(t/T)/T+dX0*cosh(t/T)-(Px/T+(T*q/C))*sinh(t/T);
}
double WalkingTrajectory::WLIPM_com_Vy0(double Y0, double Py, double z, double t, double T)
{
    return ((Y0-Y0*cosh(t/T))+Py*(cosh(t/T)-1))/(T*sinh(t/T));
}
double WalkingTrajectory::WLIPM_com_Py(double Y0, double dY0, double Py, double z, double t, double T)
{
    return Y0*cosh(t/T)+T*dY0*sinh(t/T)-Py*(cosh(t/T)-1);
}
double WalkingTrajectory::WLIPM_com_Vy(double Y0, double dY0, double Py, double z, double t, double T)
{
    return Y0*sinh(t/T)/T+dY0*cosh(t/T)-(Py/T)*sinh(t/T);
}
double WalkingTrajectory::WLIPM_com_Pz(const double COM_rho_Z, const double t, const double T, const double T_DSP, const int now_step)
{ 
    double omega = 2*PI/T;
    // return COM_rho_Z*(sin(omega*t));//same as zero 0-(-0.1)-0.1
    return 0.5*COM_rho_Z*(1-cos(omega*t));//best 0-(0.1)-0.2
}
double WalkingTrajectory::WLIPM_Foot_Px_init(const double Wlength, const double t, const double T, const double T_DSP, const int now_step)
{
    double new_T = T*(1-T_DSP);
    double new_t = t-T*T_DSP/2;
    double omega = 2*PI/new_T;
    
    if(t > T*T_DSP/2 && t <= T*(1-T_DSP/2))
    {
        return Wlength*(omega*new_t-sin(omega*new_t+0))/(2*PI);
    }
    else if(t <= T*T_DSP/2 && t > 0)
    {
        return Wlength*now_step;
    }
    else if(t > T*(1-T_DSP/2) && t <= T)
    {
        return Wlength*(now_step+1);
    }
    else 
    {
        return -1;
    }
}
double WalkingTrajectory::WLIPM_Foot_Px_fin(const double Wlength, const double t, const double T, const double T_DSP, const int now_step)
{
    double new_T = T*(1-T_DSP);
    double new_t = t-T*T_DSP/2;
    double omega = 2*PI/new_T;
    
    if(t > T*T_DSP/2 && t <= T*(1-T_DSP/2))
    {
        return Wlength*(omega*new_t-sin(omega*new_t+0))/(2*PI)+Wlength*(now_step-1);
    }
    else if(t <= T*T_DSP/2 && t > 0)
    {
        return Wlength*(now_step-1);
    }
    else if(t > T*(1-T_DSP/2) && t <= T)
    {
        return Wlength*now_step;
    }
    else 
    {
        return -1;
    }
}
double WalkingTrajectory::WLIPM_Foot_Px_repeat(const double Wlength, const double t, const double T, const double T_DSP, const int now_step)
{
    double new_T = T*(1-T_DSP);
    double new_t = t-T*T_DSP/2;
    double omega = 2*PI/new_T;
    
    if(t > T*T_DSP/2 && t <= T*(1-T_DSP/2))
    {
        return 2*Wlength*(omega*new_t-sin(omega*new_t+0))/(2*PI)+Wlength*(now_step-1);
    }
    else if(t <= T*T_DSP/2 && t > 0)
    {
        return Wlength*(now_step-1);
    }
    else if(t > T*(1-T_DSP/2) && t <= T)
    {
        return Wlength*(now_step+1);
    }
    else 
    {
        return -1;
    }
}
double WalkingTrajectory::WLIPM_Foot_Pz(const double Wheight, const double t, const double T, const double T_DSP)
{
    double new_T = T*(1-T_DSP);
    double new_t = t-T*T_DSP/2;
    double omega = 2*PI/new_T;
    
    if (t > T*T_DSP/2 && t <= T*(1-T_DSP/2))
    {
        return 0.5*Wheight*(1-cos(omega*new_t));
    }
    else
    {
        return 0;
    }
}
double WalkingTrajectory::sinh(double x)
{
    return (double)(exp(x)-exp(-x))/2;
}
double WalkingTrajectory::cosh(double x)
{
    return (double)(exp(x)+exp(-x))/2;
}
//----------------------------------------------------