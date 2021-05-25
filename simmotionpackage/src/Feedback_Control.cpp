#include "simmotionpackage/Feedback_Control.h"


extern struct Points_Struct Points;
BalanceControl balance;
WalkingGaitByLIPM walkinggait;
SimIMUData sim_imu_data;



BalanceControl::BalanceControl()
{
	
	original_ik_point_rz_ = 0.0;
	original_ik_point_lz_ = 0.0;
	sup_foot_ = doublefeet;

	name_cont_ = 0;


	ZMP_process = new ZMPProcess;
	initialize(30);
}

BalanceControl::~BalanceControl()
{
	delete ZMP_process;

}
void BalanceControl::initialize(const int control_cycle_msec)
{
	for(int i = 0; i < sizeof(init_imu_value)/sizeof(init_imu_value[0]); i++)
        init_imu_value[i].initialize();
    for(int i = 0; i < sizeof(pres_imu_value)/sizeof(pres_imu_value[0]); i++)
        pres_imu_value[i].initialize();
    for(int i = 0; i < sizeof(prev_imu_value)/sizeof(prev_imu_value[0]); i++)
        prev_imu_value[i].initialize();
    for(int i = 0; i < sizeof(ideal_imu_value)/sizeof(ideal_imu_value[0]); i++)
        ideal_imu_value[i].initialize();
    for(int i = 0; i < sizeof(passfilter_pres_imu_value)/sizeof(passfilter_pres_imu_value[0]); i++)
        passfilter_pres_imu_value[i].initialize();
    for(int i = 0; i < sizeof(passfilter_prev_imu_value)/sizeof(passfilter_prev_imu_value[0]); i++)
        passfilter_prev_imu_value[i].initialize();

    leftfoot_hip_roll_value.initialize();
    leftfoot_hip_pitch_value.initialize();
	rightfoot_hip_roll_value.initialize();
    rightfoot_hip_pitch_value.initialize();
	leftfoot_ankle_roll_value.initialize();
    leftfoot_ankle_pitch_value.initialize();
	rightfoot_ankle_roll_value.initialize();
    rightfoot_ankle_pitch_value.initialize();
	
	CoM_EPx_value.initialize();

    PIDleftfoot_hip_roll.initParam();
    PIDleftfoot_hip_pitch.initParam();
	PIDrightfoot_hip_roll.initParam();
    PIDrightfoot_hip_pitch.initParam();
	PIDleftfoot_ankle_roll.initParam();
	PIDleftfoot_ankle_pitch.initParam();
	PIDrightfoot_ankle_roll.initParam();
	PIDrightfoot_ankle_pitch.initParam();
	
	PIDleftfoot_zmp_x.initParam();
	PIDleftfoot_zmp_y.initParam();
	PIDrightfoot_zmp_x.initParam();
	PIDrightfoot_zmp_y.initParam();
	PIDCoM_x.initParam();

    for(int i = 0; i < sizeof(butterfilter_imu)/sizeof(butterfilter_imu[0]); i++)
        butterfilter_imu[i].initialize();
	//hip
    PIDleftfoot_hip_roll.setValueLimit(150, -150);
    PIDleftfoot_hip_pitch.setValueLimit(150, -150);
    PIDleftfoot_hip_roll.setKpid(0.15, 0.04, 0);
    PIDleftfoot_hip_pitch.setKpid(0.1, 0, 0);
    PIDleftfoot_hip_roll.setControlGoal(init_imu_value[(int)imu::roll].pos);
    PIDleftfoot_hip_pitch.setControlGoal(init_imu_value[(int)imu::pitch].pos);

	PIDrightfoot_hip_roll.setValueLimit(150, -150);
    PIDrightfoot_hip_pitch.setValueLimit(150, -150);
    PIDrightfoot_hip_roll.setKpid(0.15, 0.04, 0);
    PIDrightfoot_hip_pitch.setKpid(0.1, 0, 0);
    PIDrightfoot_hip_roll.setControlGoal(init_imu_value[(int)imu::roll].pos);
    PIDrightfoot_hip_pitch.setControlGoal(init_imu_value[(int)imu::pitch].pos);

	//ankle
    PIDleftfoot_ankle_roll.setValueLimit(60, -60);
    PIDleftfoot_ankle_pitch.setValueLimit(120, -120);
    PIDleftfoot_ankle_roll.setKpid(0.05, 0, 0);
    PIDleftfoot_ankle_pitch.setKpid(0.015, 0, 0);
    PIDleftfoot_ankle_roll.setControlGoal(init_imu_value[(int)imu::roll].pos);
    PIDleftfoot_ankle_pitch.setControlGoal(init_imu_value[(int)imu::pitch].pos);

	PIDrightfoot_ankle_roll.setValueLimit(60, -60);
    PIDrightfoot_ankle_pitch.setValueLimit(120, -120);
    PIDrightfoot_ankle_roll.setKpid(0.03, 0, 0);
    PIDrightfoot_ankle_pitch.setKpid(0.015, 0, 0);
    PIDrightfoot_ankle_roll.setControlGoal(init_imu_value[(int)imu::roll].pos);
    PIDrightfoot_ankle_pitch.setControlGoal(init_imu_value[(int)imu::pitch].pos);

	PIDleftfoot_zmp_x.setValueLimit(7, -7);
	PIDleftfoot_zmp_y.setValueLimit(7, -7);
	PIDleftfoot_zmp_x.setKpid(0.0125, 0, 0);
	PIDleftfoot_zmp_y.setKpid(0.0125, 0, 0);
	PIDleftfoot_zmp_x.setControlGoal(0);
	PIDleftfoot_zmp_y.setControlGoal(4.5);

	PIDrightfoot_zmp_x.setValueLimit(7, -7);
	PIDrightfoot_zmp_y.setValueLimit(7, -7);
	PIDrightfoot_zmp_x.setKpid(0.0125, 0, 0);
	PIDrightfoot_zmp_y.setKpid(0.0125, 0, 0);
	PIDrightfoot_zmp_x.setControlGoal(0);
	PIDrightfoot_zmp_y.setControlGoal(-4.5);
	
	PIDCoM_x.setValueLimit(10, -10);
	PIDCoM_x.setKpid(0.015, 0, 0);
	PIDCoM_x.setControlGoal(0);

	leftfoot_hip_roll = 0;
    leftfoot_hip_pitch = 0;
    leftfoot_ankle_roll = 0;
    leftfoot_ankle_pitch = 0;
	rightfoot_hip_roll = 0;
    rightfoot_hip_pitch = 0;
    rightfoot_ankle_roll = 0;
    rightfoot_ankle_pitch = 0;

	for(int i = 0; i < 3; i++)init_imu_value[i].pos = sim_imu_data.sensor_rpy[i];


	std::vector<float> temp;
	if(map_roll.empty())
	{
		map_roll["init_roll_pos"] = temp;
        map_roll["smaple_times_count"] = temp;
        map_roll["pres_roll_pos"] = temp;
        map_roll["passfilter_pres_roll_pos"] = temp;
        map_roll["ideal_roll_vel"] = temp;
        map_roll["pres_roll_vel"] = temp;
        map_roll["passfilter_pres_roll_vel"] = temp;
        map_roll["left_control_once_roll"] = temp;
        map_roll["left_control_total_roll"] = temp;
		map_roll["right_control_once_roll"] = temp;
        map_roll["right_control_total_roll"] = temp;
	}

	if(map_pitch.empty())
	{
		map_pitch["init_pitch_pos"] = temp;
        map_pitch["smaple_times_count"] = temp;
        map_pitch["pres_pitch_pos"] = temp;
        map_pitch["passfilter_pres_pitch_pos"] = temp;
        map_pitch["ideal_pitch_vel"] = temp;
        map_pitch["pres_pitch_vel"] = temp;
        map_pitch["passfilter_pres_pitch_vel"] = temp;
        map_pitch["left_control_once_pitch"] = temp;
        map_pitch["left_control_total_pitch"] = temp;
		map_pitch["right_control_once_pitch"] = temp;
        map_pitch["right_control_total_pitch"] = temp;
	}

	if(map_ZMP.empty())
	{
		map_ZMP["pres_ZMP_left_pos_x"] = temp;
        map_ZMP["pres_ZMP_left_pos_y"] = temp;
        map_ZMP["pres_ZMP_right_pos_x"] = temp;
        map_ZMP["pres_ZMP_right_pos_y"] = temp;
        map_ZMP["pres_ZMP_feet_pos_x"] = temp;
        map_ZMP["pres_ZMP_feet_pos_y"] = temp;

        map_ZMP["delta_v"] = temp;

        map_ZMP["raw_sensor_data_0"] = temp;
        map_ZMP["raw_sensor_data_1"] = temp;
        map_ZMP["raw_sensor_data_2"] = temp;
        map_ZMP["raw_sensor_data_3"] = temp;
        map_ZMP["raw_sensor_data_4"] = temp;
        map_ZMP["raw_sensor_data_5"] = temp;
        map_ZMP["raw_sensor_data_6"] = temp;
        map_ZMP["raw_sensor_data_7"] = temp;

        map_ZMP["sensor_force_0"] = temp;
        map_ZMP["sensor_force_1"] = temp;
        map_ZMP["sensor_force_2"] = temp;
        map_ZMP["sensor_force_3"] = temp;
        map_ZMP["sensor_force_4"] = temp;
        map_ZMP["sensor_force_5"] = temp;
        map_ZMP["sensor_force_6"] = temp;
        map_ZMP["sensor_force_7"] = temp;

		map_ZMP["leftfoot_control_once_EPx"] = temp;
		map_ZMP["leftfoot_control_once_EPy"] = temp;
		map_ZMP["leftfoot_control_total_EPx"] = temp;
		map_ZMP["leftfoot_control_total_EPy"] = temp;

		map_ZMP["rightfoot_control_once_EPx"] = temp;
		map_ZMP["rightfoot_control_once_EPy"] = temp;
		map_ZMP["rightfoot_control_total_EPx"] = temp;
		map_ZMP["rightfoot_control_total_EPy"] = temp;

		map_ZMP["new_EP_lx"] = temp;
		map_ZMP["new_EP_rx"] = temp;
		map_ZMP["new_EP_ly"] = temp;
		map_ZMP["new_EP_ry"] = temp;
	}

	if(map_CoM.empty())
	{
		map_CoM["CoM_x_control"] = temp;

		map_CoM["new_EP_lx"] = temp;
		map_CoM["new_EP_rx"] = temp;
	}

	if(map_Accel.empty())
	{
		map_Accel["Accel_ax"] = temp;
		map_Accel["Accel_ay"] = temp;
		map_Accel["Accel_az"] = temp;
	}
	map_roll.find("init_roll_pos")->second.push_back(init_imu_value[(int)imu::roll].pos);
	map_pitch.find("init_pitch_pos")->second.push_back(init_imu_value[(int)imu::pitch].pos);
}

void BalanceControl::get_sensor_value()
{
	int i;
	double rpy_radian[3] = {0};
	for(int i=0; i<3; i++)
		rpy_radian[i] = sim_imu_data.sensor_rpy[i]* DEGREE2RADIAN;
	
	roll_imu_filtered_ = roll_imu_lpf_.get_filtered_output(rpy_radian[0]);
	pitch_imu_filtered_ = pitch_imu_lpf_.get_filtered_output(rpy_radian[1]);
// 	// roll_over_limit_ = (fabs(roll_imu_filtered_) > 0.209 ? true : false);
// 	// pitch_over_limit_ = (fabs(pitch_imu_filtered_) > 0.297 ? true : false);
// // 	two_feet_grounded_ = (original_ik_point_rz_ == original_ik_point_lz_ ? true : false);

	//cog_roll_offset_ = sensor.imu_desire_[0] * DEGREE2RADIAN;
	//cog_pitch_offset_ = sensor.imu_desire_[1] * DEGREE2RADIAN;
	cog_roll_offset_=0;
	cog_pitch_offset_=0;
	double cog_y_filtered = roll_imu_filtered_ - cog_roll_offset_;
	double cog_x_filtered = pitch_imu_filtered_ - cog_pitch_offset_;
	if(Points.Inverse_PointR_Z != Points.Inverse_PointL_Z)
	{
		foot_cog_x_ = 26.2 * sin(cog_x_filtered);
		foot_cog_y_ = 26.2 * sin(cog_y_filtered);
	}
	else
	{
		foot_cog_x_ = 0;
		foot_cog_y_ = 0;
	}

// //     if( (fabs( rpy_radian[0] ) > RPY_ROLL_LIMIT || ( rpy_radian[1] ) > RPY_PITCH_LIMIT) && sensor.fall_Down_Flag_ == false) //over plus limit >> forward fall down
// //     {
// // 		if(sensor.fall_Down_Status_ == 'F')
// // 		{
// // 			gettimeofday(&timer_end_, NULL);
// // 			sensor.fall_Down_Status_ = 'F';
// // 		}
// // 		else
// // 		{
// // 			gettimeofday(&timer_start_, NULL);
// // 			sensor.fall_Down_Status_ = 'F';			
// // 		}

// //     }
// // 	else if(  ( rpy_radian[1] ) < -(RPY_PITCH_LIMIT)  && sensor.fall_Down_Flag_ == false) //over minus limit >> backward fall down
// //     {
// // 		if(sensor.fall_Down_Status_ == 'B')
// // 		{
// // 			gettimeofday(&timer_end_, NULL);
// // 			sensor.fall_Down_Status_ = 'B';
// // 		}
// // 		else
// // 		{
// // 			gettimeofday(&timer_start_, NULL);
// // 			sensor.fall_Down_Status_ = 'B';			
// // 		}
// //     }
// // 	else if( (fabs( rpy_radian[0] ) < RPY_STAND_RANGE && fabs( rpy_radian[1] ) < RPY_STAND_RANGE) && sensor.fall_Down_Flag_ == true) 
// //     {
// // 		if(sensor.fall_Down_Status_ == 'S')
// // 		{
// // 			gettimeofday(&timer_end_, NULL);
// // 			sensor.fall_Down_Status_ = 'S';
// // 		}
// // 		else
// // 		{
// // 			gettimeofday(&timer_start_, NULL);
// // 			sensor.fall_Down_Status_ = 'S';			
// // 		}	
// //     }
// // 	else
// // 	{
// // 		gettimeofday(&timer_start_, NULL);
// // 		sensor.fall_Down_Status_ = sensor.fall_Down_Status_ ;
// // 	}
	
// // 	timer_dt_ = (double)(1000000.0 * (timer_end_.tv_sec - timer_start_.tv_sec) + (timer_end_.tv_usec - timer_start_.tv_usec));	

// // 	if(timer_dt_ >= 2000000.0 && (sensor.fall_Down_Status_ == 'F' || sensor.fall_Down_Status_ == 'B') )//timer_dt_ unit is us 1sec = 1000000
// // 	{
// // 		sensor.fall_Down_Flag_ = true;
// // 	}
// // 	else if(timer_dt_ >= 4000000.0 && sensor.fall_Down_Status_ == 'S')
// // 	{
// // 		sensor.fall_Down_Flag_ = false;
// // 	}
// // 	else
// // 	{
// // 		sensor.fall_Down_Flag_ = sensor.fall_Down_Flag_;		
// // 	}


}

void BalanceControl::setSupportFoot()
{
	
	original_ik_point_rz_ = Points.Inverse_PointR_Z ;
	original_ik_point_lz_ = Points.Inverse_PointL_Z;
	pre_sup_foot_ = sup_foot_;
	if(Points.Inverse_PointR_Z != Points.Inverse_PointL_Z)
	{
		if(Points.Inverse_PointR_Y > 0)
			sup_foot_ = rightfoot;
		else if(Points.Inverse_PointR_Y < 0)
			sup_foot_ = leftfoot;
	}
	else
	{
		sup_foot_ = doublefeet;
	}
}

void BalanceControl::resetControlValue()
{
	leftfoot_hip_pitch_value.initialize();
	leftfoot_hip_roll_value.initialize();
	rightfoot_hip_pitch_value.initialize();
	rightfoot_hip_roll_value.initialize();

	leftfoot_EPx_value.initialize();
	leftfoot_EPy_value.initialize();
	rightfoot_EPx_value.initialize();
	rightfoot_EPy_value.initialize();
}

void BalanceControl::balance_control()
{
	int i;

	//get accel for test and save data 
	map_Accel.find("Accel_ax")->second.push_back(sim_imu_data.a_x);
	map_Accel.find("Accel_ay")->second.push_back(sim_imu_data.a_y);
	map_Accel.find("Accel_az")->second.push_back(sim_imu_data.a_z);

	// original_ik_point_rz_ = parameterinfo->points.IK_Point_RZ;
	// original_ik_point_lz_ = parameterinfo->points.IK_Point_LZ;

	for(i=0; i<3; i++)prev_imu_value[i].pos = pres_imu_value[i].pos;
    for(i=0; i<3; i++)pres_imu_value[i].pos = sim_imu_data.sensor_rpy[i];
	//check if switch foot or not
	// if((pre_sup_foot_ != sup_foot_) && (sup_foot_ != doublefeet))
	// {
	//		
	// }

	//----------- pitch ---------------------
	pres_imu_value[(int)imu::pitch].vel = (pres_imu_value[(int)imu::pitch].pos-prev_imu_value[(int)imu::pitch].pos)/(0.03);
	passfilter_pres_imu_value[(int)imu::pitch].pos = butterfilter_imu[(int)imu::pitch].pos.getValue(pres_imu_value[(int)imu::pitch].pos);
	passfilter_pres_imu_value[(int)imu::pitch].vel = butterfilter_imu[(int)imu::pitch].vel.getValue(pres_imu_value[(int)imu::pitch].vel);
	passfilter_prev_imu_value[(int)imu::pitch] = passfilter_pres_imu_value[(int)imu::pitch];
	
	//----------- roll ----------------------
	pres_imu_value[(int)imu::roll].vel = (pres_imu_value[(int)imu::roll].pos-prev_imu_value[(int)imu::roll].pos)/(0.03);
	passfilter_pres_imu_value[(int)imu::roll].pos = butterfilter_imu[(int)imu::roll].pos.getValue(pres_imu_value[(int)imu::roll].pos);
	passfilter_pres_imu_value[(int)imu::roll].vel = butterfilter_imu[(int)imu::roll].vel.getValue(pres_imu_value[(int)imu::roll].vel);
	passfilter_prev_imu_value[(int)imu::roll] = passfilter_pres_imu_value[(int)imu::roll];

	PIDleftfoot_hip_pitch.setControlGoal(ideal_imu_value[(int)imu::pitch].vel);
	PIDleftfoot_hip_roll.setControlGoal(ideal_imu_value[(int)imu::roll].vel);
	PIDleftfoot_hip_pitch.setControlGoal(ideal_imu_value[(int)imu::pitch].vel);
	PIDleftfoot_hip_roll.setControlGoal(ideal_imu_value[(int)imu::roll].vel);

	CoM_EPx_value.control_value_once = PIDCoM_x.calculateExpValue(passfilter_pres_imu_value[(int)imu::pitch].pos);

	// parameterinfo->points.IK_Point_LX -= CoM_EPx_value.control_value_once;
	// parameterinfo->points.IK_Point_RX -= CoM_EPx_value.control_value_once;
	if(sup_foot_ == leftfoot)
	{
		//sup
		//----------- pitch ---------------------
		leftfoot_hip_pitch_value.control_value_once = PIDleftfoot_hip_pitch.calculateExpValue(passfilter_pres_imu_value[(int)imu::pitch].vel)*0.03;//dt = 0.03
		leftfoot_hip_pitch_value.control_value_total -= leftfoot_hip_pitch_value.control_value_once;
		leftfoot_hip_pitch = leftfoot_hip_pitch_value.control_value_total/180.0*PI;

		leftfoot_ankle_pitch_value.control_value_once = PIDleftfoot_ankle_pitch.calculateExpValue(passfilter_pres_imu_value[(int)imu::pitch].vel)*0.03;//dt = 0.03
		// leftfoot_ankle_pitch_value.control_value_once = PIDleftfoot_ankle_pitch.calculateExpValue(foot_cog_x_)*0.03;//dt = 0.03
		leftfoot_ankle_pitch_value.control_value_total += leftfoot_ankle_pitch_value.control_value_once;
		leftfoot_ankle_pitch -= leftfoot_ankle_pitch_value.control_value_total/180.0*PI;
		//----------- roll ----------------------
		leftfoot_hip_roll_value.control_value_once = PIDleftfoot_hip_roll.calculateExpValue_roll(passfilter_pres_imu_value[(int)imu::roll].vel)*0.03;//dt = 0.03;
		leftfoot_hip_roll_value.control_value_total += leftfoot_hip_roll_value.control_value_once;
		leftfoot_hip_roll = leftfoot_hip_roll_value.control_value_total/180.0*PI;
		
		leftfoot_ankle_roll_value.control_value_once = PIDleftfoot_ankle_roll.calculateExpValue_roll(passfilter_pres_imu_value[(int)imu::roll].vel)*0.03;//dt = 0.03;
		// leftfoot_ankle_roll_value.control_value_once = PIDleftfoot_ankle_roll.calculateExpValue_roll(foot_cog_y_)*0.03;//dt = 0.03;
		leftfoot_ankle_roll_value.control_value_total += leftfoot_ankle_roll_value.control_value_once;
		leftfoot_ankle_roll -= leftfoot_ankle_roll_value.control_value_total/180.0*PI;

		//swing
		rightfoot_hip_pitch_value.initialize();
		rightfoot_hip_roll_value.initialize();
		rightfoot_ankle_pitch_value.initialize();
		rightfoot_ankle_roll_value.initialize();
		rightfoot_hip_pitch = 0;//-= rightfoot_hip_pitch_value.control_value_total/180.0*PI;
		rightfoot_hip_roll = 0;//+= rightfoot_hip_roll_value.control_value_total/180.0*PI;
		rightfoot_ankle_pitch = 0;
		rightfoot_ankle_roll = 0;
		
		// Points.Inverse_PointL_X -= CoM_EPx_value.control_value_once;//CoM點控制 左腳
	}
	else if(sup_foot_ == rightfoot)
	{
		//sup
		//----------- pitch ---------------------
		rightfoot_hip_pitch_value.control_value_once = PIDleftfoot_hip_pitch.calculateExpValue(passfilter_pres_imu_value[(int)imu::pitch].vel)*0.03;//dt = 0.03
		rightfoot_hip_pitch_value.control_value_total -= rightfoot_hip_pitch_value.control_value_once;
		rightfoot_hip_pitch = rightfoot_hip_pitch_value.control_value_total/180.0*PI;
		
		rightfoot_ankle_pitch_value.control_value_once = PIDleftfoot_ankle_pitch.calculateExpValue(passfilter_pres_imu_value[(int)imu::pitch].vel)*0.03;//dt = 0.03
		// rightfoot_ankle_pitch_value.control_value_once = PIDleftfoot_ankle_pitch.calculateExpValue(foot_cog_x_)*0.03;
		rightfoot_ankle_pitch_value.control_value_total += rightfoot_ankle_pitch_value.control_value_once;
		rightfoot_ankle_pitch -= rightfoot_ankle_pitch_value.control_value_total/180.0*PI;
		//----------- roll ----------------------
		rightfoot_hip_roll_value.control_value_once = PIDleftfoot_hip_roll.calculateExpValue_roll(passfilter_pres_imu_value[(int)imu::roll].vel)*0.03;//dt = 0.03;
		rightfoot_hip_roll_value.control_value_total += rightfoot_hip_roll_value.control_value_once;
		rightfoot_hip_roll = rightfoot_hip_roll_value.control_value_total/180.0*PI;
		
		rightfoot_ankle_roll_value.control_value_once = PIDleftfoot_ankle_roll.calculateExpValue_roll(passfilter_pres_imu_value[(int)imu::roll].vel)*0.03;//dt = 0.03;
		// rightfoot_ankle_roll_value.control_value_once = PIDleftfoot_ankle_roll.calculateExpValue_roll(foot_cog_y_)*0.03;//dt = 0.03;
		rightfoot_ankle_roll_value.control_value_total += rightfoot_ankle_roll_value.control_value_once;
		rightfoot_ankle_roll -= rightfoot_ankle_roll_value.control_value_total/180.0*PI;

		//swing
		leftfoot_hip_pitch_value.initialize();
		leftfoot_hip_roll_value.initialize();
		leftfoot_ankle_pitch_value.initialize();
		leftfoot_ankle_roll_value.initialize();
		leftfoot_hip_pitch = 0;//-= rightfoot_hip_pitch_value.control_value_total/180.0*PI;
		leftfoot_hip_roll = 0;//+= rightfoot_hip_roll_value.control_value_total/180.0*PI;
		leftfoot_ankle_pitch = 0;
		leftfoot_ankle_roll = 0;

		// Points.Inverse_PointR_X -= CoM_EPx_value.control_value_once;//CoM點控制 右腳
	}
	// else if(sup_foot_ == doublefeet)
	// {
	// 	leftfoot_hip_pitch_value.initialize();
	// 	leftfoot_hip_roll_value.initialize();
	// 	rightfoot_hip_pitch_value.initialize();
	// 	rightfoot_hip_roll_value.initialize();
	// }
	map_roll.find("left_control_once_roll")->second.push_back(leftfoot_hip_roll_value.control_value_once);
	map_roll.find("left_control_total_roll")->second.push_back(leftfoot_hip_roll_value.control_value_total);
	map_roll.find("right_control_once_roll")->second.push_back(rightfoot_hip_roll_value.control_value_once);
	map_roll.find("right_control_total_roll")->second.push_back(rightfoot_hip_roll_value.control_value_total);
	map_roll.find("smaple_times_count")->second.push_back(30);
    map_roll.find("pres_roll_pos")->second.push_back(pres_imu_value[(int)imu::roll].pos);	
    map_roll.find("passfilter_pres_roll_pos")->second.push_back(passfilter_pres_imu_value[(int)imu::roll].pos);
	map_roll.find("ideal_roll_vel")->second.push_back(ideal_imu_value[(int)imu::roll].vel);
    map_roll.find("pres_roll_vel")->second.push_back(pres_imu_value[(int)imu::roll].vel);
    map_roll.find("passfilter_pres_roll_vel")->second.push_back(passfilter_pres_imu_value[(int)imu::roll].vel);
	map_pitch.find("left_control_once_pitch")->second.push_back(leftfoot_hip_pitch_value.control_value_once);
	map_pitch.find("left_control_total_pitch")->second.push_back(leftfoot_hip_pitch_value.control_value_total);
	map_pitch.find("right_control_once_pitch")->second.push_back(rightfoot_hip_pitch_value.control_value_once);
	map_pitch.find("right_control_total_pitch")->second.push_back(rightfoot_hip_pitch_value.control_value_total);
	map_pitch.find("smaple_times_count")->second.push_back(30);
    map_pitch.find("pres_pitch_pos")->second.push_back(pres_imu_value[(int)imu::pitch].pos);
    map_pitch.find("passfilter_pres_pitch_pos")->second.push_back(passfilter_pres_imu_value[(int)imu::pitch].pos);
    map_pitch.find("ideal_pitch_vel")->second.push_back(ideal_imu_value[(int)imu::pitch].vel);
    map_pitch.find("pres_pitch_vel")->second.push_back(pres_imu_value[(int)imu::pitch].vel);
    map_pitch.find("passfilter_pres_pitch_vel")->second.push_back(passfilter_pres_imu_value[(int)imu::pitch].vel);

	map_CoM.find("CoM_x_control")->second.push_back(CoM_EPx_value.control_value_once);
	map_CoM.find("new_EP_lx")->second.push_back(Points.Inverse_PointL_X);
	map_CoM.find("new_EP_rx")->second.push_back(Points.Inverse_PointR_X);
}

void BalanceControl::endPointControl()
{
	// int raw_sensor_data_tmp[8];
	// for(int i=0; i<4; i++)raw_sensor_data_tmp[i] = sensor.press_left_[i];
	// for(int i=4; i<8; i++)raw_sensor_data_tmp[i] = sensor.press_right_[i-4];
	// prev_ZMP = pres_ZMP;
	// ZMP_process->setpOrigenSensorData(raw_sensor_data_tmp);
	// pres_ZMP = ZMP_process->getZMPValue();

	// // map_ZMP.find("pres_ZMP_left_pos_x")->second.push_back(pres_ZMP.left_pos.x);
	// // map_ZMP.find("pres_ZMP_left_pos_y")->second.push_back(pres_ZMP.left_pos.y);
	// // map_ZMP.find("pres_ZMP_right_pos_x")->second.push_back(pres_ZMP.right_pos.x);
	// // map_ZMP.find("pres_ZMP_right_pos_y")->second.push_back(pres_ZMP.right_pos.y);
	// // map_ZMP.find("pres_ZMP_feet_pos_x")->second.push_back(pres_ZMP.feet_pos.x);
	// // map_ZMP.find("pres_ZMP_feet_pos_y")->second.push_back(pres_ZMP.feet_pos.y);
	
	// pres_ZMP.feet_pos.x = foot_cog_x_;
	// pres_ZMP.feet_pos.y = foot_cog_y_;

	// double *sensor_force = ZMP_process->getpSensorForce();
	// int *raw_sensor_data = ZMP_process->getpOrigenSensorData();

	// map_ZMP.find("sensor_force_0")->second.push_back(sensor_force[0]);
	// map_ZMP.find("sensor_force_1")->second.push_back(sensor_force[1]);
	// map_ZMP.find("sensor_force_2")->second.push_back(sensor_force[2]);
	// map_ZMP.find("sensor_force_3")->second.push_back(sensor_force[3]);
	// map_ZMP.find("sensor_force_4")->second.push_back(sensor_force[4]);
	// map_ZMP.find("sensor_force_5")->second.push_back(sensor_force[5]);
	// map_ZMP.find("sensor_force_6")->second.push_back(sensor_force[6]);
	// map_ZMP.find("sensor_force_7")->second.push_back(sensor_force[7]);

	// map_ZMP.find("raw_sensor_data_0")->second.push_back(raw_sensor_data[0]);
	// map_ZMP.find("raw_sensor_data_1")->second.push_back(raw_sensor_data[1]);
	// map_ZMP.find("raw_sensor_data_2")->second.push_back(raw_sensor_data[2]);
	// map_ZMP.find("raw_sensor_data_3")->second.push_back(raw_sensor_data[3]);
	// map_ZMP.find("raw_sensor_data_4")->second.push_back(raw_sensor_data[4]);
	// map_ZMP.find("raw_sensor_data_5")->second.push_back(raw_sensor_data[5]);
	// map_ZMP.find("raw_sensor_data_6")->second.push_back(raw_sensor_data[6]);
	// map_ZMP.find("raw_sensor_data_7")->second.push_back(raw_sensor_data[7]);

	// if(sup_foot_ == leftfoot)
	// {
	// 	leftfoot_EPx_value.control_value_once = PIDleftfoot_zmp_x.calculateExpValue(pres_ZMP.feet_pos.x);
	// 	leftfoot_EPx_value.control_value_total += leftfoot_EPx_value.control_value_once;

	// 	// parameterinfo->points.IK_Point_LX += leftfoot_EPx_value.control_value_total;
	// 	// parameterinfo->points.IK_Point_RX += leftfoot_EPx_value.control_value_total;

	// 	rightfoot_EPx_value.initialize();
	// 	rightfoot_EPy_value.initialize();
	// }
	// else if(sup_foot_ == rightfoot)
	// {
	// 	rightfoot_EPx_value.control_value_once = PIDleftfoot_zmp_x.calculateExpValue(pres_ZMP.feet_pos.x);
	// 	rightfoot_EPx_value.control_value_total += rightfoot_EPx_value.control_value_once;

	// 	// parameterinfo->points.IK_Point_LX += rightfoot_EPx_value.control_value_total;
	// 	// parameterinfo->points.IK_Point_RX += rightfoot_EPx_value.control_value_total;

	// 	leftfoot_EPx_value.initialize();
	// 	leftfoot_EPy_value.initialize();
	// }

	// map_ZMP.find("leftfoot_control_once_EPx")->second.push_back(leftfoot_EPx_value.control_value_once);
	// map_ZMP.find("leftfoot_control_total_EPx")->second.push_back(leftfoot_EPx_value.control_value_total);
	// map_ZMP.find("rightfoot_control_once_EPx")->second.push_back(rightfoot_EPx_value.control_value_once);
	// map_ZMP.find("rightfoot_control_total_EPx")->second.push_back(rightfoot_EPx_value.control_value_total);

	// // float LIPM_vel = (float)(-(supfoot_EPx_value.control_value_total/walkinggait.Tc_));

	// map_ZMP.find("new_EP_lx")->second.push_back(parameterinfo->points.IK_Point_LX);
	// map_ZMP.find("new_EP_rx")->second.push_back(parameterinfo->points.IK_Point_RX);
	// map_ZMP.find("new_EP_ly")->second.push_back(parameterinfo->points.IK_Point_LY);
	// map_ZMP.find("new_EP_ry")->second.push_back(oints.Inverse_PointR_Y);
}

float BalanceControl::calculateCOMPosbyLIPM(float pos_adj, float vel)
{
	// return pos_adj*walkinggait.cosh(walkinggait.t_/walkinggait.Tc_)+walkinggait.Tc_*vel*walkinggait.sinh(walkinggait.t_/walkinggait.Tc_);
	return pos_adj*(walkinggait.cosh(walkinggait.t_/walkinggait.Tc_)-walkinggait.sinh(walkinggait.t_/walkinggait.Tc_));
}

void BalanceControl::control_after_ik_calculation()
{	//ROS_INFO("\n\n\n\n\n\n\n%f\n%f\n%f\n%f\n\n\n%f\n%f\n%f\n\n\n%f\n%f\n%f\n",sim_imu_data.qx,sim_imu_data.qy,sim_imu_data.qz,sim_imu_data.qw,sim_imu_data.g_x,sim_imu_data.g_y,sim_imu_data.g_z,sim_imu_data.a_x,sim_imu_data.a_y,sim_imu_data.a_z);

	if(sup_foot_ == leftfoot)
	{
		Points.Thta[10] += leftfoot_hip_roll;
		Points.Thta[11] += leftfoot_hip_pitch;
		// Points.Thta[13] += leftfoot_ankle_pitch;
		// Points.Thta[14] += leftfoot_ankle_roll;


		Points.Thta[16] += rightfoot_hip_roll;
		Points.Thta[17] += rightfoot_hip_pitch;
		// Points.Thta[19] += rightfoot_ankle_pitch;
		// Points.Thta[20] += rightfoot_ankle_roll;

		// if(leftfoot_hip_roll > 0)
		// 	Points.Thta[5] += leftfoot_hip_roll;
		// else
		// 	Points.Thta[1] += leftfoot_hip_roll;
	}
	else if(sup_foot_ == rightfoot)
	{
		Points.Thta[10] += leftfoot_hip_roll;
		Points.Thta[11] += leftfoot_hip_pitch;
		// Points.Thta[13] += leftfoot_ankle_pitch;
		// Points.Thta[14] += leftfoot_ankle_roll;

		Points.Thta[16] += rightfoot_hip_roll;
		Points.Thta[17] += rightfoot_hip_pitch;
		// Points.Thta[19] += rightfoot_ankle_pitch;
		// Points.Thta[20] += rightfoot_ankle_roll;

		// if(rightfoot_hip_roll > 0)
		// 	Points.Thta[5] += rightfoot_hip_roll;
		// else
		// 	Points.Thta[1] += rightfoot_hip_roll;
	}

	// compensate
	// double gain = 1.05;	
	if(Points.Inverse_PointR_Y < 0 && (original_ik_point_rz_ != original_ik_point_lz_))
	{	
		// Points.Thta[10] = PI_2 + (Points.Thta[10] - PI_2) *2;
		// Points.Thta[16] = PI_2 + (Points.Thta[16] - PI_2) *2;			
		
		
	} 
	else if(Points.Inverse_PointR_Y > 0 && (original_ik_point_rz_ != original_ik_point_lz_))
	{	
		// Points.Thta[10] = PI_2 + (Points.Thta[10] - PI_2) *2;
		// Points.Thta[16] = PI_2 + (Points.Thta[16] - PI_2) *2;	

		
	}
}	
PID_Controller::PID_Controller(float Kp, float Ki, float Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->error = 0;
    this->pre_error = 0;
    this->errors = 0;
    this->errord = 0;
    this->x1c = 0;
    this->x2c = 0;
    this->x3c = 0;
    this->exp_value = 0;
    this->value = 0;
    this->pre_value = 0;
    this->upper_limit = 0;
    this->lower_limit = 0;
}

PID_Controller::PID_Controller()
{
    this->Kp = 0;
    this->Ki = 0;
    this->Kd = 0;
    this->error = 0;
    this->pre_error = 0;
    this->errors = 0;
    this->errord = 0;
    this->x1c = 0;
    this->x2c = 0;
    this->x3c = 0;
    this->exp_value = 0;
    this->value = 0;
    this->pre_value = 0;
}

PID_Controller::~PID_Controller()
{
    
}

void PID_Controller::initParam()
{
    this->pre_error = 0;
    this->error = 0;
    this->errors = 0;
    this->errord = 0;
    this->exp_value = 0;
    this->value = 0;
    this->pre_value = 0;
}

void PID_Controller::setKpid(double Kp, double Ki, double Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void PID_Controller::setControlGoal(float x1c, float x2c, float x3c)
{
    this->x1c = x1c;
    this->x2c = x2c;
    this->x3c = x3c;
}

float PID_Controller::calculateExpValue(float value)//Expected value
{
    this->pre_value = this->value;
    this->value = value;
    this->pre_error = this->error;
    this->error = this->x1c - this->value;
    this->errors += this->error*0.03;
    if(this->pre_error == 0)
    {
        this->errord = 0;
    } 
    else
    {
        this->errord = (this->error - this->pre_error)/0.03;
    }
    this->exp_value = this->Kp*this->error + this->Ki*this->errors + this->Kd*this->errord;
    if(this->exp_value > this->upper_limit)
    {
        return this->upper_limit;
    }
    else if(this->exp_value < this->lower_limit)
    {
        return this->lower_limit;
    }
    else
    {
        return this->exp_value;
    }
}

float PID_Controller::calculateExpValue_roll(float value)//Expected value
{
	int non_control_area = 8;
	if(fabs(value) < 5)
	{	
		value = 0;
	}
	else
	{
		if(value <= non_control_area && value >= -non_control_area)
		{
    		this->error = this->x1c - value;
		}
		else
		{
			if(value<-non_control_area)
			{
    			this->error = (this->x1c - non_control_area) - value;
			}
			else if(value > non_control_area)
			{
			    this->error = (this->x1c + non_control_area) - value;	
			}
			else
			{
				this->error = this->x1c - value;
			}
		}
		
	}
	

    this->pre_value = this->value;
    this->value = value;
    this->pre_error = this->error;
    // this->error = this->x1c - this->value;
    this->errors += this->error*0.03;
    if(this->pre_error == 0)
    {
        this->errord = 0;
    } 
    else
    {
        this->errord = (this->error - this->pre_error)/0.03;
    }
    this->exp_value = this->Kp*this->error + this->Ki*this->errors + this->Kd*this->errord;
    if(this->exp_value > this->upper_limit)
    {
        return this->upper_limit;
    }
    else if(this->exp_value < this->lower_limit)
    {
        return this->lower_limit;
    }
    else
    {
        return this->exp_value;
    }
}


void PID_Controller::setValueLimit(float upper_limit, float lower_limit)
{
    this->upper_limit = upper_limit;
    this->lower_limit = lower_limit;
}

float PID_Controller::getError()
{
    return this->error;
}

float PID_Controller::getErrors()
{
    return this->errors;
}

float PID_Controller::getErrord()
{
    return this->errord;
}

ButterWorthParam ButterWorthParam::set(float a1, float a2, float b1, float b2)
{
    ButterWorthParam temp;
    temp.a_[0] = a1;
    temp.a_[1] = a2;
    temp.b_[0] = b1;
    temp.b_[1] = b2;
    return temp;
}

ButterWorthFilter::ButterWorthFilter()
{

}

ButterWorthFilter::~ButterWorthFilter()
{

}

void ButterWorthFilter::initialize(ButterWorthParam param)
{
    param_ = param;
    prev_output_ = 0;
    prev_value_ = 0;
}

float ButterWorthFilter::getValue(float present_value)
{
    if(prev_output_ == 0 && prev_value_ == 0)
    {
        prev_output_ = param_.b_[0]*present_value/param_.a_[0];
        prev_value_ = present_value;
        return prev_output_;
    }
    else
    {
        prev_output_ = (param_.b_[0]*present_value + param_.b_[1]*prev_value_ - param_.a_[1]*prev_output_)/param_.a_[0];
        prev_value_ = present_value;
        return prev_output_;
    }
}

BalanceLowPassFilter::BalanceLowPassFilter()
{
	cut_off_freq_ = 1.0;
	control_cycle_sec_ = 0.008;
	prev_output_ = 0;

	alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
}

BalanceLowPassFilter::~BalanceLowPassFilter()
{	}

void BalanceLowPassFilter::initialize(double control_cycle_sec, double cut_off_frequency)
{
	cut_off_freq_ = cut_off_frequency;
	control_cycle_sec_ = control_cycle_sec;
	prev_output_ = 0;

	if(cut_off_frequency > 0)
		alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
	else
		alpha_ = 1;
}

void BalanceLowPassFilter::set_cut_off_frequency(double cut_off_frequency)
{
	cut_off_freq_ = cut_off_frequency;

	if(cut_off_frequency > 0)
	alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
	else
	alpha_ = 1;
}

double BalanceLowPassFilter::get_cut_off_frequency(void)
{
	return cut_off_freq_;
}

double BalanceLowPassFilter::get_filtered_output(double present_raw_value)
{
	prev_output_ = alpha_*present_raw_value + (1.0 - alpha_)*prev_output_;
	return prev_output_;
}

string BalanceControl::DtoS(double value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();

    return str;
}

void BalanceControl::saveData()
{   ROS_INFO("aaaaaaaaaxcxcx");

	//------roll------
    char path[200] = "~/data";
	std::string tmp = std::to_string(name_cont_);
	tmp = "/Feedback_Control_Roll"+tmp+".csv";
    strcat(path, tmp.c_str());

    fstream fp;
    fp.open(path, std::ios::out);
	std::string savedText;
    std::map<std::string, std::vector<float>>::iterator it_roll;

	for(it_roll = map_roll.begin(); it_roll != map_roll.end(); it_roll++)
	{
		savedText += it_roll->first;
		if(it_roll == --map_roll.end())
		{
			savedText += "\n";
			fp<<savedText;
			savedText = "";
		}
		else
		{
			savedText += ",";
		}		
	}
	it_roll = map_roll.begin();
	int max_size = it_roll->second.size();

	for(it_roll = map_roll.begin(); it_roll != map_roll.end(); it_roll++)
	{
		if(max_size < it_roll->second.size())
            max_size = it_roll->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_roll = map_roll.begin(); it_roll != map_roll.end(); it_roll++)
        {
            if(i < it_roll->second.size())
            {
                if(it_roll == --map_roll.end())
                {
                    savedText += std::to_string(it_roll->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_roll->second[i]) + ",";
                }
            }
            else
            {
                if(it_roll == --map_roll.end())
                {
                    savedText += "none\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                    savedText += "none,";
            }
        }
    }
    fp.close();
    for(it_roll = map_roll.begin(); it_roll != map_roll.end(); it_roll++)
        it_roll->second.clear();

	//------pitch------
	char path2[200] = "~/data";
	tmp = std::to_string(name_cont_);
	tmp = "/Feedback_Control_Pitch_"+tmp+".csv";
    strcat(path2, tmp.c_str());
    fp.open(path2, std::ios::out);
	savedText = "";

    std::map<std::string, std::vector<float>>::iterator it_pitch;

	for(it_pitch = map_pitch.begin(); it_pitch != map_pitch.end(); it_pitch++)
	{
		savedText += it_pitch->first;
		if(it_pitch == --map_pitch.end())
		{
			savedText += "\n";
			fp<<savedText;
			savedText = "";
		}
		else
		{
			savedText += ",";
		}		
	}
	it_pitch = map_pitch.begin();
	max_size = it_pitch->second.size();

	for(it_pitch = map_pitch.begin(); it_pitch != map_pitch.end(); it_pitch++)
	{
		if(max_size < it_pitch->second.size())
            max_size = it_pitch->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_pitch = map_pitch.begin(); it_pitch != map_pitch.end(); it_pitch++)
        {
            if(i < it_pitch->second.size())
            {
                if(it_pitch == --map_pitch.end())
                {
                    savedText += std::to_string(it_pitch->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_pitch->second[i]) + ",";
                }
            }
            else
            {
                if(it_pitch == --map_pitch.end())
                {
                    savedText += "none\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                    savedText += "none,";
            }
        }
    }
    fp.close();
    for(it_pitch = map_pitch.begin(); it_pitch != map_pitch.end(); it_pitch++)
        it_pitch->second.clear();

	//------ZMP------
	char path3[200] = "~/data";
	tmp = std::to_string(name_cont_);
	tmp = "/Feedback_Control_ZMP_"+tmp+".csv";
    strcat(path3, tmp.c_str());
    fp.open(path3, std::ios::out);
	savedText = "";

    std::map<std::string, std::vector<float>>::iterator it_ZMP;

	for(it_ZMP = map_ZMP.begin(); it_ZMP != map_ZMP.end(); it_ZMP++)
	{
		savedText += it_ZMP->first;
		if(it_ZMP == --map_ZMP.end())
		{
			savedText += "\n";
			fp<<savedText;
			savedText = "";
		}
		else
		{
			savedText += ",";
		}		
	}
	it_ZMP = map_ZMP.begin();
	max_size = it_ZMP->second.size();

	for(it_ZMP = map_ZMP.begin(); it_ZMP != map_ZMP.end(); it_ZMP++)
	{
		if(max_size < it_ZMP->second.size())
            max_size = it_ZMP->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_ZMP = map_ZMP.begin(); it_ZMP != map_ZMP.end(); it_ZMP++)
        {
            if(i < it_ZMP->second.size())
            {
                if(it_ZMP == --map_ZMP.end())
                {
                    savedText += std::to_string(it_ZMP->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_ZMP->second[i]) + ",";
                }
            }
            else
            {
                if(it_ZMP == --map_ZMP.end())
                {
                    savedText += "none\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                    savedText += "none,";
            }
        }
    }
    fp.close();
    for(it_ZMP = map_ZMP.begin(); it_ZMP != map_ZMP.end(); it_ZMP++)
        it_ZMP->second.clear();

	//------CoM------
	char path4[200] = "~/data";
	tmp = std::to_string(name_cont_);
	tmp = "/Feedback_Control_CoM_"+tmp+".csv";
    strcat(path4, tmp.c_str());
    fp.open(path4, std::ios::out);
	savedText = "";

    std::map<std::string, std::vector<float>>::iterator it_CoM;

	for(it_CoM = map_CoM.begin(); it_CoM != map_CoM.end(); it_CoM++)
	{
		savedText += it_CoM->first;
		if(it_CoM == --map_CoM.end())
		{
			savedText += "\n";
			fp<<savedText;
			savedText = "";
		}
		else
		{
			savedText += ",";
		}		
	}
	it_CoM = map_CoM.begin();
	max_size = it_CoM->second.size();

	for(it_CoM = map_CoM.begin(); it_CoM != map_CoM.end(); it_CoM++)
	{
		if(max_size < it_CoM->second.size())
            max_size = it_CoM->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_CoM = map_CoM.begin(); it_CoM != map_CoM.end(); it_CoM++)
        {
            if(i < it_CoM->second.size())
            {
                if(it_CoM == --map_CoM.end())
                {
                    savedText += std::to_string(it_CoM->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_CoM->second[i]) + ",";
                }
            }
            else
            {
                if(it_CoM == --map_CoM.end())
                {
                    savedText += "none\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                    savedText += "none,";
            }
        }
    }
    fp.close();
    for(it_CoM = map_CoM.begin(); it_CoM != map_CoM.end(); it_CoM++)
        it_CoM->second.clear();

//------Accel------
	char path5[200] = "~/data";
	tmp = std::to_string(name_cont_);
	tmp = "/Sensor_data_accel"+tmp+".csv";
    strcat(path5, tmp.c_str());
    fp.open(path5, std::ios::out);
	savedText = "";

    std::map<std::string, std::vector<float>>::iterator it_Accel;

	for(it_Accel = map_Accel.begin(); it_Accel != map_Accel.end(); it_Accel++)
	{
		savedText += it_Accel->first;
		if(it_Accel == --map_Accel.end())
		{
			savedText += "\n";
			fp<<savedText;
			savedText = "";
		}
		else
		{
			savedText += ",";
		}		
	}
	it_Accel = map_Accel.begin();
	max_size = it_Accel->second.size();

	for(it_Accel = map_Accel.begin(); it_Accel != map_Accel.end(); it_Accel++)
	{
		if(max_size < it_Accel->second.size())
            max_size = it_Accel->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_Accel = map_Accel.begin(); it_Accel != map_Accel.end(); it_Accel++)
        {
            if(i < it_Accel->second.size())
            {
                if(it_Accel == --map_Accel.end())
                {
                    savedText += std::to_string(it_Accel->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_Accel->second[i]) + ",";
                }
            }
            else
            {
                if(it_Accel == --map_Accel.end())
                {
                    savedText += "none\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                    savedText += "none,";
            }
        }
    }
    fp.close();
    for(it_Accel = map_Accel.begin(); it_Accel != map_Accel.end(); it_Accel++)
        it_Accel->second.clear();

//-----end
	name_cont_++;
}