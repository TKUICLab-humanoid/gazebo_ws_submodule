#include "simmotionpackage/Inverse_kinematic.h"

struct Points_Struct Points;
struct Parameters_Struct Parameters;
extern BalanceControl balance;

InverseKinematic::InverseKinematic()
{	        

	tool = ToolInstance::getInstance();
	rotate_body_l_ = 0.0;
	flag_ = false;
	name_cont_ = 0;
	std::vector<double> temp;
	if(map_motor.empty()){

		map_motor["motor_11"] = temp;
		map_motor["motor_12"] = temp;
		map_motor["motor_13"] = temp;
		map_motor["motor_14"] = temp;
		map_motor["motor_15"] = temp;
		map_motor["motor_17"] = temp;
		map_motor["motor_18"] = temp;
		map_motor["motor_19"] = temp;
		map_motor["motor_20"] = temp;
		map_motor["motor_21"] = temp;
	}
}

InverseKinematic::~InverseKinematic()
{

}

void InverseKinematic::initial_angle_gain()
{

	angle_gain_[0] = ALL_Angle_Gain * 1;
	angle_gain_[1] = ALL_Angle_Gain * 1;
	angle_gain_[2] = ALL_Angle_Gain * 1;
	angle_gain_[3] = ALL_Angle_Gain * 1;

	angle_gain_[4] = ALL_Angle_Gain * 1;
	angle_gain_[5] = ALL_Angle_Gain * 1;
	angle_gain_[6] = ALL_Angle_Gain * 1;
	angle_gain_[7] = ALL_Angle_Gain * 1;

	angle_gain_[8] = ALL_Angle_Gain * 1;

	angle_gain_[9] = ALL_Angle_Gain * 1;
	angle_gain_[10] = ALL_Angle_Gain * 1.2;
	angle_gain_[11] = ALL_Angle_Gain * 1;
	angle_gain_[12] = ALL_Angle_Gain * 1;
	angle_gain_[13] = ALL_Angle_Gain * 1;
	angle_gain_[14] = ALL_Angle_Gain * 1.2;

	angle_gain_[15] = ALL_Angle_Gain * 1;
	angle_gain_[16] = ALL_Angle_Gain * 1.2;
	angle_gain_[17] = ALL_Angle_Gain * 1;
	angle_gain_[18] = ALL_Angle_Gain * 1;
	angle_gain_[19] = ALL_Angle_Gain * 1;
	angle_gain_[20] = ALL_Angle_Gain * 1.2;
}

void InverseKinematic::initial_speed_gain()
{
	speed_gain_[0] = ALL_Speed_Gain * 1;
	speed_gain_[1] = ALL_Speed_Gain * 1;
	speed_gain_[2] = ALL_Speed_Gain * 1;
	speed_gain_[3] = ALL_Speed_Gain * 1;

	speed_gain_[4] = ALL_Speed_Gain * 1;
	speed_gain_[5] = ALL_Speed_Gain * 1;
	speed_gain_[6] = ALL_Speed_Gain * 1;
	speed_gain_[7] = ALL_Speed_Gain * 1;

	speed_gain_[8] = ALL_Speed_Gain * 1;

	speed_gain_[9] = ALL_Speed_Gain * 1;
	speed_gain_[10] = ALL_Speed_Gain * 2;
	speed_gain_[11] = ALL_Speed_Gain * 1;
	speed_gain_[12] = ALL_Speed_Gain * 1;
	speed_gain_[13] = ALL_Speed_Gain * 1;
	speed_gain_[14] = ALL_Speed_Gain * 2;

	speed_gain_[15] = ALL_Speed_Gain * 1;
	speed_gain_[16] = ALL_Speed_Gain * 2;
	speed_gain_[17] = ALL_Speed_Gain * 1;
	speed_gain_[18] = ALL_Speed_Gain * 1;
	speed_gain_[19] = ALL_Speed_Gain * 1;
	speed_gain_[20] = ALL_Speed_Gain * 2;
}

void InverseKinematic::initial_inverse_kinematic()
{
	initial_parameters();
	
	double Motion_Delay = Parameters.Period_T/Parameters.Sample_Time;

	initial_points();
	initial_points_process();

	Points.Inverse_PointR_X = Points.X_COM + Points.X_Right_foot;
	Points.Inverse_PointR_Y = -Points.Y_COM + Points.Y_Right_foot;
	Points.Inverse_PointR_Z = Points.Z_COM - Points.Z_Right_foot;

	Points.Inverse_PiontR_Thta = Points.Right_Thta;
	
	Points.Inverse_PointL_X = Points.X_COM + Points.X_Left_foot;
	Points.Inverse_PointL_Y = -Points.Y_COM + Points.Y_Left_foot;
	Points.Inverse_PointL_Z = Points.Z_COM - Points.Z_Left_foot; 
	Points.Inverse_PiontL_Thta = Points.Left_Thta;    
	
	calculate_inverse_kinematic(Motion_Delay);

	for(int i = 0; i < 21; i++)
	{   
		if(Points.P_Table[i])
		{
			angle_[i] = Max_value - (Points.Thta[i] * PI_TO_OUTPUT + Position_Zero);
		}
		else
		{
			angle_[i] = Points.Thta[i] * PI_TO_OUTPUT + Position_Zero;
		}
		output_base_[i] = thta_base_[i] - angle_[i];
		output_angle_[i] = thta_base_[i];
		output_speed_[i] = 100;
		past_thta_[i] = Points.Thta[i];
		Points.UThta[i] = Points.Thta[i];
		delay_time_[i] = 256;
		past_delay_time_[i] = 256;
	}
	output_base_[14] += 0;
	output_base_[20] -= 0;
	output_base_[10] += 0;
	output_base_[16] -= 0;
	Parameters.Body_Pitch_tmp = Parameters.Body_Pitch;
}

void InverseKinematic::initial_parameters(){
	Parameters.Phase_Shift = PI;        //-pi~pi
	Parameters.X_Swing_Range = 1;       //cm
	Parameters.Y_Swing_Range = 3;       //cm
	Parameters.COM_Height = COM_HEIGHT; //cm
	Parameters.l1 = 14;                 //cm Upper
	Parameters.l2 = 14;                 //cm Down
	Parameters.R_X_Offset = 0;          //cm
	Parameters.R_Y_Offset = 0;          //cm
	Parameters.R_Z_Offset = 0;          //cm
	Parameters.L_X_Offset = 0;          //cm
	Parameters.L_Y_Offset = 0;          //cm
	Parameters.L_Z_Offset = 0;          //cm
	Parameters.COM_X_Offset = 0;        //cm
	Parameters.COM_Y_Offset = 0;        //cm
	Parameters.COM_Z_Offset = 0;        //cm
	Parameters.R_Open = -3;
	Parameters.L_Open = 3.5;
	Parameters.Body_Pitch = 0;
	Parameters.Body_Pitch_tmp = 0;
	//---------------Period_Parameters---------------//
	Parameters.Phase = 0;
	Parameters.Phase_2 = 0;
	Parameters.Open_Phase = 0;
	Parameters.Period_T = 600;          //ms
	Parameters.Period_T2 = 300;         //ms
	Parameters.Sample_Time = 24;
	//--------------Walk_Parameters------------------//
	Parameters.Push_Rate = 0.40;
}
void InverseKinematic::initial_points()
{
	int i;
	for(i = 0; i < 9; i++)  
		Points.Thta[i] = PI_2;

	Points.P_Table[0] = 0;                     //Positive
	Points.P_Table[1] = 0;                     //Positive
	Points.P_Table[2] = 0;                     //Positive
	Points.P_Table[3] = 0;                     //Positive
	Points.P_Table[4] = 0;                     //Positive
	Points.P_Table[5] = 0;                     //Positive
	Points.P_Table[6] = 0;                     //Positive
	Points.P_Table[7] = 0;                     //Positive
	Points.P_Table[8] = 0;                     //Positive
	Points.P_Table[9] = 0;                     //Positive
	Points.P_Table[10] = 0;                    //Positive
	Points.P_Table[11] = 0;                    //Positive
	Points.P_Table[12] = 0;                    //Positive
	Points.P_Table[13] = 1;                    //Negitive
	Points.P_Table[14] = 1;                    //Negitive
	Points.P_Table[15] = 0;                    //Positive
	Points.P_Table[16] = 0;                    //Pogitive
	Points.P_Table[17] = 1;                    //Negitive
	Points.P_Table[18] = 1;                    //Negitive
	Points.P_Table[19] = 0;                    //Positive
	Points.P_Table[20] = 1;                    //Negitive

	thta_base_[0] = 3044;
	thta_base_[1] = 466;
	thta_base_[2] = 511;
	thta_base_[3] = 464;
	thta_base_[4] = 1044;
	thta_base_[5] = 551;
	thta_base_[6] = 511;
	thta_base_[7] = 564;
	thta_base_[8] = 2048;
	thta_base_[9] = 2048;
	thta_base_[10] = 2048;
	thta_base_[11] = 1753;
	thta_base_[12] = 2637;
	thta_base_[13] = 2343;
	thta_base_[14] = 2048;
	thta_base_[15] = 2048;
	thta_base_[16] = 2048;
	thta_base_[17] = 2343;
	thta_base_[18] = 1460;
	thta_base_[19] = 1753;
	thta_base_[20] = 2048;
}

void InverseKinematic::initial_points_process()
{
	Points.Right_Thta = 0;                                                //-pi/2~pi/2
	Points.Left_Thta = 0;                                                 //-pi/2~pi/2
	
	Points.X_Right_foot = Parameters.R_X_Offset;                          //cm
	Points.X_Left_foot = Parameters.L_X_Offset;                           //cm
	Points.X_COM =  Parameters.COM_X_Offset;                              //cm
	
	Points.Y_Right_foot = Parameters.R_Y_Offset ;                         //cm
	Points.Y_Left_foot = Parameters.L_Y_Offset;                           //cm
	Points.Y_COM = Parameters.COM_Y_Offset;                               //cm
	
	Points.Z_Right_foot =  Parameters.R_Z_Offset;                         //cm
	Points.Z_Left_foot = Parameters.L_Z_Offset;                           //cm
	Points.Z_COM = Parameters.COM_Height + Parameters.COM_Z_Offset;       //cm
}

void InverseKinematic::calculate_inverse_kinematic(int Motion_Delay)
{
    double R_Lyz, L_Lyz, R_Lxyz, L_Lxyz;
    double RX_2, RY_2, RZ_2, LX_2, LY_2, LZ_2;
    double l1_2, l2_2, l1_l2, RL_2, LL_2;
    int i;
	
    RX_2 = Points.Inverse_PointR_X * Points.Inverse_PointR_X;
    RY_2 = Points.Inverse_PointR_Y * Points.Inverse_PointR_Y;
    RZ_2 = Points.Inverse_PointR_Z * Points.Inverse_PointR_Z;
    LX_2 = Points.Inverse_PointL_X * Points.Inverse_PointL_X;
    LY_2 = Points.Inverse_PointL_Y * Points.Inverse_PointL_Y;
    LZ_2 = Points.Inverse_PointL_Z * Points.Inverse_PointL_Z;
    l1_2 = Parameters.l1 * Parameters.l1; //大腿長度平方
    l2_2 = Parameters.l2 * Parameters.l2; //小腿長度平方
    l1_l2 = Parameters.l1 + Parameters.l2; //大腿+小腿長度
    R_Lyz = sqrt(RY_2 + RZ_2); //Lr for roll
    L_Lyz = sqrt(LY_2 + LZ_2); //Ll for roll
    R_Lxyz = sqrt(RX_2 + RZ_2 + RY_2); //LR for pit
    L_Lxyz = sqrt(LX_2 + LZ_2 + LY_2); //LL for pit
    if(R_Lxyz > (l1_l2))
        R_Lxyz = l1_l2;
    if(L_Lxyz > (l1_l2))
        L_Lxyz = l1_l2;
    RL_2 = R_Lxyz * R_Lxyz;
    LL_2 = L_Lxyz * L_Lxyz;

    Points.Thta[9] = Points.Inverse_PiontL_Thta + PI_2;
    if(Points.Inverse_PointL_Y == 0)
    {
        Points.Thta[10] = PI_2; //pi/2
    }
    else
    {
        Points.Thta[10] = atan2(Points.Inverse_PointL_Z, Points.Inverse_PointL_Y);

    }

    if(Points.Inverse_PointL_X == 0)
    {
        Points.Thta[11] = PI_2 - acos((l1_2 + LL_2 - l2_2)/(2*Parameters.l1*L_Lxyz)); //左腳髖關
    }
    else if(Points.Inverse_PointL_X > 0)
    {
        Points.Thta[11] = PI_2 - acos((l1_2 + LL_2 - l2_2)/(2*Parameters.l1*L_Lxyz)) - atan2(Points.Inverse_PointL_X,L_Lyz);
    }
    else
    {
        Points.Thta[11] = PI - acos((l1_2 + LL_2 - l2_2)/(2*Parameters.l1*L_Lxyz)) - atan2(L_Lyz,-Points.Inverse_PointL_X);
    }

    Points.Thta[12] = PI - acos((l1_2 + l2_2 -LL_2)/(2*Parameters.l1*Parameters.l2)); //左膝
    Points.Thta[13] = PI - Points.Thta[11] -Points.Thta[12]; //左踝

    if(flag_ ==  0)
        Points.Thta[14] = PI - Points.Thta[10];
    else
        Points.Thta[14] = PI - Points.Thta[10]-rotate_body_l_;

    Points.Thta[15] = Points.Inverse_PiontR_Thta + PI_2;

    if(Points.Inverse_PointR_Y == 0)
    {
        Points.Thta[16] = PI_2;
    }
    else
    {
        Points.Thta[16] = atan2(Points.Inverse_PointR_Z, Points.Inverse_PointR_Y);
    }

    if(Points.Inverse_PointR_X == 0)
    {
        Points.Thta[17] = PI_2 - acos((l1_2 + RL_2 - l2_2)/(2*Parameters.l1*R_Lxyz));
    }
    else if(Points.Inverse_PointR_X > 0)
    {
        Points.Thta[17] = PI_2 - acos((l1_2 + RL_2 - l2_2)/(2*Parameters.l1*R_Lxyz)) - atan2(Points.Inverse_PointR_X,R_Lyz);
    }
    else
    {
        Points.Thta[17] = PI - acos((l1_2 + RL_2 - l2_2)/(2*Parameters.l1*R_Lxyz)) - atan2(R_Lyz,-Points.Inverse_PointR_X);
    }

    Points.Thta[18] = PI - acos((l1_2 + l2_2 - RL_2)/(2*Parameters.l1*Parameters.l2));
    Points.Thta[19] = PI - Points.Thta[17] - Points.Thta[18];

    if(flag_ ==  0)
        Points.Thta[20] = PI - Points.Thta[16];
    else
        Points.Thta[20] = PI - Points.Thta[16]-rotate_body_l_;

    
	balance.control_after_ik_calculation();


	
	
	for( i = 0; i < 21; i++)
    {
		Points.Thta[i] = Points.Thta[i] * angle_gain_[i];
        if(Points.P_Table[i])
        {
            output_angle_[i] = (unsigned int)(Max_value - (Points.Thta[i] * PI_TO_OUTPUT + Position_Zero));
        }
        else
        {
            output_angle_[i] = (unsigned int)(Points.Thta[i] * PI_TO_OUTPUT + Position_Zero);
        }
        output_angle_[i] += output_base_[i];

        if(output_angle_[i] >Max_value)
        {
            output_angle_[i] = Max_value;
        }
        else if(output_angle_[i] <= 0)
        {
            output_angle_[i] = 0;
        }
        if (output_speed_[i] <= 0)
        {
            output_speed_[i] = 0;
        }
        else if(output_speed_[i] > 32767)
        {
            output_speed_[i] = 32767;
		}
    }
	map_motor.find("motor_11")->second.push_back(double(output_angle_[10]));
	map_motor.find("motor_12")->second.push_back(double(output_angle_[11]));
	map_motor.find("motor_13")->second.push_back(double(output_angle_[12]));       
	map_motor.find("motor_14")->second.push_back(double(output_angle_[13]));
	map_motor.find("motor_15")->second.push_back(double(output_angle_[14]));
	map_motor.find("motor_17")->second.push_back(double(output_angle_[16]));
	map_motor.find("motor_18")->second.push_back(double(output_angle_[17]));
	map_motor.find("motor_19")->second.push_back(double(output_angle_[18]));
	map_motor.find("motor_20")->second.push_back(double(output_angle_[19]));
	map_motor.find("motor_21")->second.push_back(double(output_angle_[20]));


}
void InverseKinematic::saveData()
{  
    char path[200];
	strcpy(path, tool->parameterPath.c_str());

	std::string tmp = std::to_string(name_cont_);
	tmp = "/data/IK_motor"+tmp+".csv";
    strcat(path, tmp.c_str());

	
    fstream fp;
    fp.open(path, std::ios::out);
	std::string savedText;
    std::map<std::string, std::vector<double>>::iterator it_motor;

	for(it_motor = map_motor.begin(); it_motor != map_motor.end(); it_motor++)
	{
		savedText += it_motor->first;
		if(it_motor == --map_motor.end())
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
	it_motor = map_motor.begin();
	int max_size = it_motor->second.size();

	for(it_motor = map_motor.begin(); it_motor != map_motor.end(); it_motor++)
	{
		if(max_size < it_motor->second.size())
            max_size = it_motor->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_motor = map_motor.begin(); it_motor != map_motor.end(); it_motor++)
        {
            if(i < it_motor->second.size())
            {
                if(it_motor == --map_motor.end())
                {
                    savedText += std::to_string(it_motor->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_motor->second[i]) + ",";
                }
            }
            else
            {
                if(it_motor == --map_motor.end())
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
    for(it_motor = map_motor.begin(); it_motor != map_motor.end(); it_motor++)
        it_motor->second.clear();

    name_cont_++;

}