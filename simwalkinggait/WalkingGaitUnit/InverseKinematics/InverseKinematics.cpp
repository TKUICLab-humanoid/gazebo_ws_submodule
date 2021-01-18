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

#include "walkinggait/InverseKinematics.hpp"

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/
//InverseKinematics *inversekinematics = new InverseKinematics;

InverseKinematics::InverseKinematics()
{
    theta_r0.clear();
    theta_r1.clear();
    theta_r2.clear();
    theta_r3.clear();
    theta_r4.clear();

    theta_l0.clear();
    theta_l1.clear();
    theta_l2.clear();
    theta_l3.clear();
    theta_l4.clear();
    temp_motion = new int[22];
    //parameterinfo->temp_motion_test = temp_motion;
}

InverseKinematics::~InverseKinematics()
{

}

void InverseKinematics::Initail_Parameters()
{
    //Point_Initial
    for(int i = 0; i < 9; i++)
        parameterinfo->ik_parameters.Thta[i] = PI_2;

    P_Table[0] = 0;                     //Positive
    P_Table[1] = 0;                     //Positive
    P_Table[2] = 0;                     //Positive
    P_Table[3] = 0;                     //Positive
    P_Table[4] = 0;                     //Positive
    P_Table[5] = 0;                     //Positive
    P_Table[6] = 0;                     //Positive
    P_Table[7] = 0;                     //Positive
    P_Table[8] = 0;                     //Positive
    P_Table[9] = 0;                     //Positive
    P_Table[10] = 1;                    //Negitive
    P_Table[11] = 0;                    //Positive
    P_Table[12] = 0;                    //Positive
    P_Table[13] = 1;                    //Negitive
    P_Table[14] = 1;                    //Negitive
    P_Table[15] = 0;                    //Positive
    P_Table[16] = 1;                    //Pogitive
    P_Table[17] = 1;                    //Negitive
    P_Table[18] = 1;                    //Negitive
    P_Table[19] = 0;                    //Positive
    P_Table[20] = 1;                    //Negitive

    Thta_Base[0] = 3050;//3072;
    Thta_Base[1] = 456;//466;
    Thta_Base[2] = 511;
    Thta_Base[3] = 480;//464;
    Thta_Base[4] = 1024;
    Thta_Base[5] = 560;
    Thta_Base[6] = 511;
    Thta_Base[7] = 544;//564;
    Thta_Base[8] = 2063;//2054;
    Thta_Base[9] = 2030;//3072;
    Thta_Base[10] = 2030;//1984+10;
    Thta_Base[11] = 1570;//1554-1000;
    Thta_Base[12] = 2739;//2141-2000;
    Thta_Base[13] = 2410;//2369+1000;
    Thta_Base[14] = 2048;//2028-10;
    Thta_Base[15] = 2030;//3072;
    Thta_Base[16] = 2060;//2021-12;
    Thta_Base[17] = 2530;//2510+1000;
    Thta_Base[18] = 1361;//1939+2000;
    Thta_Base[19] = 1705;//1722-1000;
    Thta_Base[20] = 2060;//2039+12;

}

void InverseKinematics::Initail_IK()
{
    int Angle[21];
    int Period_T = 600;        //ms
    int Sample_Time = 24;      //24
    int Motion_Delay = Period_T/Sample_Time;
testnum=0;
    Initail_Parameters();

    //R_Thta = 0
    parameterinfo->ik_parameters.R_Goal[0] = 0;
    parameterinfo->ik_parameters.R_Goal[1] = -4.5;//0
    parameterinfo->ik_parameters.R_Goal[2] = 0;
    parameterinfo->ik_parameters.R_Goal[3] = 0;
    //L_Thta = 0
    parameterinfo->ik_parameters.L_Goal[0] = 0;
    parameterinfo->ik_parameters.L_Goal[1] = 4.5;//0
    parameterinfo->ik_parameters.L_Goal[2] = 0;
    parameterinfo->ik_parameters.L_Goal[3] = 0;
    DoIK(Motion_Delay);
    for(int i = 0; i < 21; i++)
    {
        if(P_Table[i])
        {
            Angle[i] = Max_value - (parameterinfo->ik_parameters.Thta[i] * PI_TO_OUTPUT + Position_Zero);
        }
        else
        {
            Angle[i] =parameterinfo->ik_parameters.Thta[i] * PI_TO_OUTPUT + Position_Zero;
        }
        Output_Base[i] = Thta_Base[i] - Angle[i];
        Output_Angle[i] = Thta_Base[i];
        Output_Speed[i] = 100;
        Past_Thta[i] =parameterinfo->ik_parameters.Thta[i];
        UThta[i] =parameterinfo->ik_parameters.Thta[i];
        Delay_time[i] = 256;
        Past_Delay_time[i] = 256;
    }
}

void InverseKinematics::DoIK(int Motion_Delay)
{
    R_IK(parameterinfo->ik_parameters.R_Goal);
    L_IK(parameterinfo->ik_parameters.L_Goal);


if(parameterinfo->points.Z_Left_foot==0)
{
		for(int i = 0; i < 5; i++)
		{
			for(int j = 0; j < 5; j++)
			{
				parameterinfo->w[i*5+j] = parameterinfo->w1[i]*parameterinfo->w2[j];

				parameterinfo->ik_parameters.Thta[14] += parameterinfo->rule[i*5+j]*parameterinfo->w[i*5+j];
			}
		}
/*
parameterinfo->ik_parameters.Thta[10] += parameterinfo->w1[0]*parameterinfo->rule[0]
					+ parameterinfo->w1[1]*parameterinfo->rule[1]
					+ parameterinfo->w1[2]*parameterinfo->rule[2]
					+ parameterinfo->w1[3]*parameterinfo->rule[3]
					+ parameterinfo->w1[4]*parameterinfo->rule[4];

parameterinfo->ik_parameters.Thta[14] + = parameterinfo->w1[0]*parameterinfo->rule[0]
					+ parameterinfo->w1[1]*parameterinfo->rule[1]
					+ parameterinfo->w1[2]*parameterinfo->rule[2]
					+ parameterinfo->w1[3]*parameterinfo->rule[3]
					+ parameterinfo->w1[4]*parameterinfo->rule[4];
*/

}

if(parameterinfo->points.Z_Right_foot==0)
{

		for(int i = 0; i < 5; i++)
		{
			for(int j = 0; j < 5; j++)
			{
				parameterinfo->w[i*5+j] = parameterinfo->w1[i]*parameterinfo->w2[j];

				parameterinfo->ik_parameters.Thta[16] += parameterinfo->rule[i*5+j]*parameterinfo->w[i*5+j];
			}
		}
/*
parameterinfo->ik_parameters.Thta[16] += parameterinfo->w1[0]*parameterinfo->rule[0]
					+ parameterinfo->w1[1]*parameterinfo->rule[1]
					+ parameterinfo->w1[2]*parameterinfo->rule[2]
					+ parameterinfo->w1[3]*parameterinfo->rule[3]
					+ parameterinfo->w1[4]*parameterinfo->rule[4];

parameterinfo->ik_parameters.Thta[20] + = parameterinfo->w1[0]*parameterinfo->rule[0]
					+ parameterinfo->w1[1]*parameterinfo->rule[1]
					+ parameterinfo->w1[2]*parameterinfo->rule[2]
					+ parameterinfo->w1[3]*parameterinfo->rule[3]
					+ parameterinfo->w1[4]*parameterinfo->rule[4];
*/

}





//    parameterinfo->ik_parameters.Thta[4] = asin(parameterinfo->ik_parameters.L_Goal[0]/25)*30;
//    parameterinfo->ik_parameters.Thta[0] = asin(-parameterinfo->ik_parameters.R_Goal[0]/25)*30;


    for(int i = 0; i < 21; i++)
    {
        if(P_Table[i])
        {
            Output_Angle[i] = (unsigned int)(Max_value - (parameterinfo->ik_parameters.Thta[i] * PI_TO_OUTPUT + Position_Zero));
        }
        else
        {
            Output_Angle[i] = (unsigned int)(parameterinfo->ik_parameters.Thta[i] * PI_TO_OUTPUT + Position_Zero);
        }
        Output_Angle[i] += Output_Base[i];
        double Different_Thta;

        Different_Thta = fabs( Past_Thta[i] -parameterinfo->ik_parameters.Thta[i]);
        if(Different_Thta > 0)
        {
            Delay_time[i] = (unsigned int)(Different_Thta/(2*PI) * (1000/Motion_Delay) * 60 /0.111);
        }
        Past_Thta[i] =parameterinfo->ik_parameters.Thta[i];
        Output_Speed[i] = Delay_time[i];

        if(Output_Angle[i] >Max_value)
        {
            Output_Angle[i] = Max_value;
        }
        else if(Output_Angle[i] <= 0)
        {
            Output_Speed[i] = 0;
        }
        if (Output_Speed[i] <= 0)
        {
            Output_Speed[i] = 1;
        }
        else if(Output_Speed[i] > 1023)
        {
            Output_Speed[i] = 1023;
        }
/*
if(parameterinfo->control == 1)
{
	Output_Angle[0] -= 200;//L
	Output_Angle[4] += 200;//R
	Output_Angle[11] -= 100;
	Output_Angle[17] += 100;
}
else if(parameterinfo->control == 2)
{
	Output_Angle[0] += 300;//L
	Output_Angle[4] -= 300;//R
	Output_Angle[11] += 40;
	Output_Angle[17] -= 40;
}
*/

/*
int feetPB = 150;//+
int feetNB = -150;
int speed = 100;

	Output_Angle[0] -= parameterinfo->w1*400 + parameterinfo->w2*0 + parameterinfo->w3*(-400);//L
	Output_Angle[4] += parameterinfo->w1*400 + parameterinfo->w2*0 + parameterinfo->w3*(-400);//R
	Output_Angle[11] -= parameterinfo->w1*feetPB*1.2 + parameterinfo->w2*0 + parameterinfo->w3*(feetNB*1.2);
	Output_Angle[17] += parameterinfo->w1*feetPB*1.2 + parameterinfo->w2*0 + parameterinfo->w3*(feetNB*1.2);
	Output_Angle[12] -= parameterinfo->w1*(feetPB/2) + parameterinfo->w2*0 + parameterinfo->w3*(feetNB/2);
	Output_Angle[18] += parameterinfo->w1*(feetPB/2) + parameterinfo->w2*0 + parameterinfo->w3*(feetNB/2);
	Output_Angle[13] -= parameterinfo->w1*feetPB + parameterinfo->w2*0 + parameterinfo->w3*(feetNB*0.8);
	Output_Angle[19] += parameterinfo->w1*feetPB + parameterinfo->w2*0 + parameterinfo->w3*(feetNB*0.8);
*/
/*
	Output_Angle[0] -= parameterinfo->w1*400 + parameterinfo->w2*0 + parameterinfo->w3*(-400);//L
	Output_Angle[4] += parameterinfo->w1*400 + parameterinfo->w2*0 + parameterinfo->w3*(-400);//R
	Output_Angle[11] -= parameterinfo->w1*feetPB + parameterinfo->w2*0 + parameterinfo->w3*(feetNB);
	Output_Angle[17] += parameterinfo->w1*feetPB + parameterinfo->w2*0 + parameterinfo->w3*(feetNB);
	Output_Angle[12] -= parameterinfo->w1*(-feetPB/2) + parameterinfo->w2*0 + parameterinfo->w3*(feetNB*0.8);
	Output_Angle[18] += parameterinfo->w1*(-feetPB/2) + parameterinfo->w2*0 + parameterinfo->w3*(feetNB*0.8);
	Output_Angle[13] -= parameterinfo->w1*-feetPB/3 + parameterinfo->w2*0 + parameterinfo->w3*(-feetNB*0.2);
	Output_Angle[19] += parameterinfo->w1*-feetPB/3 + parameterinfo->w2*0 + parameterinfo->w3*(-feetNB*0.2);
*/


//Output_Speed[10] += 50;
//Output_Speed[14] += 50;
//Output_Speed[16] += 50;
//Output_Speed[20] += 50;


/*
Output_Speed[0] += speed;
Output_Speed[4] += speed;
Output_Speed[11] += speed;
Output_Speed[19] += speed;
Output_Speed[17] += speed;
Output_Speed[13] += speed;
Output_Speed[12] += speed*1.5;
Output_Speed[18] += speed*1.5;

	Output_Angle[10] += 10;
	Output_Angle[11] += 10;
	Output_Angle[16] -= 18;
*/
        temp_motion[i] = ((Output_Speed[i]<<16&0xffff0000) | (Output_Angle[i]&0x0000ffff));
        //parameterinfo->temp_motion_test[i] = temp_motion[i];
    //ROS_INFO("333");
    }
    Motion_Delay = Motion_Delay;
    temp_motion[21] = Motion_Delay;

    theta_r0.push_back(Output_Angle[10]);
    theta_r1.push_back(Output_Angle[11]);
    theta_r2.push_back(Output_Angle[12]);
    theta_r3.push_back(Output_Angle[13]);
    theta_r4.push_back(Output_Angle[14]);
    theta_l0.push_back(Output_Angle[16]);
    theta_l1.push_back(Output_Angle[17]);
    theta_l2.push_back(Output_Angle[18]);
    theta_l3.push_back(Output_Angle[19]);
    theta_l4.push_back(Output_Angle[20]);

    parameterinfo->rosflag = true;
    if(parameterinfo->complan.walking_stop)
    {
//ROS_INFO("nioujd");
    //    SaveData();
testnum=0;
    }
}

void InverseKinematics::R_IK(float *Goal)//R_IK( float *theta, float *Goal )
{
    float p, fi, C3, S3;
    float theta[6];

    theta[0] = atan2((21.75 - Goal[2]), (-Goal[1]-4.5));
    theta[4] = theta[0];

    C3 = (((21.75-Goal[2])/sin(theta[0]))*((21.75-Goal[2])/sin(theta[0])) + (Goal[0])*(Goal[0]) - 314) / (314);
    S3 = sqrt(1-C3*C3);
    theta[2] = atan2(S3, C3);
    p = sqrt((14*cos(theta[2])+14)*(14*cos(theta[2])+14) + (196*sin(theta[2])*sin(theta[2])));
    fi = atan2(14*sin(theta[2]), 14*cos(theta[2])+14);
    theta[1] = atan2(((Goal[0])/p), sqrt(1 - ((Goal[0])/p)*((Goal[0])/p))) - fi;
    theta[3] = 0 - theta[1] - theta[2];
    theta[5] = Goal[3]*5;//PI;

    parameterinfo->ik_parameters.Rtest_theta[5] = -1*theta[5];
    parameterinfo->ik_parameters.Rtest_theta[0] = (theta[0] - 1.57);
    parameterinfo->ik_parameters.Rtest_theta[1] = theta[1];
    parameterinfo->ik_parameters.Rtest_theta[2] = theta[2];
    parameterinfo->ik_parameters.Rtest_theta[3] = theta[3];
    parameterinfo->ik_parameters.Rtest_theta[4] = theta[4] - 1.57;
/*    printf("theta 0 = %f\n",parameterinfo->ik_parameters.Ltest_theta[0]);
    printf("theta 1 = %f\n",parameterinfo->ik_parameters.Ltest_theta[1]);
    printf("theta 2 = %f\n",parameterinfo->ik_parameters.Ltest_theta[2]);
    printf("theta 3 = %f\n",parameterinfo->ik_parameters.Ltest_theta[3]);
    printf("8theta 4 = %f\n",parameterinfo->ik_parameters.Ltest_theta[4]);*/

    theta[0] = 3.13727587 - theta[0];

    theta[1] = theta[1] + 1.82569142;
    theta[2] = theta[2] - 0.454453018;
    theta[3] = theta[3] + 1.770354259;

    double sertyuiwo = theta[1];
    theta[1] = theta[3];
    theta[3] = sertyuiwo;

    parameterinfo->ik_parameters.Thta[15] = Goal[3] + PI_2;
    for(int i = 0;i < 5;i++)
        parameterinfo->ik_parameters.Thta[i+16] = theta[i];

//---------------------------------
/*test[0][testnum] = parameterinfo->ik_parameters.Rtest_theta[0];
test[1][testnum] = parameterinfo->ik_parameters.Rtest_theta[1];
test[2][testnum] = parameterinfo->ik_parameters.Rtest_theta[2];
test[3][testnum] = parameterinfo->ik_parameters.Rtest_theta[3];
test[4][testnum] = parameterinfo->ik_parameters.Rtest_theta[4];
test[10][testnum] = parameterinfo->ik_parameters.Thta[16];
test[11][testnum] = parameterinfo->ik_parameters.Thta[17];
test[12][testnum] = parameterinfo->ik_parameters.Thta[18];
test[13][testnum] = parameterinfo->ik_parameters.Thta[19];
test[14][testnum] = parameterinfo->ik_parameters.Thta[20];*/
//---------------------------------

}

void InverseKinematics::L_IK(float *Goal)//L_IK( float *theta, float *Goal )
{
    
    float p, fi, C3, S3;
    float theta[6];

    theta[0] = atan2((Goal[2]-21.75), (Goal[1]-4.5));
    theta[4] = 3.1519804335 + theta[0];

    C3 = (((Goal[2]-21.75)/sin(theta[0]))*((Goal[2]-21.75)/sin(theta[0])) + (Goal[0])*(Goal[0]) - 314) / (314);
    S3 = sqrt(1-C3*C3);
    theta[2] = atan2(S3, C3);
    p = sqrt((14*cos(theta[2]) + 14)*(14*cos(theta[2]) + 14) + (196*sin(theta[2])*sin(theta[2])));
    fi = atan2(14*sin(theta[2]), 14*cos(theta[2])+14);
    theta[1] = atan2(((-Goal[0])/p), sqrt(1 - ((Goal[0])/p)*((Goal[0])/p))) - fi;
    theta[3] = 0 - theta[1] - theta[2];
    if (theta[0] < 0)
        theta[0] = -theta[0];
    theta[5] = Goal[3]*5;//PI;
    parameterinfo->ik_parameters.Ltest_theta[0] = -1*(theta[0] - 1.57);
    parameterinfo->ik_parameters.Ltest_theta[1] = theta[1];
    parameterinfo->ik_parameters.Ltest_theta[2] = theta[2];
    parameterinfo->ik_parameters.Ltest_theta[3] = theta[3];
    parameterinfo->ik_parameters.Ltest_theta[4] = theta[4] - 3.1519804335 + 1.57;;
    parameterinfo->ik_parameters.Ltest_theta[5] = theta[5];
    theta[1] = theta[1] - 1.335607373 + PI;
    theta[2] = theta[2] - 0.462082784;
    theta[3] = theta[3] + 1.797690198;


    parameterinfo->ik_parameters.Thta[9] = Goal[3] + PI_2;
    for(int i = 0;i < 5;i++)
        parameterinfo->ik_parameters.Thta[i+10] = theta[i];

//---------------------------------
/*test[5][testnum] = parameterinfo->ik_parameters.Ltest_theta[0];
test[6][testnum] = parameterinfo->ik_parameters.Ltest_theta[1];
test[7][testnum] = parameterinfo->ik_parameters.Ltest_theta[2];
test[8][testnum] = parameterinfo->ik_parameters.Ltest_theta[3];
test[9][testnum] = parameterinfo->ik_parameters.Ltest_theta[4];
test[15][testnum] = parameterinfo->ik_parameters.Thta[10];
test[16][testnum] = parameterinfo->ik_parameters.Thta[11];
test[17][testnum] = parameterinfo->ik_parameters.Thta[12];
test[18][testnum] = parameterinfo->ik_parameters.Thta[13];
test[19][testnum] = parameterinfo->ik_parameters.Thta[24];*/
//---------------------------------
testnum++;
}

string InverseKinematics::FtoS(float value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();

    return str;
}

void InverseKinematics::SaveData()
{

    //    string savedText = "R0\tR1\t"
    //                       "R2\tR3\t"
    //                       "R4\tL0\t"
    //                       "L1\tL2\tL3\t"
    //                       "L4\n";
    string savedText = "11\t12\t13\t"
                       "14\t15\t17\t"
                       "18\t19\t20\t"
                       "21\t--\t11\t12\t"
                       "13\t14\t15\t17\t"
                       "18\t19\t20\t21\n";
    char filename[]="/home/iclab03/Desktop/iclab_warehouse/newsystem/IK_Record.ods";

    fstream fp;
    fp.open(filename, ios::out);

    fp<<savedText;

    for(int i = 0; i < 100/*theta_r0.size()*/; i++)
    {
        savedText = FtoS(test[0][i]) + "\t"
                + FtoS(test[1][i]) + "\t"
                + FtoS(test[2][i]) + "\t"
                + FtoS(test[3][i]) + "\t"
                + FtoS(test[4][i]) + "\t"
                + FtoS(test[5][i]) + "\t"
                + FtoS(test[6][i]) + "\t"
                + FtoS(test[7][i]) + "\t"
                + FtoS(test[8][i]) + "\t"
                + FtoS(test[9][i]) + "\t"
                + "\t"
                + FtoS(test[10][i]) + "\t"
                + FtoS(test[11][i]) + "\t"
                + FtoS(test[12][i]) + "\t"
                + FtoS(test[13][i]) + "\t"
                + FtoS(test[14][i]) + "\t"
                + FtoS(test[15][i]) + "\t"
                + FtoS(test[16][i]) + "\t"
                + FtoS(test[17][i]) + "\t"
                + FtoS(test[18][i]) + "\t"
                + FtoS(test[19][i]) + "\n";

        fp<<savedText;
    }

    theta_r0.clear();
    theta_r1.clear();
    theta_r2.clear();
    theta_r3.clear();
    theta_r4.clear();

    theta_l0.clear();
    theta_l1.clear();
    theta_l2.clear();
    theta_l3.clear();
    theta_l4.clear();

    fp.close();

}
