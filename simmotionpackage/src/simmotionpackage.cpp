#include "simmotionpackage/simmotionpackage.h"

extern struct Points_Struct Points;
extern struct Parameters_Struct Parameters;
extern 	SimIMUData sim_imu_data;
extern BalanceControl balance;

void SimMotionPackage::Savedata(const std_msgs::Bool &msg)
{   
    balance.saveData();
    inversekinematic.saveData();

}

void SimMotionPackage::motionCallback(const tku_msgs::IKinfo_message& msg)
{		
    balance.get_sensor_value();
    balance.setSupportFoot();
    balance.balance_control();

    Points.Inverse_PointR_X = msg.IK_Point_RX;
	Points.Inverse_PointR_Y = msg.IK_Point_RY;
	Points.Inverse_PointR_Z = msg.IK_Point_RZ;
	Points.Inverse_PiontR_Thta = msg.IK_Point_RThta;
	
	Points.Inverse_PointL_X = msg.IK_Point_LX;
	Points.Inverse_PointL_Y = msg.IK_Point_LY;
	Points.Inverse_PointL_Z = msg.IK_Point_LZ;
	Points.Inverse_PiontL_Thta = msg.IK_Point_LThta;

    Parameters.Period_T = msg.Period_T;
    Parameters.Period_T2 = Parameters.Period_T/2;
    Parameters.Sample_Time = msg.Sampletime;
    inversekinematic.calculate_inverse_kinematic(Parameters.Period_T / Parameters.Sample_Time);
    
    for(int i = 9; i < 21; i++)
    {
        ik_ref_data.angle[i] = inversekinematic.thta_base_[i];
    }
    SectorData output;
    for(int i = 9; i < 21; i++)
    {
        output.angle[i] = inversekinematic.output_angle_[i] + (stand_data.angle[i] - ik_ref_data.angle[i]).toDec();
        motor_angle[i].data = output.angle[i].toPI_G();
    }
    motor_angle[10].data = -motor_angle[10].data;
    motor_angle[16].data = -motor_angle[16].data;
    motor_angle[19].data = -motor_angle[19].data;
    motor_angle[13].data = -motor_angle[13].data;

    double temp[21] = {0};
    for(int i = 0; i < 21; i++)temp[i] = this->motor_angle[i].data;
    for(int i = 9; i < 21; i++)motor_control[i].publish(this->motor_angle[i]);
    
    this->motor_angle[10].data = -this->motor_angle[10].data;
    this->motor_angle[13].data = -this->motor_angle[13].data;
    this->motor_angle[16].data = -this->motor_angle[16].data;
    this->motor_angle[19].data = -this->motor_angle[19].data;
}

void SimMotionPackage::getHeadAngle(const tku_msgs::HeadPackage &msg)
{
    if(msg.ID == (int)HeadMotorID::head_pitch)
    {
        head_angle[(int)HeadMotorID::head_pitch].data = (msg.Position-2047)/2048.0*3.14159265;
        head_control[(int)HeadMotorID::head_pitch].publish(head_angle[(int)HeadMotorID::head_pitch]);
    }
    else
    {
        head_angle[(int)HeadMotorID::neck_yaw].data = (msg.Position-2047)/2048.0*3.14159265;
        head_control[(int)HeadMotorID::neck_yaw].publish(head_angle[(int)HeadMotorID::neck_yaw]);
    }
}
void SimMotionPackage::getImuData(const sensor_msgs::Imu &msg)
{
    sim_imu_data.qx = msg.orientation.x;
    sim_imu_data.qy = msg.orientation.y;
    sim_imu_data.qz = msg.orientation.z;
    sim_imu_data.qw = msg.orientation.w;
    sim_imu_data.g_x = msg.angular_velocity.x;
    sim_imu_data.g_y = msg.angular_velocity.y;
    sim_imu_data.g_z = msg.angular_velocity.z;
    sim_imu_data.a_x = msg.linear_acceleration.x;
    sim_imu_data.a_y = msg.linear_acceleration.y;
    sim_imu_data.a_z = msg.linear_acceleration.z;
    rpy_raw_[0]=-atan2(2 * (sim_imu_data.qw * sim_imu_data.qx + sim_imu_data.qy * sim_imu_data.qz), 1 - 2 * (sim_imu_data.qx*sim_imu_data.qx + sim_imu_data.qy*sim_imu_data.qy))*RADIAN2DEGREE;
    rpy_raw_[1]=asin(2 * (sim_imu_data.qw * sim_imu_data.qy - sim_imu_data.qz * sim_imu_data.qx))*RADIAN2DEGREE;
    rpy_raw_[2]=atan2(2 * (sim_imu_data.qw * sim_imu_data.qz + sim_imu_data.qx * sim_imu_data.qy), 1 - 2 * (sim_imu_data.qy*sim_imu_data.qy + sim_imu_data.qz*sim_imu_data.qz))*RADIAN2DEGREE;
    int count = 0;
    for(count=0; count<3; count++)
        {        
            

            sim_imu_data.sensor_rpy[count]= rpy_raw_[count] - rpy_offset_[count];
            if(sim_imu_data.sensor_rpy[count] < -180)
                sim_imu_data.sensor_rpy[count] += 360;
            else if(sim_imu_data.sensor_rpy[count] > 180)
                sim_imu_data.sensor_rpy[count] -= 360;
        }

        
    Sensor_Data_Process();
}

void SimMotionPackage::SectorControlFuntion(unsigned int mode, SectorData &sector_data)
{
    double j = 0;
    int k = 0;
    switch(mode)
    {
        case (unsigned int)SectorMode::AbsoluteAngle:
        	for(int i = 0; i < MotorSum; i++)now_motion_data.angle[i] = this->stand_data.angle[i];
        	for(int i = 0; i < MotorSum; i++)motor_angle[i].data = now_motion_data.angle[i].toPI_G();
            for(int i = 0; i < MotorSum; i++)this->stand_data.angle[i] = sector_data.angle[i];
            while(k != MotorSum)
            {
                k = 0;
                for(int i = 0; i < MotorSum; i++)
                {
                    if(abs(motor_angle[i].data - (this->stand_data.angle[i].toPI_G())) >= (sector_data.speed[i].toPI()/10.0) && motor_angle[i].data != (this->stand_data.angle[i].toPI_G()) && sector_data.angle[i].toDec() >= 0 && sector_data.speed[i].toPI() > 0)
                    {
                        if(this->stand_data.angle[i].toPI() - (now_motion_data.angle[i].toPI()) > 0){
                            motor_angle[i].data = now_motion_data.angle[i].toPI_G() + (sector_data.speed[i].toPI()*j/10.0);
                        }
                        else
                        {
                            motor_angle[i].data = now_motion_data.angle[i].toPI_G() - (sector_data.speed[i].toPI()*j/10.0);
                        }
                        k = 0;
                        // std::printf("%f > %f\n", motor_angle[i].data - (this->stand_data.angle[i].toPI_G()), (sector_data.speed[i].toPI()/10.0));
                        // std::printf("%f != %f\n", motor_angle[i].data, (this->stand_data.angle[i].toPI_G()));
                        // std::printf("%d\n", sector_data.angle[i].toDec());
                        // std::printf("%f\n", sector_data.speed[i].toPI());
                    }
                    else if(abs(motor_angle[i].data - (this->stand_data.angle[i].toPI_G())) < (sector_data.speed[i].toPI()/10.0) && motor_angle[i].data != (this->stand_data.angle[i].toPI_G()) && sector_data.angle[i].toDec() >= 0 && sector_data.speed[i].toPI() > 0)
                    {
                        now_motion_data.angle[i] = this->stand_data.angle[i];
                        motor_angle[i].data = now_motion_data.angle[i].toPI_G();
                        k++;
                    }
                    else
                    {
                        motor_angle[i].data = this->stand_data.angle[i].toPI_G();
                        k++;
                    }
                }
                this->motor_angle[10].data = -this->motor_angle[10].data;
                this->motor_angle[13].data = -this->motor_angle[13].data;
                this->motor_angle[16].data = -this->motor_angle[16].data;
                this->motor_angle[19].data = -this->motor_angle[19].data;
                for(int i = 0; i < MotorSum; i++)motor_control[i].publish(this->motor_angle[i]);
                this->motor_angle[10].data = -this->motor_angle[10].data;
                this->motor_angle[13].data = -this->motor_angle[13].data;
                this->motor_angle[16].data = -this->motor_angle[16].data;
                this->motor_angle[19].data = -this->motor_angle[19].data;
                tool_gz.simDelay(10);
                j++;
            }
        break;
        case (unsigned int)SectorMode::RelativeAngle:
            for(int i = 0; i < MotorSum; i++)now_motion_data.angle[i] = this->stand_data.angle[i];
            for(int i = 0; i < MotorSum; i++)motor_angle[i].data = now_motion_data.angle[i].toPI_G();
            for(int i = 0; i < MotorSum; i++)this->stand_data.angle[i] += sector_data.angle[i];
            while(k != MotorSum)
            {
                k = 0;
                for(int i = 0; i < MotorSum; i++)
                {
                    if(abs(motor_angle[i].data - (this->stand_data.angle[i].toPI_G())) >= (sector_data.speed[i].toPI()/10.0) && motor_angle[i].data != (this->stand_data.angle[i].toPI_G()) && sector_data.angle[i].toPI() != 0 && sector_data.speed[i].toPI() > 0)
                    {
                        if(sector_data.angle[i].toDec() > 0){
                            motor_angle[i].data = now_motion_data.angle[i].toPI_G() + (sector_data.speed[i].toPI()*j/10.0);
                        }
                        else
                        {
                            motor_angle[i].data = now_motion_data.angle[i].toPI_G() - (sector_data.speed[i].toPI()*j/10.0);
                        }
                        k = 0;
                    }
                    else if(abs(motor_angle[i].data - (this->stand_data.angle[i].toPI_G())) < (sector_data.speed[i].toPI()/10.0) && motor_angle[i].data != (this->stand_data.angle[i].toPI_G()) && sector_data.angle[i].toPI() != 0 && sector_data.speed[i].toPI() > 0)
                    {
                        now_motion_data.angle[i] = this->stand_data.angle[i];
                        motor_angle[i].data = now_motion_data.angle[i].toPI_G();
                        k++;
                    }
                    else
                    {
                        now_motion_data.angle[i] = this->stand_data.angle[i];
                        motor_angle[i].data = now_motion_data.angle[i].toPI_G();
                        k++;
                    }
                }
                this->motor_angle[10].data = -this->motor_angle[10].data;
                this->motor_angle[13].data = -this->motor_angle[13].data;
                this->motor_angle[16].data = -this->motor_angle[16].data;
                this->motor_angle[19].data = -this->motor_angle[19].data;
                for(int i = 0; i < MotorSum; i++)motor_control[i].publish(this->motor_angle[i]);
                this->motor_angle[10].data = -this->motor_angle[10].data;
                this->motor_angle[13].data = -this->motor_angle[13].data;
                this->motor_angle[16].data = -this->motor_angle[16].data;
                this->motor_angle[19].data = -this->motor_angle[19].data;
                tool_gz.simDelay(10);
                j++;
            }
            // for(int i = 0; i < MotorSum; i++)ROS_INFO("speed = %d\tangle = %d", stand_data.speed[i].toDec(), stand_data.angle[i].toDec());
        break;
        case (unsigned int)SectorMode::MotionList:
            for(int i = 0; i < MotorSum; i++)now_motion_data.angle[i] = this->stand_data.angle[i];
            for(int i = 0; i < MotorSum; i++)motor_angle[i].data = now_motion_data.angle[i].toPI_G();
            for(int i = 0; i < MotorSum; i++)this->stand_data.angle[i] += sector_data.angle[i];
            while(k != MotorSum)
            {
                k = 0;
                for(int i = 0; i < MotorSum; i++)
                {
                    if(abs(motor_angle[i].data - (this->stand_data.angle[i].toPI_G())) >= (sector_data.speed[i].toPI()/10.0) && motor_angle[i].data != (this->stand_data.angle[i].toPI_G()) && sector_data.angle[i].toPI() != 0 && sector_data.speed[i].toPI() > 0)
                    {
                        if(sector_data.angle[i].toDec() > 0){
                            motor_angle[i].data = now_motion_data.angle[i].toPI_G() + (sector_data.speed[i].toPI()*j/10.0);
                        }
                        else
                        {
                            motor_angle[i].data = now_motion_data.angle[i].toPI_G() - (sector_data.speed[i].toPI()*j/10.0);
                        }
                        k = 0;
                    }
                    else if(abs(motor_angle[i].data - (this->stand_data.angle[i].toPI_G())) < (sector_data.speed[i].toPI()/10.0) && motor_angle[i].data != (this->stand_data.angle[i].toPI_G()) && sector_data.angle[i].toPI() != 0 && sector_data.speed[i].toPI() > 0)
                    {
                        now_motion_data.angle[i] = this->stand_data.angle[i];
                        motor_angle[i].data = now_motion_data.angle[i].toPI_G();
                        k++;
                    }
                    else
                    {
                        motor_angle[i].data = now_motion_data.angle[i].toPI_G();
                        k++;
                    }
                }
                this->motor_angle[10].data = -this->motor_angle[10].data;
                this->motor_angle[13].data = -this->motor_angle[13].data;
                this->motor_angle[16].data = -this->motor_angle[16].data;
                this->motor_angle[19].data = -this->motor_angle[19].data;
                for(int i = 0; i < MotorSum; i++)motor_control[i].publish(this->motor_angle[i]);
                this->motor_angle[10].data = -this->motor_angle[10].data;
                this->motor_angle[13].data = -this->motor_angle[13].data;
                this->motor_angle[16].data = -this->motor_angle[16].data;
                this->motor_angle[19].data = -this->motor_angle[19].data;
                tool_gz.simDelay(10);
                j++;
            }
            tool_gz.simDelay(sector_data.delay);
        break;
    }
}

void SimMotionPackage::SectorSend2GazeboFunction(const std_msgs::Int16 &msg)
{
    ROS_INFO("SendSectorPackage");
    char filename[10];
    sprintf(filename,"%d",msg.data);
    char pathend[20] = "/sector/";
    char pathend2[20] = ".ini";
    char path[200];
    int packagecnt;
    this->sector_data.init();

    strcpy(path, tool->parameterPath.c_str());
    strcat(pathend, filename);
    strcat(path, pathend);
    strcat(path, pathend2);

    fstream fin;
    fin.open(path, ios::in);
    if(!fin)
    {
        ROS_INFO("Filename Error!!");
    }
    else
    {
        try
        {
            packagecnt = tool->readvalue(fin, "PackageCnt", 0);
            SendSectorPackage.push_back(tool->readvalue(fin, "Package", 2));
            ROS_INFO("mode = %d",SendSectorPackage[0]);
            for(int i = 1; i < packagecnt; i++)
            {
                SendSectorPackage.push_back(tool->readvalue(fin, "|", 3));
            }
            for(int i = 1, j = 0; i < packagecnt; i+=4)
            {
                if(SendSectorPackage[i] == 68 && SendSectorPackage[i+1] == 89)
                {
                    this->sector_data.delay = SendSectorPackage[i+2];
                    ROS_INFO("delay_time = %d", this->sector_data.delay);
                    SectorControlFuntion(SendSectorPackage[0], this->sector_data);
                    this->sector_data.init();
                    i += 3;
                    j = 0;
                    if(SendSectorPackage[i] == 69 && SendSectorPackage[i+1] == 78)
                    {
                        execute_ack.data = true;
                        ExecuteCallBack_pub.publish(execute_ack);
                        ROS_INFO("Execute is finsih");
                        SendSectorPackage.clear();
                        ROS_INFO("End_LoadSector");
                        return;
                    }
                }
                if(j > 20)ROS_ERROR("MotorID over 21");
                this->sector_data.speed[j].L_D = SendSectorPackage[i];
                this->sector_data.speed[j].H_D = SendSectorPackage[i+1];

                this->sector_data.angle[j].L_D = SendSectorPackage[i+2];
                this->sector_data.angle[j].H_D = SendSectorPackage[i+3];
                // ROS_INFO("speed.L_D = %d, speed.H_D = %d", sector_data.speed[j].L_D, sector_data.speed[j].H_D);
                // ROS_INFO("angle.L_D = %d, angle.H_D = %d", sector_data.angle[j].L_D, sector_data.angle[j].H_D);
                ROS_INFO("speed = %d\tangle = %d", this->sector_data.speed[j].toDec(), this->sector_data.angle[j].toDec());
                j++;
            }
            SectorControlFuntion(SendSectorPackage[0], this->sector_data);
            execute_ack.data = true;
            ExecuteCallBack_pub.publish(execute_ack);
            ROS_INFO("Execute is finsih");
        }
        catch(exception e)
        {
        }
    }
    SendSectorPackage.clear();
    ROS_INFO("End_LoadSector");
}

bool SimMotionPackage::InterfaceReadDataFunction(tku_msgs::ReadMotion::Request &Motion_req, tku_msgs::ReadMotion::Response &Motion_res)
{
    ROS_INFO("LoadParameter");
    string filename = Motion_req.name;
    char pathend[20] = "/";
    char path[200];
    char str[10];
    int datacnt;
    int num;

    if(Motion_req.readstate == 1)
    {
        strcpy(path, tool->standPath);
        strcat(pathend, filename.c_str());
        strcat(path, pathend);
    }
    else
    {
        strcpy(path, tool->parameterPath.c_str());
        strcat(pathend, filename.c_str());
        strcat(path, pathend);
    }

    fstream fin;
    fin.open(path, ios::in);
    if(!fin)
    {
        ROS_INFO("Filename Error!!");
    }
    else
    {
        try
        {
            ROS_INFO("Start_Load");
            Motion_res.VectorCnt = tool->readvalue(fin, "VectorCnt", 0);
            for(int i = 0; i < Motion_res.VectorCnt; i++)
            {
                Motion_res.motionstate.push_back(tool->readvalue(fin, "State", 0));
                Motion_res.ID.push_back(tool->readvalue(fin, "ID", 0));
                switch(Motion_res.motionstate[i])
                {
                    case 0:
                        for(int j = 0; j < 40; j++)
                        {
                            if((j+1)%2 == 1)
                            {
                                char Motion[20] = "A";
                                sprintf(str,"%d",(j/2)+1);
                                strcat(Motion, str);
                                num = tool->readvalue(fin, Motion, 2);
                            }
                            else
                            {
                                char Delay[20] = "D";
                                sprintf(str,"%d",(j/2)+1);
                                strcat(Delay, str);
                                if(j == 39)
                                {
                                    num = tool->readvalue(fin, Delay, 0);
                                }
                                else
                                {
                                    num = tool->readvalue(fin, Delay, 2);
                                }
                            }
                            Motion_res.MotionList.push_back(num);
                        }
                        break;
                    case 1:
                        for(int j = 0; j < 21; j++)
                        {
                            char Motor[20] = "M";
                            sprintf(str,"%d",j+1);
                            strcat(Motor, str);
                            if(j == 20)
                            {
                              num = tool->readvalue(fin, Motor, 0);
                            }
                            else
                            {
                                num = tool->readvalue(fin, Motor, 2);
                            }
                            Motion_res.RelativeData.push_back(num);
                        }
                        break;
                    case 2:
                        for(int j = 0; j < 21; j++)
                        {
                            char Motor[20] = "M";
                            sprintf(str,"%d",j+1);
                            strcat(Motor, str);
                            if(j == 20)
                            {
                                num = tool->readvalue(fin, Motor, 0);
                            }
                            else
                            {
                                num = tool->readvalue(fin, Motor, 2);
                            }
                            Motion_res.RelativeData.push_back(num);
                        }
                        break;
                    case 3:
                        for(int j = 0; j < 21; j++)
                        {
                            char Motor[20] = "M";
                            sprintf(str,"%d",j+1);
                            strcat(Motor, str);
                            if(j == 20)
                            {
                                num = tool->readvalue(fin, Motor, 0);
                            }
                            else
                            {
                                num = tool->readvalue(fin, Motor, 2);
                            }
                            Motion_res.AbsoluteData.push_back(num);
                        }
                        break;
                    case 4:
                        for(int j = 0; j < 21; j++)
                        {
                            char Motor[20] = "M";
                            sprintf(str,"%d",j+1);
                            strcat(Motor, str);
                            if(j == 20)
                            {
                                num = tool->readvalue(fin, Motor, 0);
                            }
                            else
                            {
                                num = tool->readvalue(fin, Motor, 2);
                            }
                            Motion_res.AbsoluteData.push_back(num);
                        }
                        break;
                }               

            }
        }
        catch(exception e)
        {
        }
    }
    ROS_INFO("End_Load");
    return true;
}

void SimMotionPackage::InterfaceSaveDataFunction(const tku_msgs::SaveMotion &msg)
{
    MotionSaveData.SaveMotionVector.push_back(msg);
    if(msg.saveflag == true)
    {
        ROS_INFO("VectorSize = %d",MotionSaveData.SaveMotionVector.size()-1);
        string filename = msg.name;
        char pathend[20] = "/";
        char path[200];

        if(msg.savestate == 1)
        {
            strcpy(path, tool->standPath);
            strcat(pathend, filename.c_str());
            strcat(path, pathend);
        }
        else
        {
            strcpy(path, tool->parameterPath.c_str());
            strcat(pathend, filename.c_str());
            strcat(path, pathend);
        }
        
        ofstream OutFile(path);
        ROS_INFO("SaveBegin");
        OutFile << "VectorCnt = ";
        OutFile << MotionSaveData.SaveMotionVector.size()-1;
        OutFile << "\n";
        for(int i = 0; i < MotionSaveData.SaveMotionVector.size()-1; i++)
        {
            switch(MotionSaveData.SaveMotionVector[i].motionstate)
            {
                case 0:
                    OutFile << "State = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].motionstate;
                    OutFile << "\n";
                    OutFile << "ID = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].ID;
                    OutFile << "\n";
                    for(int j = 0; j < MotionSaveData.SaveMotionVector[i].MotionList.size(); j++)
                    {
                        if(j%2 == 0)
                        {
                            OutFile << "A";
                            OutFile << (j/2)+1;
                            OutFile << " = ";
                            OutFile << MotionSaveData.SaveMotionVector[i].MotionList[j];
                            OutFile << "|";
                        }
                        else
                        {
                            OutFile << "D";
                            OutFile << (j/2)+1;
                            OutFile << " = ";
                            OutFile << MotionSaveData.SaveMotionVector[i].MotionList[j];
                            if(j == MotionSaveData.SaveMotionVector[i].MotionList.size()-1)
                            {
                                break;
                            }
                            else
                            {
                                OutFile << "|";
                            }
                        }
                    }
                    OutFile << "\n";
                    break;
                case 1:
                    OutFile << "State = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].motionstate;
                    OutFile << "\n";
                    OutFile << "ID = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].ID;
                    OutFile << "\n";
                    for(int j = 0; j < MotionSaveData.SaveMotionVector[i].MotorData.size(); j++)
                    {
                        OutFile << "M";
                        OutFile << j+1;
                        OutFile << " = ";
                        OutFile << MotionSaveData.SaveMotionVector[i].MotorData[j];
                        if(j == MotionSaveData.SaveMotionVector[i].MotorData.size()-1)
                        {
                            break;
                        }
                        else
                        {
                            OutFile << "|";
                        }
                    }
                    OutFile << "\n";
                    break;
                case 2:
                    OutFile << "State = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].motionstate;
                    OutFile << "\n";
                    OutFile << "ID = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].ID;
                    OutFile << "\n";
                    for(int j = 0; j < MotionSaveData.SaveMotionVector[i].MotorData.size(); j++)
                    {
                        OutFile << "M";
                        OutFile << j+1;
                        OutFile << " = ";
                        OutFile << MotionSaveData.SaveMotionVector[i].MotorData[j];
                        if(j == MotionSaveData.SaveMotionVector[i].MotorData.size()-1)
                        {
                            break;
                        }
                        else
                        {
                            OutFile << "|";
                        }
                    }
                    OutFile << "\n";
                    break;
                case 3:
                    OutFile << "State = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].motionstate;
                    OutFile << "\n";
                    OutFile << "ID = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].ID;
                    OutFile << "\n";
                    for(int j = 0; j < MotionSaveData.SaveMotionVector[i].MotorData.size(); j++)
                    {
                        OutFile << "M";
                        OutFile << j+1;
                        OutFile << " = ";
                        OutFile << MotionSaveData.SaveMotionVector[i].MotorData[j];
                        if(j == MotionSaveData.SaveMotionVector[i].MotorData.size()-1)
                        {
                            break;
                        }
                        else
                        {
                            OutFile << "|";
                        }
                    }
                    OutFile << "\n";
                    break;
                case 4:
                    OutFile << "State = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].motionstate;
                    OutFile << "\n";
                    OutFile << "ID = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].ID;
                    OutFile << "\n";
                    for(int j = 0; j < MotionSaveData.SaveMotionVector[i].MotorData.size(); j++)
                    {
                        OutFile << "M";
                        OutFile << j+1;
                        OutFile << " = ";
                        OutFile << MotionSaveData.SaveMotionVector[i].MotorData[j];
                        if(j == MotionSaveData.SaveMotionVector[i].MotorData.size()-1)
                        {
                            break;
                        }
                        else
                        {
                            OutFile << "|";
                        }
                    }
                    OutFile << "\n";
                    break;
            }               
        }
        ROS_INFO("SaveEnd");
        MotionSaveData.SaveMotionVector.clear();
        OutFile.close();
    }
}

void SimMotionPackage::InterfaceSend2SectorFunction(const tku_msgs::InterfaceSend2Sector &msg)
{
    SaveSectorPackage.push_back(msg.Package);
    int stamp = 0;
    int checksum_int = 0;
    int checksum_Lhand_int = 0;
    int checksum_Rhand_int = 0;
    int checksum_Lfoot_int = 0;
    int checksum_Rfoot_int = 0;
    uint8_t checksum_Lhand;
    uint8_t checksum_Rhand;
    uint8_t checksum_Lfoot;
    uint8_t checksum_Rfoot;
    int count = 0;
    int len = SaveSectorPackage.size();
    if (SaveSectorPackage[0] == 0x53 && SaveSectorPackage[1] == 0x54 && SaveSectorPackage[len-2] == 0x4E && SaveSectorPackage[len-1] == 0x45)
    {
        char pathend[20] = "/sector/";
        char pathend2[20] = ".ini";
        char path[200];
        string filename = msg.sectorname;

        strcpy(path, tool->parameterPath.c_str());
        strcat(pathend, filename.c_str());
        strcat(path, pathend);
        strcat(path, pathend2);

        ofstream OutFile(path);
        ROS_INFO("SaveSectorBegin");
        OutFile << "PackageCnt = ";
        OutFile << SaveSectorPackage[len-3];
        OutFile << "\n";
        OutFile << "Package = ";
        OutFile << SaveSectorPackage[2];
        OutFile <<"|| ";
        int pkgsum = 1;
        interface_ack.data = true;
        if(SaveSectorPackage[2] == 242 || SaveSectorPackage[2] == 243)
        {
            if(SaveSectorPackage[len-3] == 85)
            {
                for(int i = 3; i < SaveSectorPackage[len-3] + 2; i++)
                {
                    checksum_int += SaveSectorPackage[i];
                    if(i < 19)
                    {
                        checksum_Lhand_int = checksum_int;
                    }
                    else if(i < 35)
                    {
                        checksum_Rhand_int = checksum_int - checksum_Lhand_int;
                    }
                    else if(i < 63)
                    {
                        checksum_Lfoot_int = checksum_int - checksum_Lhand_int - checksum_Rhand_int;
                    }
                    else
                    {
                        checksum_Rfoot_int = checksum_int - checksum_Lhand_int - checksum_Rhand_int - checksum_Lfoot_int;
                    }
                }
                checksum_Lhand = checksum_Lhand_int & 0xff;
                checksum_Rhand = checksum_Rhand_int & 0xff;
                checksum_Lfoot = checksum_Lfoot_int & 0xff;
                checksum_Rfoot = checksum_Rfoot_int & 0xff;
                ROS_INFO("checksum_Lhand = %d",checksum_Lhand);
                ROS_INFO("checksum_Rhand = %d",checksum_Rhand);
                ROS_INFO("checksum_Lfoot = %d",checksum_Lfoot);
                ROS_INFO("checksum_Rfoot = %d",checksum_Rfoot);
                if(checksum_Lhand == SaveSectorPackage[len-7] && checksum_Rhand == SaveSectorPackage[len-6] && checksum_Lfoot == SaveSectorPackage[len-5] && checksum_Rfoot == SaveSectorPackage[len-4])
                {
                    ROS_INFO("Send sector is successful!!");
                    interface_ack.data = true;
                }
                else
                {
                    ROS_INFO("Send sector is fail!!");
                    interface_ack.data = false;
                }
            }
            else
            {
                ROS_INFO("Send sector is fail!!");
                interface_ack.data = false;
            }
            InterfaceCallBack_pub.publish(interface_ack);
        }
        for(int i = 3; i < SaveSectorPackage[len-3] + 2; i++)   //[0]&[1] is headpackage so +2 to save last package 
        {
            if(SaveSectorPackage[2] == 244)
            {
                if(SaveSectorPackage[i+1] == 68 && SaveSectorPackage[i+2] == 89)
                {
                    ROS_INFO("pkgsum = %d",pkgsum);
                    if(pkgsum == 84 || pkgsum == 87)
                    {
                        pkgsum = 1;
                            interface_ack.data = true;
                    }
                    else
                    {
                        ROS_INFO("Send sector is fail!!");
                        interface_ack.data = false;
                        InterfaceCallBack_pub.publish(interface_ack);
                    }
                }
                else
                {
                    if(i == SaveSectorPackage[len-3] + 1)
                    {
                        ROS_INFO("Send sector is successful!!");
                        InterfaceCallBack_pub.publish(interface_ack);
                    }
                    pkgsum++;
                }
            }
            if(!interface_ack.data)
            {
                break;
            }
            OutFile << SaveSectorPackage[i];
            OutFile <<"|| ";
        }
        ROS_INFO("SaveSectorEnd");
        SaveSectorPackage.clear();
        OutFile.close();
    }
}

bool SimMotionPackage::InterfaceCheckSectorFunction(tku_msgs::CheckSector::Request &req, tku_msgs::CheckSector::Response &res)
{
    CheckSectorPackage.clear();
    printf("CheckSectorStart\n");
    char filename[10];
    sprintf(filename,"%d",req.data);
    char pathend[20] = "/sector/";
    char pathend2[20] = ".ini";
    char path[200];
    int packagecnt;
    int returnvalue;
    bool motionlist_flag = true;
    int cnt_tmp = 84;

    strcpy(path, tool->parameterPath.c_str());
    strcat(pathend, filename);
    strcat(path, pathend);
    strcat(path, pathend2);

    fstream fin;
    fin.open(path, ios::in);
    if(!fin)
    {
        printf("Filename Error!!\n");
    }
    else
    {
        packagecnt = tool->readvalue(fin, "PackageCnt", 0);
        returnvalue = tool->readvalue(fin, "Package", 4);
        if(returnvalue != -1)
        {
            CheckSectorPackage.push_back(returnvalue);
        }
        else
        {
            res.checkflag = false;
            return true;
        }
        printf("mode = %d\n",CheckSectorPackage[0]);
        for(int i = 1; i < packagecnt; i++)
        {
            returnvalue = tool->readvalue(fin, "|", 5);
            if(returnvalue != -1)
            {
                CheckSectorPackage.push_back(returnvalue);
            }
            else
            {
                res.checkflag = false;
                return true;
            }
        }
        switch(CheckSectorPackage[0])
        {
            case 242:
            case 243:
                if(packagecnt != 85)
                {
                    printf("\033[0;31m242 243 Packagecnt is not correct!!\033[0m\n");
                    res.checkflag = false;
                    return true;
                }
                printf("Sector %d is correct!!\n",req.data);
                printf("CheckSectorEnd\n");
                res.checkflag = true;
                return true;
                break;
            case 244:
                while(motionlist_flag)
                {
                    if(cnt_tmp+5 > packagecnt)
                    {
                        printf("\033[0;31m244 count of Package is not the same as Packagecnt!!\033[0m\n");
                        res.checkflag = false;
                        return true;
                    }
                    if(CheckSectorPackage[cnt_tmp+1] == 68 && CheckSectorPackage[cnt_tmp+2] == 89)
                    {
                        if(CheckSectorPackage[cnt_tmp+4] == 69 && CheckSectorPackage[cnt_tmp+5] == 78)
                        {
                            motionlist_flag = false;
                        }
                        cnt_tmp += 87;
                    }
                    else
                    {
                        printf("\033[0;31m244 Package have not 68 89!!\033[0m\n");
                        res.checkflag = false;
                        return true;
                    }
                }
                printf("Sector %d is correct!!\n",req.data);
                printf("CheckSectorEnd\n");
                res.checkflag = true;
                return true;
                break;
            default:
                printf("\033[0;31m%d is not correct mode!!\033[0m\n",SendSectorPackage[0]);
                res.checkflag = false;
                return true;
                break;
        }
    }
}

void SimMotionPackage::readStandFunction()
{
    ROS_INFO("read_stand_data");
    char pathend[20] = "/sector/";
    char pathend2[20] = ".ini";
    char path[200];
    int packagecnt;

    strcpy(path, tool->parameterPath.c_str());
    strcat(pathend, "29");
    strcat(path, pathend);
    strcat(path, pathend2);

    fstream fin;
    fin.open(path, ios::in);
    if(!fin)
    {
        ROS_INFO("Sector 29 no exist Error!!");
    }
    else
    {
        try
        {
            packagecnt = tool->readvalue(fin, "PackageCnt", 0);
            SendSectorPackage.push_back(tool->readvalue(fin, "Package", 2));
            ROS_INFO("mode = %d",SendSectorPackage[0]);
            for(int i = 1; i < packagecnt; i++)
            {
                SendSectorPackage.push_back(tool->readvalue(fin, "|", 3));
            }
            for(int i = 1, j = 0; i < packagecnt; i+=4)
            {
                if(SendSectorPackage[i] == 68 && SendSectorPackage[i+1] == 89)
                {
                    stand_data.delay = SendSectorPackage[i+2];
                    ROS_INFO("delay_time = %d", stand_data.delay);
                    SectorControlFuntion(SendSectorPackage[0], stand_data);
                    stand_data.init();
                    i += 3;
                    j = 0;
                    if(SendSectorPackage[i] == 69 && SendSectorPackage[i+1] == 78)
                    {
                        execute_ack.data = true;
                        ExecuteCallBack_pub.publish(execute_ack);
                        SendSectorPackage.clear();
                        ROS_INFO("end_read_stand_data");
                        return;
                    }
                }
                if(j > 20)ROS_ERROR("MotorID over 21");
                stand_data.speed[j].L_D = SendSectorPackage[i];
                stand_data.speed[j].H_D = SendSectorPackage[i+1];

                stand_data.angle[j].L_D = SendSectorPackage[i+2];
                stand_data.angle[j].H_D = SendSectorPackage[i+3];
                ROS_INFO("stand_speed = %d\tstand_angle = %d", stand_data.speed[j].toDec(), stand_data.angle[j].toDec());
                j++;
            }
            execute_ack.data = true;
            ExecuteCallBack_pub.publish(execute_ack);
        }
        catch(exception e)
        {
        }
    }
    ROS_INFO("end_read_stand_data");
    SendSectorPackage.clear();
}
void SimMotionPackage::Sensor_Data_Process()
{   
    double IMU_Value[3];
    tku_msgs::SensorPackage sensorpackage;

    for(int i=0; i<3; i++)
    {
        IMU_Value[i]=sim_imu_data.sensor_rpy[i];
        sensorpackage.IMUData.push_back(IMU_Value[i]);
    }
    Sensorpackage_pub.publish(sensorpackage);
    sensorpackage.IMUData.clear();
}
void SimMotionPackage::SensorSetFunction(const tku_msgs::SensorSet &msg)
{
    bool IMU_Reset = msg.IMUReset;
    if(IMU_Reset)
        {
            for(int count=0; count<3; count++)
                rpy_offset_[count] = rpy_raw_[count];
            IMU_Reset = false;
        }
}
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "simmotionpackage");
	ros::NodeHandle nh;
	ros::NodeHandle nhPrivate("~");
	ros::CallbackQueue my_queue;
    nhPrivate.setCallbackQueue(&my_queue);
    ros::AsyncSpinner spinner(0,&my_queue);
    spinner.start();
    ros::AsyncSpinner s(1);
    s.start();
	SimMotionPackage simmotionpackage(nh, nhPrivate);

	ros::Rate loop_rate(5);
	while(nh.ok())
	{
		loop_rate.sleep();

	}

	return 0;
}