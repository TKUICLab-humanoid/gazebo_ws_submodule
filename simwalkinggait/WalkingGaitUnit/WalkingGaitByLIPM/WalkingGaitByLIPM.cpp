#include "walkinggait/WalkingGaitByLIPM.hpp"

WalkingGaitByLIPM::WalkingGaitByLIPM()
{
    tool = ToolInstance::getInstance();

    is_parameter_load_ = false;

    period_t_ = 600;// T
    sample_time_ = 30;
    time_point_ = 0;
    sample_point_ = 0;
    now_step_ = 0;
    pre_step_ = 0;
    step_ = 999;
    g_ = 980;
    step_length_ = 0;//x
    last_step_length_ = 0;
    shift_length_ = 0;//y
    last_shift_length_ = 0;
    theta_ = 0;//theta
    width_size_ = 4.5;
    lift_height_ = 6;//default_Z
    left_step_ = 0;
    right_step_ = 0;
    now_length_ = 0;
    now_left_length_ = 0;
    now_right_length_ = 0;
    now_shift_ = 0;
    now_left_shift_ = 0;
    now_right_shift_ = 0;
    last_length_ = 0;
    last_shift_ = 0;
    last_theta_ = 0;
    now_width_ = 0;
    if_finish_ = false;
    plot_once_ = false;
    ready_to_stop_ = false;
    name_cont_ = 0;
}

WalkingGaitByLIPM::~WalkingGaitByLIPM()
{

}

void WalkingGaitByLIPM::initialize()
{
    parameterinfo->complan.time_point_ = 0;
    parameterinfo->complan.sample_point_ = 0;

    std::vector<float> temp;
	if(map_walk.empty())
	{
		map_walk["l_foot_x"] = temp;
        map_walk["l_foot_y"] = temp;
        map_walk["l_foot_z"] = temp;
        map_walk["l_foot_t"] = temp;
        map_walk["r_foot_x"] = temp;
        map_walk["r_foot_y"] = temp;
        map_walk["r_foot_z"] = temp;
        map_walk["r_foot_t"] = temp;
        map_walk["com_x"] = temp;
		map_walk["com_y"] = temp;
        map_walk["com_z"] = temp;
		map_walk["ideal_zmp_x"] = temp;
		map_walk["ideal_zmp_y"] = temp;
        map_walk["roll"] = temp;
		map_walk["pitch"] = temp;
		map_walk["yaw"] = temp;
		map_walk["points"] = temp;
        map_walk["time"] = temp;
	}
}

void WalkingGaitByLIPM::resetParameter()
{
    is_parameter_load_ = false;
    if_finish_ = false;
    time_point_ = 0;
    sample_point_ = 0;
    now_step_ = 0;
    pre_step_ = 0;
    step_ = 999;
    step_length_ = 0;
    last_step_length_ = 0;
    shift_length_ = 0;
    last_shift_length_ = 0;
    theta_ = 0;
    left_step_ = 0;
    right_step_ = 0;
    now_length_ = 0;
    now_left_length_ = 0;
    now_right_length_ = 0;
    now_shift_ = 0;
    now_left_shift_ = 0;
    now_right_shift_ = 0;
    last_length_ = 0;
    last_shift_ = 0;
    last_theta_ = 0;
    now_width_ = 0;
}

void WalkingGaitByLIPM::process()
{
    double T_DSP = 0;

    if(!is_parameter_load_)
    {
        step_length_ = parameterinfo->X;
        shift_length_ = parameterinfo->Y;
        period_t_ = parameterinfo->parameters.Period_T;
        T_DSP = parameterinfo->parameters.OSC_LockRange;

        if((theta_ >= 0) && ((pre_step_ % 2) == 1))
        {
            theta_ = parameterinfo->THETA;
        }
        else if((theta_ <= 0) && ((pre_step_ % 2) == 0))
        {
            theta_ = parameterinfo->THETA;
        }

        lift_height_ = parameterinfo->parameters.BASE_Default_Z;
        abs_theta_ = fabs(theta_);
        is_parameter_load_ = true;
    }

    parameterinfo->complan.sample_point_++;
    parameterinfo->complan.time_point_ = parameterinfo->complan.sample_point_*(parameterinfo->parameters.Period_T/parameterinfo->parameters.Sample_Time);
    sample_point_++;
    time_point_ = sample_point_ * sample_time_;
    Tc_ = sqrt(parameterinfo->parameters.COM_Height/g_);

    TT_ = (double)period_t_ * 0.001;

    t_ = (double)((time_point_ - (int)sample_time_) % period_t_ + sample_time_)/1000;

    now_step_ = (sample_point_ - 1)/(int)(period_t_ / sample_time_);

    if(pre_step_ != now_step_)
    {
        if((now_step_ % 2) == 1 && now_step_ > 1)
        {
            left_step_++;
        }
        else if((now_step_ % 2) == 0 && now_step_ > 1)
        {
            right_step_++;
        }

        if((pre_step_ % 2) == 1)
        {
            now_right_length_ = now_right_length_ + (last_step_length_ + step_length_);
            now_right_shift_ = now_right_shift_ + (last_shift_length_ + shift_length_);
        }
        else if((pre_step_ % 2) == 0)
        {
            if(pre_step_ == 0)
            {
                now_left_length_ = now_left_length_ + step_length_;
                now_left_shift_ = now_left_shift_ + shift_length_;
            }
            else
            {
                now_left_length_ = now_left_length_ + (last_step_length_ + step_length_);
                now_left_shift_ = now_left_shift_ + (last_shift_length_ + shift_length_);
            }
        }

        last_step_length_ = step_length_;//上次的跨幅
        last_length_ = now_length_;//上次到達的位置
        now_length_ += step_length_;//現在要到的位置
        last_shift_length_ = shift_length_;//上次的Y軸位移量
        last_shift_ = now_shift_;//上次的Y軸位移位置
        now_shift_ += shift_length_;//現在要到的Y軸位移位置
        last_theta_ = theta_;//前一次的Theta量
        is_parameter_load_ = false;
        
    }

    pre_step_ = now_step_;//步數儲存

    now_width_ = width_size_ * pow(-1, now_step_+1);

    if(ready_to_stop_)
    {
        step_ = now_step_ + 1;
        ready_to_stop_ = false;
    }

    if(now_step_ == 0)
        parameterinfo->complan.walking_state = StartStep;
    else if(now_step_ == step_)
        parameterinfo->complan.walking_state = StopStep;
    else if(now_step_ > step_)
    {
        if_finish_ = true;
        plot_once_ = true;
        parameterinfo->complan.walking_stop = true;
        resetParameter();
        // saveData();
    }
    else
        parameterinfo->complan.walking_state = Repeat;
    
    switch (parameterinfo->complan.walking_state)
    {
        case StartStep:
            now_length_ = 0;
            vx0_ = wComVelocityInit(0, step_length_/2, now_length_, TT_, Tc_);
            px_ = wComPosition(0, vx0_, now_length_, t_, Tc_);
            vy0_ = wComVelocityInit(0, now_shift_+shift_length_/2, now_shift_+now_width_, TT_, Tc_);
            py_ = wComPosition(0, vy0_, now_shift_+now_width_, t_, Tc_);
            
            lpx_ = wFootPosition(0, step_length_, t_, TT_, T_DSP);
            rpx_ = 0;
            lpy_ = wFootPosition(now_left_shift_, shift_length_, t_, TT_, T_DSP);
            rpy_ = 0;
            lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP);
            rpz_ = 0;

            if(theta_ < 0)
            {
                lpt_ = 0;
                rpt_ = 0;
            }
            else
            {
                lpt_ = wFootTheta(abs_theta_, 0, t_, TT_, T_DSP);
                rpt_ = wFootTheta(-abs_theta_, 0, t_, TT_, T_DSP);
            }
            break;
        case StopStep:
            vx0_ = wComVelocityInit(last_length_+(last_step_length_/2), now_length_, now_length_, TT_, Tc_);
            px_ = wComPosition(last_length_+(last_step_length_/2), vx0_, now_length_, t_, Tc_);
            vy0_ = wComVelocityInit(last_shift_+(last_shift_length_/2), now_shift_, now_shift_+now_width_, TT_, Tc_);
            py_ = wComPosition(last_shift_+(last_shift_length_/2), vy0_, now_shift_+now_width_, t_, Tc_);

            if((now_step_ % 2) == 1)
            {
                lpx_ = now_length_;
                rpx_ = wFootPosition(now_right_length_, (last_step_length_+step_length_)/2, t_, TT_, T_DSP);
                lpy_ = now_left_shift_;
                rpy_ = wFootPosition(now_right_shift_, (last_shift_length_+shift_length_)/2, t_, TT_, T_DSP);
                lpz_ = 0;
                rpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP);

                if(theta_ < 0)
                {
                    lpt_ = 0;
                    rpt_ = 0;
                }
                else
                {
                    lpt_ = wFootTheta(abs_theta_, 1, t_, TT_, T_DSP);
                    rpt_ = wFootTheta(-abs_theta_, 1, t_, TT_, T_DSP);
                }
            }
            else if((now_step_ % 2) == 0)
            {
                lpx_ = wFootPosition(now_left_length_, (last_step_length_+step_length_)/2, t_, TT_, T_DSP);
                rpx_ = now_length_;
                lpy_ = wFootPosition(now_left_shift_, (last_shift_length_+shift_length_)/2, t_, TT_, T_DSP);
                rpy_ = now_right_shift_;
                lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP);
                rpz_ = 0;

                if(theta_ < 0)
                {
                    lpt_ = wFootTheta(abs_theta_, 1, t_, TT_, T_DSP);
                    rpt_ = wFootTheta(-abs_theta_, 1, t_, TT_, T_DSP);
                }
                else
                {
                    lpt_ = 0;
                    rpt_ = 0;
                }
            }
            break;
        case Repeat:
            vx0_ = wComVelocityInit(last_length_+(last_step_length_/2), now_length_+(step_length_/2), now_length_, TT_, Tc_);
            px_ = wComPosition(last_length_+(last_step_length_/2), vx0_, now_length_, t_, Tc_);
            vy0_ = wComVelocityInit(last_shift_+(last_shift_length_/2), now_shift_+(shift_length_/2), now_shift_+now_width_, TT_, Tc_);
            py_ = wComPosition(last_shift_+(last_shift_length_/2), vy0_, now_shift_+now_width_, t_, Tc_);

            if((now_step_ % 2) == 1)
            {
                lpx_ = now_length_;
                rpx_ = wFootPositionRepeat(now_right_length_, (last_step_length_+step_length_)/2, t_, TT_, T_DSP);
                lpy_ = now_left_shift_;
                rpy_ = wFootPositionRepeat(now_right_shift_, (last_shift_length_+shift_length_)/2, t_, TT_, T_DSP);
                lpz_ = 0;
                rpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP);

                if(theta_*last_theta_ >= 0)
                {
                    if(theta_ < 0)
                    {
                        lpt_ = wFootTheta(abs_theta_, 0, t_, TT_, T_DSP);
                        rpt_ = wFootTheta(-abs_theta_, 0, t_, TT_, T_DSP);
                    }
                    else
                    {
                        if(now_step_ == 1)
                        {
                            lpt_ = 0;
                            rpt_ = 0;
                        }
                        else
                        {
                            lpt_ = wFootTheta(abs_theta_, 1, t_, TT_, T_DSP);
                            rpt_ = wFootTheta(-abs_theta_, 1, t_, TT_, T_DSP);
                        }
                    }
                }
                else
                {
                    lpt_ = 0;
                    rpt_ = 0;
                } 
            }
            else if((now_step_ % 2) == 0)
            {
                lpx_ = wFootPositionRepeat(now_left_length_, (last_step_length_+step_length_)/2, t_, TT_, T_DSP);
                rpx_ = now_length_;
                lpy_ = wFootPositionRepeat(now_left_shift_, (last_shift_length_+shift_length_)/2, t_, TT_, T_DSP);
                rpy_ = now_right_shift_;
                lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP);
                rpz_ = 0;

                if(theta_*last_theta_ >= 0)
                {
                    if(theta_ < 0)
                    {
                        lpt_ = wFootTheta(abs_theta_, 1, t_, TT_, T_DSP);
                        rpt_ = wFootTheta(-abs_theta_, 1, t_, TT_, T_DSP);
                    }
                    else
                    {
                        lpt_ = wFootTheta(abs_theta_, 0, t_, TT_, T_DSP);
                        rpt_ = wFootTheta(-abs_theta_, 0, t_, TT_, T_DSP);
                    }
                }
                else
                {
                    lpt_ = 0;
                    rpt_ = 0;
                }
            }
            break;
        default:
            break;
    }

    step_point_lx_ = lpx_ - px_;
    step_point_rx_ = rpx_ - px_;
    step_point_ly_ = lpy_ - py_;
    step_point_ry_ = rpy_ - py_;
    step_point_lz_ = parameterinfo->parameters.COM_Height - lpz_;
    step_point_rz_ = parameterinfo->parameters.COM_Height - rpz_;
    step_point_ltheta_ = 0 - lpt_;
    step_point_rtheta_ = 0 - rpt_;

    if(now_step_ > step_)
    {
        step_point_lx_ = 0;
        step_point_rx_ = 0;
        step_point_ly_ = 0;
        step_point_ry_ = 0;
        step_point_lz_ = parameterinfo->parameters.COM_Height;
        step_point_rz_ = parameterinfo->parameters.COM_Height;
        step_point_ltheta_ = 0;
        step_point_rtheta_ = 0;

        if_finish_ = true;
    }
    else
    {
        // map_walk.find("l_foot_x")->second.push_back(step_point_lx_);
        // map_walk.find("r_foot_x")->second.push_back(step_point_rx_);
        // map_walk.find("l_foot_y")->second.push_back(step_point_ly_);
        // map_walk.find("r_foot_y")->second.push_back(step_point_ry_);
        // map_walk.find("l_foot_z")->second.push_back(step_point_lz_);
        // map_walk.find("r_foot_z")->second.push_back(step_point_rz_);
        // map_walk.find("l_foot_t")->second.push_back(step_point_ltheta_);
        // map_walk.find("r_foot_t")->second.push_back(step_point_rtheta_);
        // map_walk.find("com_x")->second.push_back(px_);
        // map_walk.find("com_y")->second.push_back(py_);
        // map_walk.find("now_step_")->second.push_back(now_step_);
        // map_walk.find("ideal_zmp_x")->second.push_back(now_length_);
        // map_walk.find("ideal_zmp_y")->second.push_back(now_shift_+now_width_);
        // map_walk.find("roll")->second.push_back(sensor.rpy_[0]);
        // map_walk.find("pitch")->second.push_back(sensor.rpy_[1]);
        // map_walk.find("yaw")->second.push_back(period_t_ / sample_time_);
        // map_walk.find("points")->second.push_back(sample_point_);
        // map_walk.find("time")->second.push_back(t_);
    }

    parameterinfo->points.IK_Point_RX = step_point_rx_;
	parameterinfo->points.IK_Point_RY = step_point_ry_;
	parameterinfo->points.IK_Point_RZ = step_point_rz_;
	parameterinfo->points.IK_Point_RTheta = step_point_rtheta_;
	parameterinfo->points.IK_Point_LX = step_point_lx_;
	parameterinfo->points.IK_Point_LY = step_point_ly_;
	parameterinfo->points.IK_Point_LZ = step_point_lz_;
	parameterinfo->points.IK_Point_LTheta = step_point_ltheta_;
}

double WalkingGaitByLIPM::wComVelocityInit(double x0, double xt, double px, double t, double T)
{
    return (xt - x0*cosh(t/T) + px*(cosh(t/T)-1))/(T*sinh(t/T));
}

double WalkingGaitByLIPM::wComPosition(double x0, double vx0, double px, double t, double T)
{
    return (x0*cosh(t/T) + T*vx0*sinh(t/T) - px*(cosh(t/T)-1));
}

double WalkingGaitByLIPM::wFootPosition(const double start, const double length, const double t, const double T, const double T_DSP)
{
    double new_T = T*(1-T_DSP);
    double new_t = t-T*T_DSP/2;
    double omega = 2*PI/new_T;

    if(t > 0 && t <= T*T_DSP/2)
        return start;
    else if(t > T*T_DSP/2 && t <= T*(1-T_DSP/2))
        return length*(omega*new_t-sin(omega*new_t))/(2*PI)+start;
    else
        return length+start;
}

double WalkingGaitByLIPM::wFootPositionRepeat(const double start, const double length, const double t, const double T, const double T_DSP)
{
    double new_T = T*(1-T_DSP);
    double new_t = t-T*T_DSP/2;
    double omega = 2*PI/new_T;

    if(t > 0 && t <= T*T_DSP/2)
        return start;
    else if(t > T*T_DSP/2 && t <= T*(1-T_DSP/2))
        return 2*length*(omega*new_t-sin(omega*new_t))/(2*PI)+start;
    else
        return 2*length+start;
}

double WalkingGaitByLIPM::wFootPositionZ(const double height, const double t, const double T, const double T_DSP)
{
    double new_T = T*(1-T_DSP);
    double new_t = t-T*T_DSP/2;
    double omega = 2*PI/new_T;

    if(t > T*T_DSP/2 && t <= T*(1-T_DSP/2))
        return 0.5*height*(1-cos(omega*new_t));
    else
        return 0;
}

double WalkingGaitByLIPM::wFootTheta(const double theta, bool reverse, const double t, const double T, const double T_DSP)
{
    double new_T = T*(1-T_DSP);
    double new_t = t-T*T_DSP/2;
    double omega = 2*PI/new_T;

    if(t > 0 && t <= T*T_DSP/2)
        return 0;
    else if(t > T*T_DSP/2 && t <= T*(1-T_DSP/2) && !reverse)
        return 0.5*theta*(1-cos(0.5*omega*(new_t)));
    else if(t > T*T_DSP/2 && t <= T*(1-T_DSP/2) && reverse)
        return 0.5*theta*(1-cos(0.5*omega*(new_t-new_T)));
    else
        return 0;
}

double WalkingGaitByLIPM::unit_step(double x)
{
    if(x < 0)
        return 0;
    else
        return 1;
}

double WalkingGaitByLIPM::sinh(double x)
{
    return (double)(exp(x)-exp(-x))/2;
}

double WalkingGaitByLIPM::cosh(double x)
{
    return (double)(exp(x)+exp(-x))/2;
}

string WalkingGaitByLIPM::DtoS(double value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();
    return str;
}

void WalkingGaitByLIPM::saveData()
{
    char path[200];
    strcpy(path, tool->parameterPath.c_str());
	std::string tmp = std::to_string(name_cont_);
	tmp = "/Walking_Trajectory_"+tmp+".csv";
    strcat(path, tmp.c_str());

    fstream fp;
    fp.open(path, std::ios::out);
	std::string savedText;
    std::map<std::string, std::vector<float>>::iterator it_walk;

	for(it_walk = map_walk.begin(); it_walk != map_walk.end(); it_walk++)
	{
		savedText += it_walk->first;
		if(it_walk == --map_walk.end())
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
	it_walk = map_walk.begin();
	int max_size = it_walk->second.size();

	for(it_walk = map_walk.begin(); it_walk != map_walk.end(); it_walk++)
	{
		if(max_size < it_walk->second.size())
            max_size = it_walk->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_walk = map_walk.begin(); it_walk != map_walk.end(); it_walk++)
        {
            if(i < it_walk->second.size())
            {
                if(it_walk == --map_walk.end())
                {
                    savedText += std::to_string(it_walk->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_walk->second[i]) + ",";
                }
            }
            else
            {
                if(it_walk == --map_walk.end())
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
    for(it_walk = map_walk.begin(); it_walk != map_walk.end(); it_walk++)
        it_walk->second.clear();

    name_cont_++;
}