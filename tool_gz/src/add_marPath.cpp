#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <geometry_msgs/Twist.h>

#include <std_msgs/Float32.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/GetWorldProperties.h>

#define PI 3.14159265

typedef struct drawpathinfo
{
	double m,XMin,XMax;
	double YMin,YMax;
	float vX,vY,vtheta;
	float dist,lastdist,middledistmod,shortdistmod,circle_dist;
	float errordist=0.025,crossdist=0,longdist=0.5;
	float rotaTheta,exp_theta=0.52359, circle_rota_theta;
	int longcnt,middlecnt,shortcnt,path_cnt,circle_cnt;
}DrawPathInfo;

DrawPathInfo stDrawLine;
std::string select_model;
std::vector<geometry_msgs::Point> vPathPoint;
ros::ServiceClient add_model_client;
ros::ServiceClient GetWorldProperties_Client;
int path_cnt;
float dist_type;
bool getdataflag = false, circle_flag =false;

void get_allmodels(std::vector<std::string> &vmodel_name)
{
	gazebo_msgs::GetWorldProperties srv;
	if(GetWorldProperties_Client.call(srv))
	{
		vmodel_name = srv.response.model_names;
	}
	else
	{
		ROS_ERROR("Failed to get WorldAllModels");
	}
}

std::string loadModelSDF(const std::string file)
{
	std::string filename;
	try
	{
		if (file.find("package://") == 0)
		{
			filename = file;
			filename.erase(0, strlen("package://"));
			size_t pos = filename.find("/");
			if (pos != std::string::npos)
			{
				std::string package = filename.substr(0, pos);
				filename.erase(0, pos);
				std::string package_path = ros::package::getPath(package);
				filename = package_path + filename;
			}
		}
		else
		{
			ROS_ERROR("Failed to locate file: %s", file.c_str());
			return "";
		}
	}
	catch (std::exception& e)
	{
		ROS_ERROR("Failed to retrieve file: %s", e.what());
		return "";
	}

	std::ifstream t(filename);
	std::string outstr;

	t.seekg(0, std::ios::end);
	outstr.reserve(t.tellg());
	t.seekg(0, std::ios::beg);

	outstr.assign((std::istreambuf_iterator<char>(t)),
		std::istreambuf_iterator<char>());

	return outstr;
}
void set_length(const std_msgs::Float32 &msg)
{
	stDrawLine.longdist = (float)msg.data;
	ROS_INFO("stDrawLine.longdist = %f " , msg.data);
}

void set_path(const geometry_msgs::TwistConstPtr &_msg)
{
	geometry_msgs::Point PathPoint;
	PathPoint.x = _msg->linear.x/70.0*5;  //比例
	PathPoint.y = _msg->linear.y/70.0*5;
	PathPoint.z = 0.001;  //比地面高一點點
	stDrawLine.exp_theta = _msg->angular.z;	//期望的角度
	circle_flag = _msg->angular.x;
	vPathPoint.push_back(PathPoint);

	if(vPathPoint.size() > 1)  //一定會進來ㄉif
	{
		stDrawLine.XMin = vPathPoint[vPathPoint.size()-2].x;
		stDrawLine.YMin = vPathPoint[vPathPoint.size()-2].y;
		stDrawLine.XMax = vPathPoint[vPathPoint.size()-1].x;
		stDrawLine.YMax = vPathPoint[vPathPoint.size()-1].y;
	}
	if(stDrawLine.XMax != stDrawLine.XMin)
	{  //算斜率
		stDrawLine.m = (float)(stDrawLine.YMax-stDrawLine.YMin)/(stDrawLine.XMax-stDrawLine.XMin);
	}
	else
	{
		stDrawLine.m = 99999;
	}
	stDrawLine.dist = sqrt((stDrawLine.XMax-stDrawLine.XMin)*(stDrawLine.XMax-stDrawLine.XMin)
		+(stDrawLine.YMax-stDrawLine.YMin)*(stDrawLine.YMax-stDrawLine.YMin)); //開根號 算長度 √(x^2+y^2)
	stDrawLine.vX = (float)(stDrawLine.XMax-stDrawLine.XMin)/stDrawLine.dist;  //cos
	stDrawLine.vY = (float)(stDrawLine.YMax-stDrawLine.YMin)/stDrawLine.dist;  //sin
	stDrawLine.longcnt = (int)floor(stDrawLine.dist/stDrawLine.longdist) + 1;  //0.5
	// stDrawLine.lastdist = (stDrawLine.dist*1000) - stDrawLine.longcnt * 500;  //總長-long的全長
	// stDrawLine.middledistmod = (int)stDrawLine.lastdist%1000;  //取餘數 小數後3位
	// stDrawLine.shortdistmod = (int)stDrawLine.middledistmod%125;  //取餘數 除以0.125後的餘數	
	// stDrawLine.middlecnt = (int)floor(stDrawLine.middledistmod/125) ; //0.125
	// stDrawLine.shortcnt = (int)floor(stDrawLine.shortdistmod/50) ; //0.05	
	stDrawLine.rotaTheta = atan(stDrawLine.m);
//---------------------------------------------------------------------------------------------------
	// stDrawLine.circle_dist = 30.0/180.0*PI*stDrawLine.dist/sin(30.0/180.0*PI);
	stDrawLine.circle_dist = stDrawLine.exp_theta*stDrawLine.dist/sin(stDrawLine.exp_theta);
	stDrawLine.circle_cnt = floor(stDrawLine.circle_dist/stDrawLine.longdist) + 1;	//因為會少一點點距離所以要補回來
	stDrawLine.vtheta = 2*(float)stDrawLine.exp_theta/(stDrawLine.circle_cnt - 1);	//因為 60/10 要轉10次 但第一次不轉-> cnt=11
	
	if(stDrawLine.vX >= 0 && stDrawLine.vY >=0)
	{
		stDrawLine.circle_rota_theta = atan(stDrawLine.m);
	}
	else if(stDrawLine.vX < 0 && stDrawLine.vY > 0)
	{
		stDrawLine.circle_rota_theta = -atan(stDrawLine.vX/stDrawLine.vY) + PI/2;
	}
	else if(stDrawLine.vX < 0 && stDrawLine.vY == 0)
	{
		stDrawLine.circle_rota_theta = PI;
	}
	else if(stDrawLine.vX > 0 && stDrawLine.vY < 0)
	{
		stDrawLine.circle_rota_theta = atan(stDrawLine.m);
	}
	else if(stDrawLine.vX == 0 && stDrawLine.vY < 0)
	{
		stDrawLine.circle_rota_theta = -PI/2;
	}
	else if(stDrawLine.vX < 0 && stDrawLine.vY < 0)
	{
		stDrawLine.circle_rota_theta = -atan(stDrawLine.vX/stDrawLine.vY) - PI/2.0;
	}
	stDrawLine.circle_rota_theta += stDrawLine.exp_theta;
//---------------------------------------------------------------------------------------------------
	getdataflag = true;	    
}

void updatepathmodel(int &maxcnt)
{
	std::vector<std::string> vmodel_name;	
	get_allmodels(vmodel_name);
	if(!vmodel_name.empty()) //empty:vector內是空的會回傳true
	{
		for(int i = vmodel_name.size()-1; i >= 0; i--)
		{
			if(vmodel_name[i].substr(0,strlen("path_model_")) == "path_model_")
			{
				vmodel_name[i].erase(0, strlen("path_model_"));  //刪掉"path_model_"
				if(maxcnt <= std::stoi(vmodel_name[i]))  //stoi 將string轉為數字
					maxcnt = std::stoi(vmodel_name[i]) + 1;
			}
		}
	}			
}

void set_model(float &tmpx, float &tmpy, float &R, float &P, float &Y, int &maxcnt, float &x, float &y, float &z, float &circle_tmpx, float &circle_tmpy)
{
	updatepathmodel(maxcnt);	
	gazebo_msgs::SpawnModel srv;
	srv.request.model_xml = loadModelSDF("package://models/" + select_model + "/model.sdf");

	for(int i = 0; i < path_cnt; i++)
	{
		srv.request.model_name = "path_model_" + std::to_string(maxcnt+i);  //path_model_XX
		if(!circle_flag)
		{
			if(i==(path_cnt-1))
			{
				srv.request.initial_pose.position.x = stDrawLine.XMax - stDrawLine.vX*dist_type/2;
				srv.request.initial_pose.position.y = stDrawLine.YMax - stDrawLine.vY*dist_type/2;
			}
			else
			{
				srv.request.initial_pose.position.x = tmpx + stDrawLine.vX*dist_type*i; 
				srv.request.initial_pose.position.y = tmpy + stDrawLine.vY*dist_type*i; 
			}
		}
		else
		{
			if(i==(path_cnt-1))
			{
				// Y = stDrawLine.circle_rota_theta - stDrawLine.vtheta*i;
				if(sqrt((stDrawLine.XMax-circle_tmpx)*(stDrawLine.XMax-circle_tmpx)+(stDrawLine.YMax-circle_tmpy)*(stDrawLine.YMax-circle_tmpy)) <= dist_type)
				{
					stDrawLine.vX = (stDrawLine.XMax-circle_tmpx)/sqrt((stDrawLine.XMax-circle_tmpx)*(stDrawLine.XMax-circle_tmpx)+(stDrawLine.YMax-circle_tmpy)*(stDrawLine.YMax-circle_tmpy));
					stDrawLine.vY = (stDrawLine.YMax-circle_tmpy)/sqrt((stDrawLine.XMax-circle_tmpx)*(stDrawLine.XMax-circle_tmpx)+(stDrawLine.YMax-circle_tmpy)*(stDrawLine.YMax-circle_tmpy));
					srv.request.initial_pose.position.x = stDrawLine.XMax - stDrawLine.vX*dist_type/2;  
					srv.request.initial_pose.position.y = stDrawLine.YMax - stDrawLine.vY*dist_type/2; 
				}
				else
				{
					if(dist_type == 0.5)
					{
						dist_type = 1;
						srv.request.model_xml = loadModelSDF("package://models/short_path_model/model.sdf");
					}
					else if(dist_type == 1)
					{
						dist_type = 2;
						srv.request.model_xml = loadModelSDF("package://models/middle_path_model/model.sdf");
					}
					stDrawLine.vX = (stDrawLine.XMax-circle_tmpx)/sqrt((stDrawLine.XMax-circle_tmpx)*(stDrawLine.XMax-circle_tmpx)+(stDrawLine.YMax-circle_tmpy)*(stDrawLine.YMax-circle_tmpy));
					stDrawLine.vY = (stDrawLine.YMax-circle_tmpy)/sqrt((stDrawLine.XMax-circle_tmpx)*(stDrawLine.XMax-circle_tmpx)+(stDrawLine.YMax-circle_tmpy)*(stDrawLine.YMax-circle_tmpy));
					srv.request.initial_pose.position.x = stDrawLine.XMax - stDrawLine.vX*dist_type/2;  
					srv.request.initial_pose.position.y = stDrawLine.YMax - stDrawLine.vY*dist_type/2; 
				}
			}
			else
			{
				Y = stDrawLine.circle_rota_theta - stDrawLine.vtheta*i;
				stDrawLine.vX = cos(Y);
				stDrawLine.vY = sin(Y);
				if(i!=0)
				{
					circle_tmpx += stDrawLine.vX*dist_type;
					circle_tmpy += stDrawLine.vY*dist_type;
				}
				srv.request.initial_pose.position.x = circle_tmpx;  
				srv.request.initial_pose.position.y = circle_tmpy;  	
			}
		}
		srv.request.initial_pose.position.z = 0.001;
		srv.request.initial_pose.orientation.x = sin(R/2.0)*cos(P/2.0)*cos(Y/2.0) - cos(R/2.0)*sin(P/2.0)*sin(Y/2.0);
		srv.request.initial_pose.orientation.y = cos(R/2.0)*sin(P/2.0)*cos(Y/2.0) + sin(R/2.0)*cos(P/2.0)*sin(Y/2.0);
		srv.request.initial_pose.orientation.z = cos(R/2.0)*cos(P/2.0)*sin(Y/2.0) - sin(R/2.0)*sin(P/2.0)*cos(Y/2.0);
		srv.request.initial_pose.orientation.w = cos(R/2.0)*cos(P/2.0)*cos(Y/2.0) + sin(R/2.0)*sin(P/2.0)*sin(Y/2.0);
		add_model_client.call(srv);										
	}
	tmpx = srv.request.initial_pose.position.x;
	tmpy = srv.request.initial_pose.position.y;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "add_marPath");
	ros::NodeHandle nh;
	add_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
	GetWorldProperties_Client = nh.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
	ros::Subscriber set_path_sub = nh.subscribe("/line_force", 1, set_path);
	ros::Subscriber set_model_sub = nh.subscribe("/line_length", 1, set_length);
	ros::Rate loop_rate(6);

	geometry_msgs::Point PathPoint;
	PathPoint.x = 0;
	PathPoint.y = 0;
	PathPoint.z = 0.001;
	vPathPoint.push_back(PathPoint);
	

	while(nh.ok())
	{
		
		if(getdataflag)
		{
			int maxcnt = 0;			
			float x=0,y=0,z=0,R=0,P=0,Y=0,tmpx=0,tmpy=0,circle_tmpx = 0,circle_tmpy = 0;			
			gazebo_msgs::SpawnModel srv;
			Y = stDrawLine.rotaTheta;
			path_cnt = 0;

			if(stDrawLine.longcnt != 0)
			{
				tmpx = stDrawLine.XMin + stDrawLine.vX*(stDrawLine.longdist/2);  //0.25
				tmpy = stDrawLine.YMin + stDrawLine.vY*(stDrawLine.longdist/2);
				circle_tmpx =  stDrawLine.XMin + cos(stDrawLine.circle_rota_theta)*(stDrawLine.longdist/2);
				circle_tmpy =  stDrawLine.YMin + sin(stDrawLine.circle_rota_theta)*(stDrawLine.longdist/2);
				
				circle_flag ? path_cnt = stDrawLine.circle_cnt : path_cnt = stDrawLine.longcnt;

				if(stDrawLine.longdist == 0.5)
				{
					select_model = "extremelyshort_path_model";
				}
				else if(stDrawLine.longdist == 1)
				{
					select_model = "short_path_model";
				}
				else if(stDrawLine.longdist == 2)
				{
					select_model = "middle_path_model";
				}
				else if(stDrawLine.longdist == 3)
				{
					select_model = "long_path_model";
				}
				else if(stDrawLine.longdist == 5)
				{
					select_model = "extremelylong_path_model";
				}
				
				dist_type = stDrawLine.longdist;
				set_model(tmpx, tmpy, R, P, Y, maxcnt, x, y, z, circle_tmpx, circle_tmpy);

				// tmpx = stDrawLine.XMax - stDrawLine.vX*(stDrawLine.longdist/2);
				// tmpy = stDrawLine.YMax - stDrawLine.vY*(stDrawLine.longdist/2);
				// //select_model = "extremelyshort_path_model";	
				// path_cnt = 1;
				// dist_type = stDrawLine.errordist;
				// set_model(tmpx, tmpy, R, P, Y, maxcnt, x, y, z, circle_tmpx, circle_tmpy);

				if(!circle_flag)
				{
					tmpx = stDrawLine.XMax;
					tmpy = stDrawLine.YMax;
					select_model = "path_cross";	
					path_cnt = 1;
					dist_type = stDrawLine.crossdist;
					set_model(tmpx, tmpy, R, P, Y, maxcnt, x, y, z, circle_tmpx, circle_tmpy);

					tmpx = stDrawLine.XMin;
					tmpy = stDrawLine.YMin;
					select_model = "path_cross";	
					path_cnt = 1;
					dist_type = stDrawLine.crossdist;
					set_model(tmpx, tmpy, R, P, Y, maxcnt, x, y, z, circle_tmpx, circle_tmpy);
				}
				
			}	
			getdataflag = false;	
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
