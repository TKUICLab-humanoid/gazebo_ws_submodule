#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <gazebo_msgs/SpawnModel.h>

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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "add_model");
	ros::NodeHandle nh;
	ros::ServiceClient add_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
	std::string model_file, model_name, Path, initial_pose_str, string_tamp_str;
	float x=0,y=0,z=0,R=0,P=0,Y=0;
	char string_tamp_char[200];
	//model_name = argv[1];
	//Path = argv[2];

	printf("model_file : ");
	scanf("%s", &string_tamp_char);
	//printf("\n");
	model_file = string_tamp_char;
	Path = "package://models/"+model_file+"/model.sdf";

	printf("model_name : ");
	scanf("%s", &string_tamp_char);
	//printf("\n");
	model_name = string_tamp_char;

	gazebo_msgs::SpawnModel srv;
	srv.request.model_name = model_name;
	srv.request.model_xml = loadModelSDF(Path);

	printf("model_initial_pose = [x, y, z]\n");
	printf("model_initial_pose = ");
	scanf("%s", &string_tamp_char);
	initial_pose_str = string_tamp_char;
	size_t npos;
	if(!initial_pose_str.empty() && initial_pose_str != "0")
	{
		initial_pose_str.erase(0,1);
		initial_pose_str.erase(strlen(initial_pose_str.c_str())-1,strlen(initial_pose_str.c_str()));
	//ROS_INFO("%s",initial_pose_str.c_str());

		npos = initial_pose_str.find(",");
		string_tamp_str = initial_pose_str.substr(0, npos);
		initial_pose_str.erase(0, strlen(string_tamp_str.c_str())+1);
		x = atof(string_tamp_str.c_str());
	//ROS_INFO("x = %f",x);
	//ROS_INFO("%s",initial_pose_str.c_str());

		npos = initial_pose_str.find(",");
		string_tamp_str = initial_pose_str.substr(0, npos);
		initial_pose_str.erase(0, strlen(string_tamp_str.c_str())+1);
		y = atof(string_tamp_str.c_str());
	//("y = %f",y);
	//ROS_INFO("%s",initial_pose_str.c_str());

		npos = initial_pose_str.find(",");
		string_tamp_str = initial_pose_str.substr(0, npos);
		initial_pose_str.erase(0, strlen(string_tamp_str.c_str())+1);
		z = atof(string_tamp_str.c_str());
	}
//--------------------------------------------------------------------------------------------------
	printf("model_initial_pose = [R, P, Y]\n");
	printf("model_initial_pose = ");
	scanf("%s", &string_tamp_char);
	initial_pose_str = string_tamp_char;
	if(!initial_pose_str.empty() && initial_pose_str != "0")
	{
		initial_pose_str.erase(0,1);
		initial_pose_str.erase(strlen(initial_pose_str.c_str())-1,strlen(initial_pose_str.c_str()));

		npos = initial_pose_str.find(",");
		string_tamp_str = initial_pose_str.substr(0, npos);
		initial_pose_str.erase(0, strlen(string_tamp_str.c_str())+1);
		R = atof(string_tamp_str.c_str())/180.0*3.14159265;

		npos = initial_pose_str.find(",");
		string_tamp_str = initial_pose_str.substr(0, npos);
		initial_pose_str.erase(0, strlen(string_tamp_str.c_str())+1);
		P = atof(string_tamp_str.c_str())/180.0*3.14159265;

		npos = initial_pose_str.find(",");
		string_tamp_str = initial_pose_str.substr(0, npos);
		initial_pose_str.erase(0, strlen(string_tamp_str.c_str())+1);
		Y = atof(string_tamp_str.c_str())/180.0*3.14159265;
		//ROS_INFO("%f",Y);
	}

	srv.request.initial_pose.position.x = x;
	srv.request.initial_pose.position.y = y;
	srv.request.initial_pose.position.z = z;
	srv.request.initial_pose.orientation.x = sin(R/2.0)*cos(P/2.0)*cos(Y/2.0) - cos(R/2.0)*sin(P/2.0)*sin(Y/2.0);
	srv.request.initial_pose.orientation.y = cos(R/2.0)*sin(P/2.0)*cos(Y/2.0) + sin(R/2.0)*cos(P/2.0)*sin(Y/2.0);
	srv.request.initial_pose.orientation.z = cos(R/2.0)*cos(P/2.0)*sin(Y/2.0) - sin(R/2.0)*sin(P/2.0)*cos(Y/2.0);
	srv.request.initial_pose.orientation.w = cos(R/2.0)*cos(P/2.0)*cos(Y/2.0) + sin(R/2.0)*sin(P/2.0)*sin(Y/2.0);
	add_model_client.call(srv);
	
	return 0;
}