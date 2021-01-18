#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "tku_libs/modelinfo.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "test_fake_frame");
  ros::NodeHandle node;
  gazebo_tool tool_gz;
  tool_gz.set_Client(node);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  
  while(node.ok())
  {
    tool_gz.get_modelinfo("kidsize");

    transform.setOrigin( tf::Vector3(tool_gz.mapModelInfo["kidsize"].Pose.x, tool_gz.mapModelInfo["kidsize"].Pose.y
      , tool_gz.mapModelInfo["kidsize"].Pose.z/*-0.336341-0.0635*/));
    q.setRPY(tool_gz.mapModelInfo["kidsize"].Angular.x, tool_gz.mapModelInfo["kidsize"].Angular.y
      , tool_gz.mapModelInfo["kidsize"].Angular.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dummy", "waist_link"));
    ros::Duration(0.01).sleep();
  }
  return 0;
};
