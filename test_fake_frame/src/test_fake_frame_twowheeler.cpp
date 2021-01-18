#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "tku_libs/modelinfo.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "test_fake_frame_twowheeler");
  ros::NodeHandle node;
  gazebo_tool tool_gz;
  tool_gz.set_Client(node);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  
  while(node.ok())
  {
    tool_gz.get_modelinfo("twowheeler");

    transform.setOrigin( tf::Vector3(tool_gz.mapModelInfo["twowheeler"].Pose.x, tool_gz.mapModelInfo["twowheeler"].Pose.y
      , tool_gz.mapModelInfo["twowheeler"].Pose.z/*-0.336341-0.0635*/));
    q.setRPY(tool_gz.mapModelInfo["twowheeler"].Angular.x, tool_gz.mapModelInfo["twowheeler"].Angular.y
      , tool_gz.mapModelInfo["twowheeler"].Angular.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dummy", "body_link"));
    ros::Duration(0.01).sleep();
  }
  return 0;
};
