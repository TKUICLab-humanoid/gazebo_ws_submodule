#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
// #include <turtlesim/Velocity.h>
// #include <turtlesim/Spawn.h>

// int main(int argc, char** argv){
//   ros::init(argc, argv, "my_tf_listener");

//   ros::NodeHandle node;

//   ros::service::waitForService("spawn");
//   ros::ServiceClient add_turtle = 
//     node.serviceClient<turtlesim::Spawn>("spawn");
//   turtlesim::Spawn srv;
//   add_turtle.call(srv);

//   ros::Publisher turtle_vel = 
//     node.advertise<turtlesim::Velocity>("turtle2/command_velocity", 10);

//   tf::TransformListener listener;

//   ros::Rate rate(10.0);
//   while (node.ok()){
//     tf::StampedTransform transform;
//     try{
//       listener.lookupTransform("/turtle2", "/turtle1",  
//                                ros::Time(0), transform);
//     }
//     catch (tf::TransformException ex){
//       ROS_ERROR("%s",ex.what());
//       ros::Duration(1.0).sleep();
//     }

//     turtlesim::Velocity vel_msg;
//     vel_msg.angular = 4.0 * atan2(transform.getOrigin().y(),
//                                 transform.getOrigin().x());
//     vel_msg.linear = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
//                                 pow(transform.getOrigin().y(), 2));
//     turtle_vel.publish(vel_msg);

//     rate.sleep();
//   }
//   return 0;
// };

// #include <ros/ros.h>
// #include <tf2_msgs/TFMessage.h>

// void getPos(const tf2_msgs::TFMessage _msg)
// {
//   if(!_msg.empty())
//   {
//     // printf(_msg.transforms.getOrigin().x());
//   }
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf_listener2");
  ros::NodeHandle node;
  // ros::Subscriber sub = node.subscribe("/tf", 10, &getPos);
  ros::Publisher path_pub = node.advertise<nav_msgs::Path>("/trajectory/waist_link",10);

  tf::TransformListener listener;
  nav_msgs::Path path;
  path.header.stamp=ros::Time::now();
  path.header.frame_id="dummy";//reference this link postion to setting origen position

  float old_x = 0;
  float old_y = 0;
  float old_z = 0;
  double time_now = ros::Time::now().toSec();
  double time_old = ros::Time::now().toSec();
  ros::Rate rate(100.0);
  while (node.ok())
  {
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("/dummy", "/right_hip_yaw_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    //---
    time_now = ros::Time::now().toSec();
    if((float)transform.getOrigin().x() != old_x || (float)transform.getOrigin().y() != old_y || (float)transform.getOrigin().z() != old_z)
    {
      time_old = time_now;
      // time_now = ros::Time::now().toSec();
      geometry_msgs::PoseStamped this_pose_stamped;
      this_pose_stamped.pose.position.x = (float)transform.getOrigin().x();
      this_pose_stamped.pose.position.y = (float)transform.getOrigin().y()+0.044;
      this_pose_stamped.pose.position.z = (float)transform.getOrigin().z();
      this_pose_stamped.pose.orientation.x = (float)transform.getRotation().x();
      this_pose_stamped.pose.orientation.y = (float)transform.getRotation().y();
      this_pose_stamped.pose.orientation.z = (float)transform.getRotation().z();
      this_pose_stamped.pose.orientation.w = (float)transform.getRotation().w();
      this_pose_stamped.header.stamp = transform.stamp_;
      this_pose_stamped.header.frame_id = transform.child_frame_id_;
      path.poses.push_back(this_pose_stamped);
      path_pub.publish(path);
      old_x = (float)transform.getOrigin().x();
      old_y = (float)transform.getOrigin().y();
      old_z = (float)transform.getOrigin().z();
      // std::printf("x = %f, y = %f, z = %f cm x = %f, y = %f, z = %f, w = %f\n", transform.getOrigin().x()*100, transform.getOrigin().y()*100, transform.getOrigin().z()*100, transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
      std::printf("x = %f, y = %f, z = %f cm\n", transform.getOrigin().x()*100, transform.getOrigin().y()*100, transform.getOrigin().z()*100);
    }
    else if(time_now - time_old > 5)
    {
      time_old = time_now;
      path.poses.clear();
      std::printf("clear\n");
    }
    // std::printf("%f %f\n",time_now, time_old);
    // std::printf("%f\n", transform.getRotation().getAxis().x());

    // std::printf("x = %f, y = %f, z = %f cm\n", transform.getOrigin().x()*100, transform.getOrigin().y()*100, transform.getOrigin().z()*100);
    rate.sleep();
  }
  return 0;
};

