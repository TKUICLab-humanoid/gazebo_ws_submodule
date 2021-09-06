
#include <gazebo/gazebo.hh>
#include <vector>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include "ros/ros.h"

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      double a, w ,t, x, y ,z;
      
      ros::spinOnce();
      ros::Time time = ros::Time::now();
      
      t = time.toSec();
      a = 0.2;//旋轉半徑
      w = 4;//2pi/w=週期
      x= 1.2;//與機器人直線距離  1.2極限
      y= 0+a*cos(w*(t)); 
      z= 0.5+a*sin(w*(t)); //0.5-a＝靶高

      // Apply a small linear velocity to the model.
      this->model->SetWorldPose(ignition::math::Pose3d(x, y, z, 0, -1.57, 0),
		                                  1,
		                                  1 
	                                  );
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}

