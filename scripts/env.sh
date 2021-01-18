#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source ${DIR}/../../../devel/setup.bash
My_Gazebo_Model_Path="$( cd "${DIR}/../models" && pwd )"
export GAZEBO_MODEL_PATH=$My_Gazebo_Model_Path:
alias add_model="ls ${My_Gazebo_Model_Path} && rosrun fira_gz add_model"
#alias gz_kidsize="roslaunch fira_gz rrbot_world.launch"
#alias gz_kidsize_control="roslaunch urdf nimbro_op_controllers.launch"
#alias imageprocess="roslaunch imageprocess imageprocess.launch"
#alias motionpackage="rosrun motionpackage motionpackage"
#alias walkinggait="rosrun walkinggait walkinggait"
alias kidsize_make="catkin_make --pkg kidsize_msgs kidsize_libs; catkin_make"
#echo ${My_Gazebo_Model_Path}
#echo ${GAZEBO_MODEL_PATH}

