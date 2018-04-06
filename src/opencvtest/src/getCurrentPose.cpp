
//通过ros发布实时的机器人的信息。

#include <stdio.h>  
#include <arpa/inet.h>  
#include "moveit/move_group_interface/move_group.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include "geometry_msgs/Pose.h"
#define PI 3.14159265354

int main(int argc, char *argv[])  
{  
    ros::init(argc, argv, "hdh");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    
    moveit::planning_interface::MoveGroup group("manipulator");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    moveit::core::RobotState kinematic_state(*group.getCurrentState());
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    geometry_msgs::Pose pose;
    std::vector<double> joint_values;
    joint_values.resize(6);
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pose = group.getCurrentPose(group.getEndEffectorLink()).pose;
        ROS_INFO_STREAM("Current Pose is "<<pose);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;  
}  
