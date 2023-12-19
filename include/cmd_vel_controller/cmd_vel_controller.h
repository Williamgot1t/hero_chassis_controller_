
#ifndef CMD_VEL_CONTROLLER_H
#define CMD_VEL_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include<iostream>

namespace cmd_vel_controller{

    class CmdVelController :
            public controller_interface::Controller<hardware_interface::EffortJointInterface> {
    public:
        CmdVelController()= default ;
        ~CmdVelController() override = default;

        bool init(hardware_interface::EffortJointInterface *effort_joint_interface,ros::NodeHandle &n) override;

        void update(const ros::Time &time, const ros::Duration &period) override;
        void getcmd(const geometry_msgs::TwistConstPtr& twist);

        hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
        double v1,v2,v3,v4;
        ros::Subscriber sub_command_;

    };
}// namespace

#endif
