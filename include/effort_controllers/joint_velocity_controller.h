#ifndef JOINT_VELOCITY_CONTROLLER_H
#define JOINT_VELOCITY_CONTROLLER_H


#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <memory>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>

#include <iostream>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/console.h>
#include<unistd.h>

#include "Odom/odom_tf_pub.h"


namespace effort_controllers
{

    class JointVelocityController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:

        JointVelocityController();
        ~JointVelocityController();

        bool init(hardware_interface::EffortJointInterface *robot, const std::string &joint_name, const control_toolbox::Pid &pid);

        bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

        void starting(const ros::Time& time);

        void update(const ros::Time& time, const ros::Duration& period);

        void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);

        double front_left_command_,front_right_command_,back_right_command_,back_left_command_,v1,v2,v3,v4;
        hardware_interface::JointHandle front_left_joint_,back_left_joint_,front_right_joint_,back_right_joint_;
    private:
        int loop_count_;
        control_toolbox::Pid pid_controller_;

        std::unique_ptr<
                realtime_tools::RealtimePublisher<
                        control_msgs::JointControllerState> > controller_state_publisher_ ;

        ros::Subscriber sub_command_;

        void setCommandFL(const std_msgs::Float64ConstPtr& msg);
        void setCommandFR(const std_msgs::Float64ConstPtr& msg);
        void setCommandBL(const std_msgs::Float64ConstPtr& msg);
        void setCommandBR(const std_msgs::Float64ConstPtr& msg);
        void getcmd(const geometry_msgs::TwistConstPtr& twist);

    };

} // namespace

#endif
