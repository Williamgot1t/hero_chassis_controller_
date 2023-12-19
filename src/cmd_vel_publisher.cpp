#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "cmd_vel_controller/cmd_vel_controller.h"

int main(int argc, char** argv)
{

    //publisher
    ros::init(argc, argv, "car_control");
    ros::NodeHandle n;
    ros::Publisher command_pub = n.advertise<geometry_msgs::Twist>(  "cmd_vel", 1000);
    ros::Rate loop_rate(10);
    while(1)
    {
        if(command_pub.getNumSubscribers()>0)
            break;
        else
            ROS_INFO("cmd_vel not connected!!");
    }
    ROS_INFO("Sending  cmd_vel successfully!!");

    while (ros::ok())
    {
        geometry_msgs::Twist car_cmd_vel;
        car_cmd_vel.linear.x =100;
        car_cmd_vel.linear.y =100;
        car_cmd_vel.linear.z = 100;
        car_cmd_vel.angular.x = 100;
        car_cmd_vel.angular.y = 100;
        car_cmd_vel.angular.z=100;
        command_pub.publish(car_cmd_vel);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}