#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <Odom/odom_pub.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <ros/console.h>
#include <unistd.h>

#include <dynamic_tutorial/tutorialConfig.h>
#include <dynamic_reconfigure/client.h>

double x = 0.0;
double y = 0.0;
double th = 0.0;
//默认机器人的起始位置是odom参考系下的0点
double vx = 0;
double vy = 0;
double vth = 0;

//odom_param
double flv=0;
double frv=0;
double blv=0;
double brv=0;

double wheel_track=0.5;
double wheel_base=0.475;


void fl_velocitysubscriber (const std_msgs::Float64ConstPtr &msg)
{
    flv= msg->data;
}
void bl_velocitysubscriber (const std_msgs::Float64ConstPtr &msg)
{
    blv= msg->data;
}
void fr_velocitysubscriber (const std_msgs::Float64ConstPtr &msg)
{
    frv= msg->data;
}
void br_velocitysubscriber (const std_msgs::Float64ConstPtr &msg)
{
    brv= msg->data;
}
void messageCallback(const hero_chassis_controller::odom_pub::ConstPtr& msg)
{
    vx = wheel_track;
    vy = double(msg->back_left_velocity);
    vth =double(msg->front_right_velocity);
}
void dynCallBack(const dynamic_tutorial::tutorialConfig &config)
{
    wheel_base=config.wheel_base;
    wheel_track=config.wheel_track;
    ROS_INFO("vx=%f",config.wheel_track);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_pub");
    ros::NodeHandle n;
    ros::Subscriber front_left_velocity_controller_sub= n.subscribe<std_msgs::Float64>("/controller/front_left_velocity_controller/command", 1, fl_velocitysubscriber);
    ros::Subscriber front_right_velocity_controller_sub= n.subscribe<std_msgs::Float64>("/controller/front_right_velocity_controller/command", 1, fr_velocitysubscriber);
    ros::Subscriber back_left_velocity_controller_sub= n.subscribe<std_msgs::Float64>("/controller/back_left_velocity_controller/command", 1, bl_velocitysubscriber);
    ros::Subscriber back_right_velocity_controller_sub= n.subscribe<std_msgs::Float64>("/controller/back_right_velocity_controller/command", 1, br_velocitysubscriber);
    ros::Subscriber odom_param_sub = n.subscribe("odom_param_pub", 1, messageCallback);
    ros::Publisher  odom_param_pub = n.advertise<hero_chassis_controller::odom_pub>("odom_param_pub", 1);//make a param publisher
    ros::Publisher  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);



    dynamic_reconfigure::Client<dynamic_tutorial::tutorialConfig> dynamic_client("dynamic_client", dynCallBack);
    dynamic_tutorial::tutorialConfig config;




    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time, last_time;
    last_time = ros::Time::now();




    ros::Rate r(20);//以20Hz的速率发布里程信息，
    while(ros::ok())
    {
        //param_pub init
        hero_chassis_controller::odom_pub odom_param_msg;
        odom_param_msg.front_left_velocity=flv;
        odom_param_msg.front_right_velocity=frv;
        odom_param_msg.back_left_velocity=blv;
        odom_param_msg.back_right_velocity=brv;


        ros::spinOnce();
        current_time = ros::Time::now();



        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th)) * dt;
        double delta_y = (vx * sin(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y =vy;
        odom.twist.twist.angular.z = vth;

\
        //publish the message
        odom_pub.publish(odom);
        odom_param_pub.publish(odom_param_msg);
        last_time = current_time;

        r.sleep();
    }
    // return 0;
}