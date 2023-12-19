#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_tutorial/tutorialConfig.h>


/** 
 * @brief 回调函数-收到reconfigure时调用
 * @param config    tutorialConfig引用对象    
 * @param level     优先级
 *
 * @return 无
 */
void callback(dynamic_tutorial::tutorialConfig &config,uint32_t level)
{
    ROS_INFO("Reconfigure request:%f,%f",
             config.wheel_base,config.wheel_track);
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"dynamic_tutorial_node");
    ros::NodeHandle nh;
    //定义一个动态参数配置服务
    dynamic_reconfigure::Server<dynamic_tutorial::tutorialConfig> server;
    //定义一个动态参数配置服务的回调type
    dynamic_reconfigure::Server<dynamic_tutorial::tutorialConfig>::CallbackType f;
    //绑定callback函数
    f = boost::bind(&callback,_1,_2);
    //设置callback函数
    server.setCallback(f);

    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}
