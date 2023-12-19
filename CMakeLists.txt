cmake_minimum_required(VERSION 3.0.2)
project(hero_chassis_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
        angles
        control_msgs
        control_toolbox
        controller_interface
        forward_command_controller
        hardware_interface
        pluginlib
        realtime_tools
        roscpp
        std_msgs
        urdf
        tf          #Odon
        nav_msgs    #Odon
        rospy       #Odon

        message_generation
        geometry_msgs
	    dynamic_reconfigure

        )

add_message_files(FILES odom_pub.msg)
generate_messages(DEPENDENCIES std_msgs)
generate_dynamic_reconfigure_options(
  cfg/tutorial.cfg
)
# Declare catkin package
catkin_package(
        CATKIN_DEPENDS
        control_msgs
        control_toolbox
        controller_interface
        forward_command_controller
        hardware_interface
        realtime_tools
        std_msgs
        urdf

        message_runtime  # must be put here

        LIBRARIES
        ${PROJECT_NAME}
        INCLUDE_DIRS
        include

)


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})


add_library(${PROJECT_NAME}
        src/joint_velocity_controller.cpp
        )



add_executable (odom_tf_pub src/odom_tf_pub.cpp)
target_link_libraries (odom_tf_pub ${catkin_LIBRARIES})
add_dependencies(odom_tf_pub ${PROJECT_NAME}_generate_messages_cpp)



add_executable (cmd_vel_publisher src/cmd_vel_publisher.cpp)
target_link_libraries (cmd_vel_publisher ${catkin_LIBRARIES})
add_dependencies(cmd_vel_publisher ${PROJECT_NAME}_generate_messages_cpp)

add_executable(dynamic_tutorial_node src/dynamic_tutorial.cpp)
add_dependencies(dynamic_tutorial_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamic_tutorial_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

# Install targets
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

# Install plugins
install(FILES effort_controllers_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
