#include <ros/ros.h>
#include <ros/spinner.h>
#include "epos4_control/epos4_control.h"
#include <controller_manager/controller_manager.h>
#include "epos4_control/epos4_trajectory_commander.h"
#include <vector>

int main(int argc, char** argv) {
    ros::init(argc, argv, "epos_trajectory_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string robot_ns = argv[1];
    ROS_INFO_STREAM("robot_ns="<<robot_ns);

    std::vector<std::string> motor_names;
    for(int i = 0; i < argc-2; ++i)
    {
        motor_names.push_back(argv[i+2]);
    }

    epos4_control::epos4_trajectory_commander traj_commander(motor_names, robot_ns);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // subscribers
    ros::Subscriber add_traj_sub = nh.subscribe("/add_goal_trajectory_", 10, &epos4_control::epos4_trajectory_commander::Add, &traj_commander);
    ros::Subscriber execute_traj_sub = nh.subscribe("/execute_goal_trajectory_", 10, &epos4_control::epos4_trajectory_commander::Execute, &traj_commander);

}
