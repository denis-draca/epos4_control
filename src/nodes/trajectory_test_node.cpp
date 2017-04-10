#include <ros/ros.h>
#include <ros/spinner.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>

class RobotArm
{

public:
    typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

private:
    // Action client for the joint trajectory action
    // used to trigger the arm movement action
    TrajClient* traj_client_;

public:
    //! Initialize the action client and wait for action server to come up
    RobotArm()
    {
        // tell the action client that we want to spin a thread by default
        traj_client_ = new TrajClient("/SL_Back/joint_trajectory_controller/follow_joint_trajectory", true);

        // wait for action server to come up
        while(!traj_client_->waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the joint_trajectory_action server");
        }
    }

    //! Clean up the action client
    ~RobotArm()
    {
        delete traj_client_;
    }

    //! Sends the command to start a given trajectory
    void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
    {
        // When to start the trajectory: 1s from now
        goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);
        traj_client_->sendGoal(goal);
    }

    //! Generates a simple trajectory with two waypoints, used as an example
    /*! Note that this trajectory contains two waypoints, joined together
    as a single trajectory. Alternatively, each of these waypoints could
    be in its own trajectory - a trajectory can have one or more waypoints
    depending on the desired application.
    */
    control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory()
    {
        //our goal variable
        control_msgs::FollowJointTrajectoryGoal goal;

        // First, the joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back("SL_Back_Left");
        goal.trajectory.joint_names.push_back("SL_Back_Right");

        int dof = goal.trajectory.joint_names.size();
        int traj_size = 3;

        // We will have two waypoints in this goal trajectory
        goal.trajectory.points.resize(traj_size);

        // First trajectory point
        // Positions
        int ind = 0;
        goal.trajectory.points[ind].positions.resize(dof);
        goal.trajectory.points[ind].positions[0] = 0.0;
        goal.trajectory.points[ind].positions[1] = 0.0;

        // Velocities
        goal.trajectory.points[ind].velocities.resize(dof);
        for (size_t j = 0; j < dof; ++j)
        {
            goal.trajectory.points[ind].velocities[j] = 0.0;
        }
        // To be reached 1 second after starting along the trajectory
        goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

        // Second trajectory point
        // Positions
        ind += 1;
        goal.trajectory.points[ind].positions.resize(dof);
        goal.trajectory.points[ind].positions[0] = 2000;
        goal.trajectory.points[ind].positions[1] = 2000;

        // Velocities
        goal.trajectory.points[ind].velocities.resize(dof);
        for (size_t j = 0; j < dof; ++j)
        {
            goal.trajectory.points[ind].velocities[j] = 0.0;
        }
        // To be reached 2 seconds after starting along the trajectory
        goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

        // Second trajectory point
        // Positions
        ind += 1;
        goal.trajectory.points[ind].positions.resize(dof);
        goal.trajectory.points[ind].positions[0] = 1000;
        goal.trajectory.points[ind].positions[1] = 1000;

        // Velocities
        goal.trajectory.points[ind].velocities.resize(dof);
        for (size_t j = 0; j < dof; ++j)
        {
            goal.trajectory.points[ind].velocities[j] = 0.0;
        }
        // To be reached 2 seconds after starting along the trajectory
        goal.trajectory.points[ind].time_from_start = ros::Duration(2.5);

        //we are done; return the goal
        return goal;
    }

    //! Returns the current state of the action
    actionlib::SimpleClientGoalState getState()
    {
        ROS_INFO("getState");
        return traj_client_->getState();
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_test_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    RobotArm arm;
    arm.startTrajectory(arm.armExtensionTrajectory());
    ROS_INFO("Sent");
}
