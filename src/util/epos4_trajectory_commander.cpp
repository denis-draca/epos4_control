#include "epos4_control/epos4_trajectory_commander.h"
#include <boost/foreach.hpp>

namespace epos4_control {

epos4_trajectory_commander::epos4_trajectory_commander(const std::vector<std::string>& motor_names, const std::string& ns)
    : m_started(false)
    , m_completed(true)
    , m_paused(false)
    , m_motor_names(motor_names)
    , m_segmentationThreshold(4.0)
    , m_accelerationCoefficient(1.0)
    , m_stepResolutionDegrees(1.6)
{
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("/"+ns+"/joint_trajectory_controller/follow_joint_trajectory", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the joint_trajectory_action server");
    }
}

//    void
//epos4_trajectory_commander::Add(const JointStateType &jointState, double timeOffset)
//{
//    m_targetTrajectoryTimePair.push_back(std::make_pair(jointState,timeOffset));
//}

    void
epos4_trajectory_commander::Add(const control_msgs::FollowJointTrajectoryGoal& traj)
{
    if(traj.trajectory.points.size() == 0) { return; }
    if(traj.trajectory.points[0].positions.size() != DOF){ return; }


    double prev_time = traj.trajectory.points[0].time_from_start.toSec();
    for(int i = 0; i < (int)traj.trajectory.points.size(); ++i)
    {
        JointStateType jointState;
        for(int j=0;j<(int)traj.trajectory.points[i].positions.size();++j)
        {
            jointState[j] = (traj.trajectory.points[i].positions[j]);
        }
        int millisecTimeOffset = int(MILLISECONDS_PER_SECOND*(traj.trajectory.points[i].time_from_start.toSec() - prev_time));


        InterpolateTrajectory(jointState,millisecTimeOffset);


        prev_time = traj.trajectory.points[i].time_from_start.toSec();
    }
}

    void
epos4_trajectory_commander::Execute(const std_msgs::StringConstPtr& str)
{
    ROS_INFO_STREAM("Execute: " << str);

    control_msgs::FollowJointTrajectoryGoal goal;

    BOOST_FOREACH(const std::string& motor_name, m_motor_names) {
      ROS_INFO_STREAM("Execute: initialising motor " << motor_name);
      goal.trajectory.joint_names.push_back(motor_name);
    }

    int dof = goal.trajectory.joint_names.size();
    int traj_size = m_targetTrajectoryTimePair.size();

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(traj_size);

    double time_accumulated = 0.0;

    for(int ind = 0; ind < traj_size; ++ind)
    {
        JointStateType jointstate = m_targetTrajectoryTimePair[ind].first;
        time_accumulated += m_targetTrajectoryTimePair[ind].second;

        // Positions & Velocities
        goal.trajectory.points[ind].positions.resize(dof);
        for(int i=0;i<dof;++i)
        {
            goal.trajectory.points[ind].positions[i] = jointstate[i];
            goal.trajectory.points[ind].velocities[i] = 0.0;
        }

        // To be reached 1 second after starting along the trajectory
        goal.trajectory.points[ind].time_from_start = ros::Duration(time_accumulated);
    }

    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);
    traj_client_->sendGoal(goal);
}

    void
epos4_trajectory_commander::InterpolateTrajectory(const JointStateType& jointState, MillisecondType timeOffset)
{
    // Smoothing algorithm for m_targetTrajectoryTimePair
    RealType maxDegrees = MaxDifferenceInDegree( jointState , m_targetTrajectoryTimePair.back().first );
    RealType degreesTraveledDuringAcceleration = GetDegreesTraveledDuringAcceleration();
    RealType remainingDegrees = maxDegrees - degreesTraveledDuringAcceleration;
    RealType remainingMilliseconds = remainingDegrees / GetMaxSpeedDegreesPerMillisecond();
    RealType pathTotalMilliseconds = degreesTraveledDuringAcceleration + remainingMilliseconds;

//    LOG4CXX_DEBUG(m_log, "Add: remainingMilliseconds if at max velocity " << remainingMilliseconds << ", pathTotalMilliseconds " << pathTotalMilliseconds)

    // If the time offset is less than the absolute maximum path speed then change it
    if (timeOffset < pathTotalMilliseconds)
        timeOffset = pathTotalMilliseconds;

//    LOG4CXX_DEBUG(m_log, "Add: jointState " << jointState
//                  << ", -  m_trajectory.back().first " <<  m_trajectory.back().first
//                  << ".Abs() " << (jointState - m_trajectory.back().first).Abs()
//                  << ".Max() " << (jointState - m_trajectory.back().first).Abs().Max()
//                  << " < m_segmentationThreshold " << m_segmentationThreshold
//                  << " 1/0? " << ((jointState - m_trajectory.back().first).Abs().Max() < m_segmentationThreshold))

    // If the movment is small between previous joint state in trajectory then just add it with the time
    if ( MaxDifferenceInDegree(jointState , m_targetTrajectoryTimePair.back().first) < m_segmentationThreshold )
    {
//        LOG4CXX_DEBUG(m_log, "Add: Movement is small so adding jointState " << jointState << " to be completed in timeOffset ms " << timeOffset)
        m_targetTrajectoryTimePair.push_back(std::make_pair(jointState,timeOffset));
    }
    else // We have a large joint movement, convert into blended trajectory for smoother motion
    {
//        LOG4CXX_DEBUG(m_log, "Add: There is a large joint movement so blending the trajectory");
        TrajectoryGeneratorStoreType blend;
        TrajectoryGeneratorType trajGen;

        trajGen.SetQddMax( D2R( TrajectoryGeneratorType::JointStateType(30,30) * m_accelerationCoefficient) );
//        LOG4CXX_DEBUG(m_log, "Add: SetQddMax radians " << D2R(DynamixelMotionProfile::GetInstance()->GetMaxAcceleration() * m_accelerationCoefficient));

        trajGen.SetStepResolution( D2R(m_stepResolutionDegrees) );
//        LOG4CXX_DEBUG(m_log, "Add: m_stepResolutionDegrees " << m_stepResolutionDegrees);

        trajGen.SetQ0( D2R(m_targetTrajectoryTimePair.back().first) );
//        LOG4CXX_DEBUG(m_log, "Add: SetQ0 D2R(m_trajectory.back().first) " << D2R(m_trajectory.back().first));

        trajGen.SetQ1( D2R(jointState) );
//        LOG4CXX_DEBUG(m_log, "Add: SetQ1 D2R(jointState) " << D2R(jointState));

        trajGen.SetTotalTime( RealType(timeOffset) / MILLISECONDS_PER_SECOND );
//        LOG4CXX_DEBUG(m_log, "Add: RealType(timeOffset) / MILLISECONDS_PER_SECOND " << RealType(timeOffset) / MILLISECONDS_PER_SECOND);

        trajGen.ComputeTrajectory( blend );
//        LOG4CXX_DEBUG(m_log, "Add: Computed blend.size()" << blend.size())

        int cumulativeMilliseconds = 0;
        for (int i=1; i<int(blend.size())-1; ++i)
        {
            const int millisecondsOffset = int(MILLISECONDS_PER_SECOND * (blend[i].t - blend[i-1].t));
            cumulativeMilliseconds += millisecondsOffset;
//            LOG4CXX_DEBUG(m_log, "Add: blend jointState " << R2D(blend[i].q) << " millisecondsOffset " << millisecondsOffset)
            m_targetTrajectoryTimePair.push_back( std::make_pair( R2D(blend[i].q), millisecondsOffset ) );
        }

        const int millisecondsOffset = int(MILLISECONDS_PER_SECOND * (blend[blend.size()-1].t - blend[blend.size()-2].t));
//        LOG4CXX_DEBUG(m_log, "Add: IGNORING this final blend step jointState " << R2D(blend[blend.size()-1].q) << " millisecondsOffset " << millisecondsOffset
//                << " since it is often errornous. Adding goal and remaining time as final step.")

        // Adding the final step
        if (cumulativeMilliseconds < timeOffset)
        {
//            LOG4CXX_DEBUG(m_log, "Add: final step jointState " << jointState << ", timeOffset-cumulativeMilliseconds " << timeOffset-cumulativeMilliseconds)
            m_targetTrajectoryTimePair.push_back( std::make_pair( jointState, timeOffset-cumulativeMilliseconds));
        }
        else
        {
//            LOG4CXX_WARN(m_log, "Add: timeOffset is less cumulativeMilliseconds so adding long final step jointState " << jointState << ", timeOffset " << timeOffset)
            m_targetTrajectoryTimePair.push_back( std::make_pair( jointState, timeOffset));
        }
    }
}

}
