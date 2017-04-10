#ifndef EPOS_TRAJECTORY_COMMANDER_H_
#define EPOS_TRAJECTORY_COMMANDER_H_

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "epos4_control/epos4_control.h"
#include <controller_manager/controller_manager.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <vector>
#include <algorithm>

#include "../ClassicTrajectory/BlendedTrajectory.h"

namespace epos4_control {

#define PRO_MAX_SPEED_RPM           13.0
#define MILLISECONDS_PER_SECOND     1000.0
#define SECONDS_PER_MINUTE          60.0
#define MILLISECONDS_PER_MINUTE     60000.0
#define DEGREES_PER_REVOLUTION      360.0
#define PRO_MAX_ACCELERATION_DEGREES_PER_SECOND_PER_SECOND  12000
#define DOF                         2

class epos4_trajectory_commander{

public:
    typedef ClassicTrajectory::BlendedTrajectory<DOF> TrajectoryGeneratorType;
    typedef TrajectoryGeneratorType::TrajectoryStoreType TrajectoryGeneratorStoreType;

    typedef TrajectoryGeneratorType::JointStateType JointStateType;
    typedef std::vector<std::pair<JointStateType,double> > JointTrajectoryTimePairType;
    typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

    typedef float RealType;
    typedef int MillisecondType;



public:
    epos4_trajectory_commander(const std::vector<std::string>& motor_names, const std::string &ns);

protected:
    bool m_started;                                             //!< Is the command active
    bool m_completed;                                           //!< The flag set when the trajectory is complete
    bool m_paused;                                              //!< Pause the motion

    std::vector<std::string> m_motor_names;                     //!< motor names

    JointTrajectoryTimePairType m_targetTrajectoryTimePair;     //!< target trajectory

    TrajClient* traj_client_;                                   //!< Action client for the joint trajectory action used to trigger the arm movement action

    RealType m_segmentationThreshold;                           //!< The threshold for when a large joint movement should be segmented for smooth motion
    RealType m_accelerationCoefficient;                         //!< acceleration Coefficient
    RealType m_stepResolutionDegrees;                           //!< In trajectory generation this is the smallest step size that is generating in interpolations


public:
    /// Stop Movement
    void Stop(){}

//    /// Add \c jointState with a duration of \c timeOffset
//    void Add(const JointStateType& jointState, double timeOffset);

    /// Add \c jointState from ROS's control_msgs::FollowJointTrajectoryGoal
    void Add(const control_msgs::FollowJointTrajectoryGoal& traj);

    /// Execute method
    void Execute(const std_msgs::StringConstPtr &str);

    /// Clear Traj
    void Clear(){ m_targetTrajectoryTimePair.clear(); }

public: // Accessors
    /// IsStarted
    bool IsStarted(){ return m_started; }

    /// IsComplete
    bool IsCompleted(){ return m_completed; }

    /// IsComplete
    bool IsPaused(){ return m_paused; }

private:
    void InterpolateTrajectory(const JointStateType &jointState, MillisecondType timeOffset);

    /// Get the maximum speed (degrees per milliseconds)
    RealType GetMaxSpeedDegreesPerMillisecond() { return RealType(PRO_MAX_SPEED_RPM * DEGREES_PER_REVOLUTION / MILLISECONDS_PER_MINUTE);}

    /// Get the maximum speed (degrees per second)
    RealType GetMaxSpeedDegreesPerSecond() { return RealType(PRO_MAX_SPEED_RPM * DEGREES_PER_REVOLUTION / SECONDS_PER_MINUTE);}

    /// GetMaxAccelerationDegreesPerSecondPerSecond
    RealType GetMaxAccelerationDegreesPerSecondPerSecond(){ return PRO_MAX_ACCELERATION_DEGREES_PER_SECOND_PER_SECOND; }


    /// Calculate the degrees travelled and time taken during acceleration to max velocity
    RealType GetDegreesTraveledDuringAcceleration()
    {
        RealType degreesTraveledDuringAcceleration = 0.5 * GetAccelerationSeconds() * GetMaxSpeedDegreesPerSecond();
        return degreesTraveledDuringAcceleration;
    }

    /// GetAccelerationSeconds
    RealType GetAccelerationSeconds()
    {
        RealType accelerationSeconds = GetMaxSpeedDegreesPerSecond() / GetMaxAccelerationDegreesPerSecondPerSecond();
        return accelerationSeconds;
    }

    float MaxDifferenceInDegree(const JointStateType &jointState1, const JointStateType &jointState2)
    {
        std::vector<int> v;
        int size = jointState1.size()<jointState2.size()?jointState2.size():jointState1.size();
        for(int i=0;i<size;++i)
        {
            v.push_back(abs(jointState1[i]-jointState2[i]));
        }
        return *std::max_element(v.begin(), v.end());
    }
};

}


#endif
