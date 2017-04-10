#ifndef EPOS_HARDWARE_EPOS_HARDWARE_H_
#define EPOS_HARDWARE_EPOS_HARDWARE_H_

#include <ros/ros.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>
#include "epos4_control/utils.h"
#include "epos4_control/epos.h"
#include "epos4_control/epos_manager.h"


namespace epos4_control {

class EposHardware : public hardware_interface::RobotHW {
public:
    EposHardware(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::vector<std::string>& motor_names);
    bool init();
    void read();
    void write();
    void update_diagnostics();
private:
    hardware_interface::ActuatorStateInterface asi;
    hardware_interface::VelocityActuatorInterface avi;
    hardware_interface::PositionActuatorInterface api;

    hardware_interface::JointStateInterface jsi;
    hardware_interface::PositionJointInterface jpi;
    hardware_interface::VelocityJointInterface jvi;
    hardware_interface::EffortJointInterface jei;

    EposManager epos_manager_;

    transmission_interface::RobotTransmissions robot_transmissions;
    boost::scoped_ptr<transmission_interface::TransmissionInterfaceLoader> transmission_loader;
};

}


#endif
