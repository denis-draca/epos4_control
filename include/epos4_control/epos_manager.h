#ifndef EPOS_HARDWARE_EPOS_MANAGER_H_
#define EPOS_HARDWARE_EPOS_MANAGER_H_

#include <ros/ros.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include "epos4_control/utils.h"
#include "epos4_control/epos.h"


namespace epos4_control {

class EposManager {
public:
    EposManager(hardware_interface::ActuatorStateInterface& asi,
              hardware_interface::VelocityActuatorInterface& avi,
              hardware_interface::PositionActuatorInterface& api,
              hardware_interface::JointStateInterface &jsi,
              hardware_interface::PositionJointInterface &jpi,
              hardware_interface::VelocityJointInterface &jvi,
              hardware_interface::EffortJointInterface &jei,
              ros::NodeHandle& nh, ros::NodeHandle& pnh,
              const std::vector<std::string>& motor_names);
    bool init();
    void read();
    void write();
    void update_diagnostics();

    std::vector<boost::shared_ptr<Epos> > motors() { return motors_; }

private:
    std::vector<boost::shared_ptr<Epos> > motors_;
    EposFactory epos_factory;

    hardware_interface::ActuatorStateInterface* asi_;
    hardware_interface::VelocityActuatorInterface* avi_;
    hardware_interface::PositionActuatorInterface* api_;

    hardware_interface::JointStateInterface* jsi_;
    hardware_interface::PositionJointInterface* jpi_;
    hardware_interface::VelocityJointInterface* jvi_;
    hardware_interface::EffortJointInterface* jei_;
};

}


#endif
