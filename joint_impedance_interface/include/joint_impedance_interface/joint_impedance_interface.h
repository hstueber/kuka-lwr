#ifndef JOINT_IMPEDANCE_INTERFACE_H
#define JOINT_IMPEDANCE_INTERFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>


namespace hardware_interface
{

class ImpedanceJointHandle : public JointStateHandle
{
public:
    ImpedanceJointHandle() : JointStateHandle(), cmd_position_(0), cmd_effort_(0), cmd_stiffness_(0), cmd_damping_(0) {}

    ImpedanceJointHandle(const JointStateHandle& js, double* cmd_position, double* cmd_effort, double* cmd_stiffness, double* cmd_damping)
      : JointStateHandle(js), cmd_position_(cmd_position), cmd_effort_(cmd_effort), cmd_stiffness_(cmd_stiffness), cmd_damping_(cmd_damping)
    {
        if (!cmd_position) {
            throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command position pointer is null.");
        }
        if (!cmd_effort) {
            throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command effort pointer is null.");
        }
        if (!cmd_stiffness) {
            throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command stiffness pointer is null.");
        }
        if (!cmd_damping) {
            throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command damping pointer is null.");
        }
        /*
        cmd_position_ = cmd_position;
        cmd_effort_ = cmd_effort;
        cmd_stiffness_ = cmd_stiffness;
        cmd_damping_ = cmd_damping;
        */
    }

    void setCommand(double cmd_position, double cmd_effort, double cmd_stiffness, double cmd_damping)
    {
        assert(cmd_position_); assert(cmd_effort_); assert(cmd_stiffness_); assert(cmd_damping_);
        // std::cout << "passed assert, writing to pointers" << std::endl;
        *cmd_position_ = cmd_position;
        *cmd_effort_ = cmd_effort;
        *cmd_stiffness_ = cmd_stiffness;
        *cmd_damping_ = cmd_damping;
        // std::cout << "passed setCommand function" << std::endl;
    }

private:
    double* cmd_position_;
    double* cmd_effort_;
    double* cmd_stiffness_;
    double* cmd_damping_;

};


// Commanding Joints with native KUKA impedance controller
class ImpedanceJointInterface : public HardwareResourceManager<ImpedanceJointHandle, ClaimResources> {};

}  // end namespace hardware interface

#endif
