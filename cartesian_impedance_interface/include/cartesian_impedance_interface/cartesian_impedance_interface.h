#ifndef CARTESIAN_IMPEDANCE_INTERFACE_H
#define CARTESIAN_IMPEDANCE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_command_interface.h>


namespace hardware_interface
{

// Commanding Cartesian position with native KUKA impedance controller
class PositionCartesianInterface : public JointCommandInterface {};

}  // end namespace hardware interface

#endif
