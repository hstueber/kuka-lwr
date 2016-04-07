
#ifndef LWR_CONTROLLERS__LWR_STATE_CONTROLLER_H
#define LWR_CONTROLLERS__LWR_STATE_CONTROLLER_H

#include <lwr_controllers/KinematicChainControllerBase.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <kdl/stiffness.hpp>
#include <kdl_conversions/kdl_msg.h>

// Base class with useful URDF parsing and kdl chain generator
#include "lwr_controllers/KinematicChainControllerBase.h"

namespace lwr_controllers
{
    class LWRStateController: public controller_interface::KinematicChainControllerBase<hardware_interface::JointStateInterface>
	{
	public:

        LWRStateController();
        ~LWRStateController();

        bool init(hardware_interface::JointStateInterface *robot, ros::NodeHandle &n);

		void starting(const ros::Time& time);

		void update(const ros::Time& time, const ros::Duration& period);

	private:
        std::string robot_namespace_;
        std::vector<std::string>
            joint_names_,
            cart_12_names_,
            cart_6_names_;

        std::vector<hardware_interface::JointStateHandle>
            joint_state_handles_,
            joint_state_handles_estExtTrq_,
            cart_state_handles_;

        // ROS API (topic, service and dynamic reconfigure)
        ros::Publisher
            pub_msr_joint_state_,
            pub_msr_cart_wrench_,
            pub_msr_cart_pos_,
            pub_msr_cart_pos_base_link_;

        sensor_msgs::JointState joint_state_msg_;
        geometry_msgs::PoseStamped pose_msg_, pose_base_link_msg_;
        geometry_msgs::WrenchStamped ext_wrench_msg_;

        // Cartesian vars
        KDL::Frame x_msr_, x_msr_base_link_;
        KDL::Frame x_FRI_;
        KDL::Wrench estExtTcpFT_;

        // FRI<->KDL conversion
        void fromFRItoKDL(const std::vector<double>& in, KDL::Frame& out);
        void fromFRItoKDL(const std::vector<double>& in, KDL::Stiffness& out);
        void fromFRItoKDL(const std::vector<double>& in, KDL::Wrench& out);

        // transform of the robot mounting position
        KDL::Frame base_link2robot_;

	};

} // namespace

#endif
