#ifndef LWR_CONTROLLERS__CARTESIAN_IMPEDANCE_CONTROLLER_H
#define LWR_CONTROLLERS__CARTESIAN_IMPEDANCE_CONTROLLER_H

// ROS added
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <lwr_controllers/Stiffness.h>
#include <std_msgs/Float64MultiArray.h>

// KDL added
#include <kdl/stiffness.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/frames.hpp>

// BOOST added
#include <boost/scoped_ptr.hpp>

// Base class with useful URDF parsing and kdl chain generator
#include "lwr_controllers/KinematicChainControllerBase.h"

// The format of the command specification
#include "lwr_controllers/SetCartesianImpedanceCommand.h"

// to read KDL chain for the world robot transform
#include "KinematicChainControllerBase.h"

// include new cartesian impedance interface
#include <cartesian_impedance_interface/cartesian_impedance_interface.h>

#include <Eigen/Eigen>

namespace lwr_controllers
{
    //class ITRCartesianImpedanceController: public controller_interface::Controller<hardware_interface::PositionCartesianInterface>
    class ITRCartesianImpedanceController: public controller_interface::KinematicChainControllerBase<hardware_interface::PositionCartesianInterface>
	{
	public:
        ITRCartesianImpedanceController();
        ~ITRCartesianImpedanceController();

        bool init(hardware_interface::PositionCartesianInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void stopping(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
        //void command(const geometry_msgs::PoseConstPtr &msg);
        void pose(const geometry_msgs::PoseConstPtr &msg);
        //void command(const lwr_controllers::CartesianImpedancePoint::ConstPtr &msg);
        bool command_cb(lwr_controllers::SetCartesianImpedanceCommand::Request &req, lwr_controllers::SetCartesianImpedanceCommand::Response &res);
        //void updateFT(const geometry_msgs::WrenchStamped::ConstPtr &msg);
        void additionalFT(const geometry_msgs::WrenchStamped::ConstPtr &msg);
        void gains(const std_msgs::Float64MultiArrayConstPtr &msg);

        // set a world pose, which is then transformed into the robot KS
        void pose_base_link(const geometry_msgs::PoseConstPtr &msg);

    protected:

        std::string robot_namespace_;
        std::vector<std::string> joint_names_, cart_12_names_, cart_6_names_;
        std::vector<hardware_interface::JointHandle> joint_handles_, cart_handles_;
		
        // ROS API (topic, service and dynamic reconfigure)
        ros::Subscriber sub_pose_;
        ros::Subscriber sub_pose_world_;
        ros::Subscriber sub_gains_;
        ros::Subscriber sub_addFT_;

        ros::ServiceServer srv_command_;
        ros::Subscriber sub_ft_measures_;
        //ros::Publisher pub_goal_;
        ros::Publisher pub_msr_pos_;

		// Cartesian vars
        KDL::Frame x_ref_;
        KDL::Frame x_des_;
        KDL::Frame x_cur_;
        KDL::Frame x_prev_;
        KDL::Frame x_FRI_;
		KDL::Twist x_err_;
        KDL::Frame x_set_;

        Eigen::Quaterniond x_prev_quat_;
        Eigen::Quaterniond x_cur_quat_;
        Eigen::Quaterniond x_des_quat_;
        Eigen::Quaterniond x_set_quat_;

        KDL::Twist x_dot_;

        // transform of the robot mounting position
        KDL::Frame base_link2robot_;
        // base_link coordinates of received command
        KDL::Frame x_base_link_;

		// The jacobian at q_msg
        // KDL::Jacobian J_;

        // Measured external force
        KDL::Wrench f_cur_;
        KDL::Wrench f_des_;
        // KDL::Wrench f_error_;

		// That is, desired kx, ky, kz, krx, kry, krz, they need to be expressed in the same ref as x, and J
		KDL::Stiffness k_des_;
        KDL::Stiffness d_des_;

        bool cmd_flag_;

        double max_trans_speed_; // m/s
        double max_rot_speed_;   // rad/s

		// Because of the lack of the Jacobian transpose in KDL
		void multiplyJacobian(const KDL::Jacobian& jac, const KDL::Wrench& src, KDL::JntArray& dest);

        // FRI<->KDL conversion
        void fromKDLtoFRI(const KDL::Frame& in, std::vector<double>& out);
        void fromKDLtoFRI(const KDL::Stiffness& in, std::vector<double>& out);
        void fromKDLtoFRI(const KDL::Wrench& in, std::vector<double>& out);
        void fromFRItoKDL(const std::vector<double>& in, KDL::Frame& out);
        void fromFRItoKDL(const std::vector<double>& in, KDL::Stiffness& out);
        void fromFRItoKDL(const std::vector<double>& in, KDL::Wrench& out);
	};

}

#endif
