#ifndef LWR_CONTROLLERS__ONE_TASK_INVERSE_KINEMATICS_H
#define LWR_CONTROLLERS__ONE_TASK_INVERSE_KINEMATICS_H

#include "KinematicChainControllerBase.h"
#include "lwr_controllers/PoseRPY.h"

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
//#include <sstream>

// add include for new command topics pose
#include <geometry_msgs/PoseStamped.h>

namespace lwr_controllers
{
	class CartesianPosition: public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
	{
	public:
		CartesianPosition();
		~CartesianPosition();

		bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);

        void pose(const geometry_msgs::PoseConstPtr &msg);
        void pose_base_link(const geometry_msgs::PoseConstPtr &msg);
        void command(const lwr_controllers::PoseRPY::ConstPtr &msg);

	private:
		ros::Subscriber sub_command_;
		ros::Subscriber sub_gains_;

		KDL::Frame x_;		//current pose
		KDL::Frame x_des_;	//desired pose
		KDL::Twist v_des_;      //desired velocity

        // transform of the robot mounting position
        KDL::Frame base_link2robot_;
        // base_link coordinates of received command
        KDL::Frame x_base_link_;

        std::string robot_namespace_;

		ros::Time cmd_stamp_; // Timestamp of last command
		KDL::Twist x_err_;

		KDL::JntArray q_cmd_; // computed set points

		KDL::Jacobian J_;	//Jacobian

		Eigen::MatrixXd J_pinv_;
		Eigen::Matrix<double,3,3> skew_;

		struct quaternion_
		{
			KDL::Vector v;
			double a;
		} quat_curr_, quat_des_;

		KDL::Vector v_temp_;
		
		int cmd_flag_;

		KDL::JntArrayAcc joint_des_states_last_step;
		
		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
		boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
		boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;
	};

}

#endif
