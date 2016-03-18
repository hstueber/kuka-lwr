
#ifndef LWR_CONTROLLERS__ITR_JOINT_INPEDANCE_CONTROLLER_H
#define LWR_CONTROLLERS__ITR_JOINT_INPEDANCE_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>

#include <boost/scoped_ptr.hpp>
#include <joint_impedance_interface/joint_impedance_interface.h>


namespace lwr_controllers
{

    class ITRJointImpedanceController: public controller_interface::KinematicChainControllerBase<hardware_interface::ImpedanceJointInterface>
    {
    public:

        ITRJointImpedanceController();
        ~ITRJointImpedanceController();

        bool init(hardware_interface::ImpedanceJointInterface *robot, ros::NodeHandle &n);

        void starting(const ros::Time& time);

        void update(const ros::Time& time, const ros::Duration& period);
        // void command(const std_msgs::Float64MultiArray::ConstPtr &msg);
        void setPosition(const std_msgs::Float64MultiArray::ConstPtr &msg);
        void setAddTorque(const std_msgs::Float64MultiArray::ConstPtr &msg);
        void setGains(const std_msgs::Float64MultiArray::ConstPtr &msg);

    private:

        ros::Subscriber sub_gains_;
        ros::Subscriber sub_position_;
        ros::Subscriber sub_torque_;

        KDL::JntArrayVel dotq_msr_;
        KDL::JntArray q_msr_, q_des_;
        KDL::JntArray tau_des_, tau_cmd_, tau_gravity_;
        KDL::JntArray K_, D_;
        KDL::JntArray add_torque_;

        boost::scoped_ptr<KDL::ChainDynParam> id_solver_gravity_;

        KDL::JntArray initial_position_;
        bool isInitial;

    };

} // namespace

#endif
