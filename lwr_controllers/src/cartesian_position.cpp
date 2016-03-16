
#include <lwr_controllers/cartesian_position.h>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>

#include <math.h>

namespace lwr_controllers 
{
CartesianPosition::CartesianPosition() {}
CartesianPosition::~CartesianPosition() {}

bool CartesianPosition::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
{
    if( !(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n)) )
    {
        ROS_ERROR("Couldn't initilize CartesianPosition controller.");
        return false;
    }

    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
    ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));

    q_cmd_.resize(kdl_chain_.getNrOfJoints());
    J_.resize(kdl_chain_.getNrOfJoints());

    // get joint positions
    joint_des_states_last_step.resize(kdl_chain_.getNrOfJoints());
    for(int i=0; i < joint_handles_.size(); i++)
    {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
        joint_des_states_.q(i) = joint_msr_states_.q(i);
        joint_des_states_last_step.q(i) = joint_msr_states_.q(i);
        joint_des_states_last_step.qdot(i) = 0.0;
    }

    // computing forward kinematics
    fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

    //Desired posture is the current one
    x_des_ = x_;
    v_des_ = KDL::Twist(KDL::Vector(0,0,0),KDL::Vector(0,0,0)) ;
    cmd_flag_ = 0;

    sub_command_ = nh_.subscribe("command", 1, &CartesianPosition::command, this);

    return true;
}

void CartesianPosition::starting(const ros::Time& time)
{

}

void CartesianPosition::update(const ros::Time& time, const ros::Duration& period)
{

    // get joint positions
    for(int i=0; i < joint_handles_.size(); i++)
    {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    }

    if (cmd_flag_)
    {
        // computing Jacobian
        jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);

        // computing J_pinv_
        pseudo_inverse(J_.data, J_pinv_);

        // computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

        // end-effector position error
        x_err_.vel = (x_des_.p - x_.p)*5.0 + v_des_.vel;

        // getting quaternion from rotation matrix
        x_.M.GetQuaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a);
        x_des_.M.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

        skew_symmetric(quat_des_.v, skew_);

        for (int i = 0; i < skew_.rows(); i++)
        {
            v_temp_(i) = 0.0;
            for (int k = 0; k < skew_.cols(); k++)
                v_temp_(i) += skew_(i,k)*(quat_curr_.v(k));
        }

        // end-effector orientation error
        x_err_.rot = (quat_curr_.a*quat_des_.v - quat_des_.a*quat_curr_.v - v_temp_)*5.0;

        // Nullspace movement
        Eigen::MatrixXd mat_temp = (MatrixXd::Identity(7,7) - (J_pinv_ * J_.data));
        KDL::JntArray q_dot_null;
        q_dot_null.resize(7);
        for (int i=0; i < mat_temp.rows(); i++)
        {
            q_dot_null(i) = 0;
            for (int k=0; k < mat_temp.cols(); k++)
            {
                q_dot_null(i) += mat_temp(i,k) * -0.001 * joint_msr_states_.q(i);
            }
        }
        // computing q_dot
        for (int i = 0; i < J_pinv_.rows(); i++)
        {
            joint_des_states_.qdot(i) = q_dot_null(i);
            for (int k = 0; k < J_pinv_.cols(); k++)
                joint_des_states_.qdot(i) += J_pinv_(i,k)*x_err_(k); // J_pinv*x_err + (I - J_pinv * J) * const * q_msr

        }
        for (int i = 0; i < joint_handles_.size(); i++)
        {
            double qddot = (joint_des_states_.qdot(i) - joint_des_states_last_step.qdot(i))/period.toSec();
            //ROS_INFO_STREAM("joint_" << i << ": " << qddot);
            qddot = std::min(std::max(qddot,-1.5),1.5); // limit acceleration
            joint_des_states_.qdot(i) = joint_des_states_last_step.qdot(i) + qddot*period.toSec();
            joint_des_states_.qdot(i) = std::min(std::max(joint_des_states_.qdot(i),-0.5),0.5); // limit velocity

            // integrating q_dot -> getting q (Euler method)

            joint_des_states_.q(i) += period.toSec()*joint_des_states_.qdot(i);
            //joint_des_states_.q(i) = (joint_des_states_.qdot(i) + joint_msr_states_.q(i))*0.1 + joint_des_states_.q(i) * 0.9;
            //ROS_INFO_STREAM("joint_" << i << ": " << joint_msr_states_.q(i) << "\t desired: " << joint_des_states_.q(i) << "\t period " << period.toSec());


            // joint limits saturation
            
            if (joint_des_states_.q(i) < joint_limits_.min(i))
                joint_des_states_.q(i) = joint_limits_.min(i);
            if (joint_des_states_.q(i) > joint_limits_.max(i))
                joint_des_states_.q(i) = joint_limits_.max(i);
        }

        /*if (Equal(x_, x_des_, 0.005))
            {
                ROS_INFO("On target");
                cmd_flag_ = 0;
            }*/
    }


    // set controls for joints
    for (int i = 0; i < joint_handles_.size(); i++)
    {
        joint_handles_[i].setCommand(joint_des_states_.q(i));
        joint_des_states_last_step.qdot(i) = /*joint_des_states_last_step.qdot(i)*0.9 + 0.1* */(joint_des_states_.q(i) - joint_des_states_last_step.q(i))/period.toSec();
        //ROS_INFO_STREAM("joint_" << i << ": " << joint_des_states_.q(i) << "\t" << joint_des_states_last_step.q(i) << "\t" << joint_des_states_last_step.qdot(i));
        joint_des_states_last_step.q(i) = joint_des_states_.q(i);
    }
}

void CartesianPosition::command(const lwr_controllers::PoseRPY::ConstPtr &msg)
{
    KDL::Frame frame_des_;
    double dt = (ros::Time::now() - cmd_stamp_).toSec();
    switch(msg->id)
    {
    case 0:
        frame_des_ = KDL::Frame(
                    KDL::Rotation::RPY(msg->orientation.roll,
                                       msg->orientation.pitch,
                                       msg->orientation.yaw),
                    KDL::Vector(msg->position.x,
                                msg->position.y,
                                msg->position.z));
        if(dt>0.0 && !Equal(frame_des_.p,x_des_.p))
        {
            cmd_stamp_ = ros::Time::now();
            v_des_.vel = (frame_des_.p - x_des_.p)/dt;
            //ROS_INFO("Vel: %f %f %f", v_des_.vel.x(), v_des_.vel.y(),v_des_.vel.z());
        }
        break;

    case 1: // position only
        frame_des_ = KDL::Frame(
                    KDL::Vector(msg->position.x,
                                msg->position.y,
                                msg->position.z));
        break;

    case 2: // orientation only
        frame_des_ = KDL::Frame(
                    KDL::Rotation::RPY(msg->orientation.roll,
                                       msg->orientation.pitch,
                                       msg->orientation.yaw));
        break;

    default:
        ROS_INFO("Wrong message ID");
        return;
    }
    x_des_ = frame_des_;
    cmd_flag_ = 1;
}
}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::CartesianPosition, controller_interface::ControllerBase)
