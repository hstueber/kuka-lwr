#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <lwr_controllers/itr_joint_impedance_controller.h>

#define TEOUT(str) printf("TE: "); printf(str); printf("\n");
#include <iostream>

namespace lwr_controllers {

ITRJointImpedanceController::ITRJointImpedanceController()
{
    isInitial = true;
}

ITRJointImpedanceController::~ITRJointImpedanceController() {}

bool ITRJointImpedanceController::init(hardware_interface::ImpedanceJointInterface *robot, ros::NodeHandle &n)
{

  KinematicChainControllerBase<hardware_interface::ImpedanceJointInterface>::init(robot, n);

  K_.resize(kdl_chain_.getNrOfJoints());
  D_.resize(kdl_chain_.getNrOfJoints());

  ROS_DEBUG(" Number of joints in handle = %lu", joint_handles_.size() );

  for (int i = 0; i < joint_handles_.size(); ++i){
      if ( !nh_.getParam("stiffness_gains", K_(i) ) ){
          ROS_WARN("Stiffness gain not set in yaml file, Using %f", K_(i));
      }
  }
  for (int i = 0; i < joint_handles_.size(); ++i){
      if ( !nh_.getParam("damping_gains", D_(i)) ){
          ROS_WARN("Damping gain not set in yaml file, Using %f", D_(i));
      }
  }

  max_joint_speed_ = 0.5;   // rad/s
  // get param for speed limit
  if ( !nh_.getParam("max_joint_speed", max_joint_speed_) ){
      ROS_WARN("Max joint speed not set in yaml file, Using %f", max_joint_speed_);
  }

  dotq_msr_.resize(kdl_chain_.getNrOfJoints());
  q_msr_.resize(kdl_chain_.getNrOfJoints());
  q_des_.resize(kdl_chain_.getNrOfJoints());
  q_set_.resize(kdl_chain_.getNrOfJoints());
  q_des_vel_.resize(kdl_chain_.getNrOfJoints());
  add_torque_.resize(kdl_chain_.getNrOfJoints());

  sub_position_ = nh_.subscribe("position", 1, &ITRJointImpedanceController::setPosition, this);
  sub_gains_ = nh_.subscribe("gains", 1, &ITRJointImpedanceController::setGains, this);
  sub_torque_ = nh_.subscribe("torque", 1, &ITRJointImpedanceController::setAddTorque, this);

  return true;
}

void ITRJointImpedanceController::starting(const ros::Time& time)
{
  // get joint positions
  for(size_t i=0; i<joint_handles_.size(); i++) {
    K_(i) = 300;
    D_(i) = 0.7;
    add_torque_(i) = 0;
    dotq_msr_.q(i) = joint_handles_[i].getPosition();
    q_des_(i) = dotq_msr_.q(i);
    q_set_(i) = dotq_msr_.q(i);
    dotq_msr_.qdot(i) = joint_handles_[i].getVelocity();   
    }
}

void ITRJointImpedanceController::update(const ros::Time& time, const ros::Duration& period)
{
    if (isInitial) {
        q_des_ = dotq_msr_.q;
        isInitial = false;
    }

    for(int i=0; i<joint_handles_.size(); i++) {
        // get desired joint velocity
        q_des_vel_.qdot(i) = (q_des_(i) - q_set_(i)) / period.toSec();

        dotq_msr_.q(i) = joint_handles_[i].getPosition();
        q_msr_(i) = dotq_msr_.q(i);
        dotq_msr_.qdot(i) = joint_handles_[i].getVelocity();
    }

    // compute euklidean norm of joint velocity
    double q_des_vel_norm = 0;
    for (int i=0; i<joint_handles_.size(); i++) {
        q_des_vel_norm += q_des_vel_.qdot(i) * q_des_vel_.qdot(i);
    }
    q_des_vel_norm = sqrt(q_des_vel_norm);

    // limit joint velocity
    if (q_des_vel_norm > max_joint_speed_) {
        for (int i=0; i<joint_handles_.size(); i++) {
            q_des_vel_.qdot(i) = q_des_vel_.qdot(i) / q_des_vel_norm * max_joint_speed_;
        }
    }

    // integrate joint position from limited joint velocity
    for (int i=0; i<joint_handles_.size(); i++) {
        q_set_(i) += q_des_vel_.qdot(i) * period.toSec();
        // limit position saturation
        q_set_(i) = std::max(std::min(q_set_(i), joint_limits_.max(i)), joint_limits_.min(i));
    }

    // set command to FRI...
    for(size_t i=0; i<joint_handles_.size(); i++) {
        joint_handles_[i].setCommand(q_set_(i), add_torque_(i), K_(i), D_(i));
    }
}


void ITRJointImpedanceController::setPosition(const std_msgs::Float64MultiArray::ConstPtr &msg){
  if (msg->data.size() == 0) {
    ROS_INFO("Message Size = 0. Desired configuration must be: %lu dimension", joint_handles_.size());
    }
  else if ((int)msg->data.size() != joint_handles_.size()) {
    ROS_ERROR("Position message has the wrong size: %d", (int)msg->data.size());
    return;
    }
  else
  {
    for (unsigned int j = 0; j < joint_handles_.size(); ++j)
      q_des_(j) = msg->data[j];
  }

}

void ITRJointImpedanceController::setAddTorque(const std_msgs::Float64MultiArray::ConstPtr &msg){
  if (msg->data.size() == 0) {
    ROS_INFO("Desired configuration must be: %lu dimension", joint_handles_.size());
    }
  else if ((int)msg->data.size() != joint_handles_.size()) {
    ROS_ERROR("additional_torque message had the wrong size: %d", (int)msg->data.size());
    return;
    }
  else
  {
    for (unsigned int j = 0; j < joint_handles_.size(); ++j)
      add_torque_(j) = msg->data[j];
  }

}

void ITRJointImpedanceController::setGains(const std_msgs::Float64MultiArray::ConstPtr &msg){

  if (msg->data.size() == 2*joint_handles_.size()){
    for (unsigned int i = 0; i < joint_handles_.size(); ++i){
      K_(i) = msg->data[i];
      D_(i)= msg->data[i + joint_handles_.size()];
      }
    }
  else
  {
    ROS_ERROR("Wrong message dimensions. Num of Joint handles = %lu. Correct format: stiffness[0 ... 6]; damping[7 ... 13]", joint_handles_.size());
  }

  //ROS_INFO("Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());

  //ROS_INFO("New gains K: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf",
  //K_(0), K_(1), K_(2), K_(3), K_(4), K_(5), K_(6));
  //ROS_INFO("New gains D: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf",
  //D_(0), D_(1), D_(2), D_(3), D_(4), D_(5), D_(6));

  }


}  // namespace

PLUGINLIB_EXPORT_CLASS( lwr_controllers::ITRJointImpedanceController, controller_interface::ControllerBase)
