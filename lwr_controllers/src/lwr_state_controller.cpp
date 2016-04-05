
#include <pluginlib/class_list_macros.h>
#include <algorithm>

#include <lwr_controllers/lwr_state_controller.h>

namespace lwr_controllers {

LWRStateController::LWRStateController() {}

LWRStateController::~LWRStateController() {}

bool LWRStateController::init(hardware_interface::JointStateInterface *robot, ros::NodeHandle &n)
{
    if (!ros::param::search(n.getNamespace(),"robot_name", robot_namespace_))
    {
        ROS_WARN_STREAM("LWRStateController: No robot name found on parameter server ("<<n.getNamespace()<<"/robot_name), using the namespace...");
        robot_namespace_ = n.getNamespace();
        //return false;
    }
    if (!n.getParam("robot_name", robot_namespace_))
    {
        ROS_WARN_STREAM("LWRStateController: Could not read robot name from parameter server ("<<n.getNamespace()<<"/robot_name), using the namespace...");
        robot_namespace_ = n.getNamespace();
        //return false;
    }

    joint_names_.push_back( robot_namespace_ + std::string("_0_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_1_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_2_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_3_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_4_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_5_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_6_joint") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_xx") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_yx") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_zx") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_pos_x") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_xy") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_yy") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_zy") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_pos_y") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_xz") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_yz") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_zz") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_pos_z") );
    cart_6_names_.push_back( robot_namespace_ + std::string("_X") );
    cart_6_names_.push_back( robot_namespace_ + std::string("_Y") );
    cart_6_names_.push_back( robot_namespace_ + std::string("_Z") );
    cart_6_names_.push_back( robot_namespace_ + std::string("_A") );
    cart_6_names_.push_back( robot_namespace_ + std::string("_B") );
    cart_6_names_.push_back( robot_namespace_ + std::string("_C") );

    // now get all handles: estExtTcpFT, estExtJntTrq,
    for(int c = 0; c < 18; ++c)
    {
        if(c < 12)
            cart_state_handles_.push_back(robot->getHandle(cart_12_names_.at(c)));
        if(c > 11 && c < 18)
            cart_state_handles_.push_back(robot->getHandle(cart_6_names_.at(c-12) + std::string("_estExtTcpFT")));
        /*
        if(c > 17 && c < 24)
            cart_state_handles_.push_back(robot->getHandle(cart_6_names_.at(c-18) + std::string("_damping")));
        if(c > 23 && c < 30)
            cart_state_handles_.push_back(robot->getHandle(cart_6_names_.at(c-24) + std::string("_wrench")));
        */
    }
    for(int j = 0; j < 7; ++j)
    {
        joint_state_handles_.push_back(robot->getHandle(joint_names_.at(j) + std::string("_estExtJntTrq")));
    }

    pub_msr_joint_state_ = n.advertise<sensor_msgs::JointState>(n.resolveName("msr_joint_state"),0);
    pub_msr_cart_wrench_ = n.advertise<geometry_msgs::WrenchStamped>(n.resolveName("msr_cart_wrench"),0);
    pub_msr_cart_pos_ = n.advertise<geometry_msgs::PoseStamped>(n.resolveName("msr_cart_pose"),0);
    //pub_msr_cart_pos_base_link_ = n.advertise<geometry_msgs::PoseStamped>(n.resolveName("msr_cart_pose_base_link"),0);

    // Initial position
    KDL::Rotation cur_R(cart_state_handles_.at(0).getPosition(),
                        cart_state_handles_.at(1).getPosition(),
                        cart_state_handles_.at(2).getPosition(),
                        cart_state_handles_.at(4).getPosition(),
                        cart_state_handles_.at(5).getPosition(),
                        cart_state_handles_.at(6).getPosition(),
                        cart_state_handles_.at(8).getPosition(),
                        cart_state_handles_.at(9).getPosition(),
                        cart_state_handles_.at(10).getPosition());
    KDL::Vector cur_p(cart_state_handles_.at(3).getPosition(),
                        cart_state_handles_.at(7).getPosition(),
                        cart_state_handles_.at(11).getPosition());
    KDL::Frame cur_T( cur_R, cur_p );
    x_msr_ = cur_T;

    // Initial force/torque measure

    estExtTcpFT_ = KDL::Wrench (KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));

    return true;
}

void LWRStateController::starting(const ros::Time& time)
{

}

void LWRStateController::update(const ros::Time& time, const ros::Duration& period)
{
    // get current values
    //std::cout << "Update current values" << std::endl;
    KDL::Rotation cur_R(cart_state_handles_.at(0).getPosition(),
                        cart_state_handles_.at(1).getPosition(),
                        cart_state_handles_.at(2).getPosition(),
                        cart_state_handles_.at(4).getPosition(),
                        cart_state_handles_.at(5).getPosition(),
                        cart_state_handles_.at(6).getPosition(),
                        cart_state_handles_.at(8).getPosition(),
                        cart_state_handles_.at(9).getPosition(),
                        cart_state_handles_.at(10).getPosition());
    KDL::Vector cur_p(cart_state_handles_.at(3).getPosition(),
                        cart_state_handles_.at(7).getPosition(),
                        cart_state_handles_.at(11).getPosition());
    KDL::Frame cur_T( cur_R, cur_p );
    x_msr_ = cur_T;

    // Prepare to publish everything...
    std::string frame_id = "lwr_right_base_link";

    // publish measured cartesian pose
    pose_msg_.header.stamp = time;
    pose_msg_.header.frame_id = frame_id;    // TODO generalize frame name to left/right robot
    tf::poseKDLToMsg(x_msr_, pose_msg_.pose);
    pub_msr_cart_pos_.publish(pose_msg_);

    // publish estimated external Wrench on TCP
    ext_wrench_msg_.header.stamp = time;
    ext_wrench_msg_.header.frame_id = frame_id;
    tf::wrenchKDLToMsg(estExtTcpFT_, ext_wrench_msg_.wrench);
    pub_msr_cart_wrench_.publish(ext_wrench_msg_);

    // publish joint states (position, velocity, estExtJntTrq=effort)
    for (int j=0; j<joint_state_handles_.size(); j++) {
        joint_state_msg_.position.at(j) = joint_state_handles_.at(j).getPosition();
        joint_state_msg_.velocity.at(j) = joint_state_handles_.at(j).getVelocity();
        joint_state_msg_.effort.at(j) = joint_state_handles_.at(j).getEffort();
    }
    pub_msr_joint_state_.publish(joint_state_msg_);
}

void LWRStateController::fromFRItoKDL(const std::vector<double>& in, KDL::Frame& out)
{
    KDL::Rotation R(in.at(0), in.at(1), in.at(2), in.at(4), in.at(5), in.at(6), in.at(8), in.at(9), in.at(10));
    KDL::Vector p(in.at(3), in.at(7), in.at(11));
    KDL::Frame T(R, p);
    out = T;
}

void LWRStateController::fromFRItoKDL(const std::vector<double>& in, KDL::Stiffness& out)
{
    KDL::Stiffness s(in.at(0), in.at(1), in.at(2), in.at(3), in.at(4), in.at(5));
    out = s;
}

void LWRStateController::fromFRItoKDL(const std::vector<double>& in, KDL::Wrench& out)
{
    KDL::Wrench w(KDL::Vector(in.at(0), in.at(1), in.at(2)), KDL::Vector(in.at(3), in.at(4), in.at(5)));
    out = w;
}


} // namespace

PLUGINLIB_EXPORT_CLASS( lwr_controllers::LWRStateController, controller_interface::ControllerBase)
