#include <math.h>
#include <pluginlib/class_list_macros.h>
#include <kdl_conversions/kdl_msg.h>

#include "lwr_controllers/itr_cartesian_impedance_controller.h"

// trying to smooth velocity
#include <control_toolbox/filters.h>

using namespace std;

namespace lwr_controllers 
{
    ITRCartesianImpedanceController::ITRCartesianImpedanceController() {}
    ITRCartesianImpedanceController::~ITRCartesianImpedanceController() {}

    bool ITRCartesianImpedanceController::init(hardware_interface::PositionCartesianInterface *robot, ros::NodeHandle &n)
    {
        ROS_INFO("THIS CONTROLLER USES THE TCP INFORMATION SET ON THE FRI SIDE... SO BE SURE YOU KNOW WHAT YOU ARE DOING!");

        if (!ros::param::search(n.getNamespace(),"robot_name", robot_namespace_))
        {
            ROS_WARN_STREAM("ITRCartesianImpedanceController: No robot name found on parameter server ("<<n.getNamespace()<<"/robot_name), using the namespace...");
            robot_namespace_ = n.getNamespace();
            //return false;
        }
        if (!n.getParam("robot_name", robot_namespace_))
        {
            ROS_WARN_STREAM("ITRCartesianImpedanceController: Could not read robot name from parameter server ("<<n.getNamespace()<<"/robot_name), using the namespace...");
            robot_namespace_ = n.getNamespace();
            //return false;
        }

        // stuff to read KDL chain in order to get the transform between base_link and robot
        if( !(KinematicChainControllerBase<hardware_interface::PositionCartesianInterface >::init(robot, n)) )
        {
            ROS_ERROR("Couldn't execute init of the KinematikChainController to get the KDL chain.");
            return false;
        }
        KDL::Chain mount_chain;
        //cout << "reading segment 0 name:" << endl;
        kdl_tree_.getChain(kdl_tree_.getRootSegment()->first, this->robot_namespace_ + "_base_link", mount_chain);
        //cout << "KDL segment 0 name: " << mount_chain.getSegment(0).getName() << endl;
        base_link2robot_ = mount_chain.getSegment(0).getFrameToTip();
        //cout << "base_link2robot transform: " << endl << base_link2robot_ << endl;

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

        // now get all handles, 12 for cart pos, 6 for stiff, 6 for damp, 6 for wrench, 6 for joints
        for(int c = 0; c < 30; ++c)
        {
            if(c < 12)
                cart_handles_.push_back(robot->getHandle(cart_12_names_.at(c)));
            if(c > 11 && c < 18)
                cart_handles_.push_back(robot->getHandle(cart_6_names_.at(c-12) + std::string("_stiffness")));
            if(c > 17 && c < 24)
                cart_handles_.push_back(robot->getHandle(cart_6_names_.at(c-18) + std::string("_damping")));
            if(c > 23 && c < 30)
                cart_handles_.push_back(robot->getHandle(cart_6_names_.at(c-24) + std::string("_wrench")));
        }
        for(int j = 0; j < 6; ++j)
        {
            joint_handles_.push_back(robot->getHandle(joint_names_.at(j)));
        }

        sub_pose_ = n.subscribe(n.resolveName("pose"), 1, &ITRCartesianImpedanceController::pose, this);
        sub_pose_world_ = n.subscribe(n.resolveName("pose_base_link"), 1, &ITRCartesianImpedanceController::pose_base_link, this);

        sub_gains_ = n.subscribe(n.resolveName("gains"), 1, &ITRCartesianImpedanceController::gains, this);
        sub_addFT_ = n.subscribe(n.resolveName("wrench"), 1, &ITRCartesianImpedanceController::additionalFT, this);

        srv_command_ = n.advertiseService("set_command", &ITRCartesianImpedanceController::command_cb, this);
        //sub_ft_measures_ = n.subscribe(n.resolveName("ft_measures"), 1, &ITRCartesianImpedanceController::updateFT, this);
        //pub_goal_ = n.advertise<geometry_msgs::PoseStamped>(n.resolveName("goal"),0);

        pub_msr_pos_ = n.advertise<geometry_msgs::PoseStamped>(n.resolveName("measured_cartesian_pose"),0);

        // Initial position
        KDL::Rotation cur_R(cart_handles_.at(0).getPosition(),
                            cart_handles_.at(1).getPosition(),
                            cart_handles_.at(2).getPosition(),
                            cart_handles_.at(4).getPosition(),
                            cart_handles_.at(5).getPosition(),
                            cart_handles_.at(6).getPosition(),
                            cart_handles_.at(8).getPosition(),
                            cart_handles_.at(9).getPosition(),
                            cart_handles_.at(10).getPosition());
        KDL::Vector cur_p(cart_handles_.at(3).getPosition(),
                            cart_handles_.at(7).getPosition(),
                            cart_handles_.at(11).getPosition());
        KDL::Frame cur_T( cur_R, cur_p );
        x_ref_ = cur_T;
        x_des_ = cur_T;
        x_set_ = cur_T;

        // Initial Cartesian stiffness
        KDL::Stiffness k( 800.0, 800.0, 800.0, 50.0, 50.0, 50.0 );
        k_des_ = k;

        // Initial force/torque measure
        KDL::Wrench w(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));
        f_des_ = w;

        // Initial Cartesian damping
        KDL::Stiffness d(0.7, 0.7, 0.7, 0.7, 0.7, 0.7);
        d_des_ = d;

        for (int i = 0; i < 3; ++i){
            if ( !nh_.getParam("position_stiffness_gains", k_des_[i] ) ){
                ROS_WARN("Position stiffness gain not set in yaml file, Using %f", k_des_[i]);
            }
        }
        for (int i = 3; i < 6; ++i){
            if ( !nh_.getParam("orientation_stiffness_gains", k_des_[i] ) ){
                ROS_WARN("Orientation stiffness gain not set in yaml file, Using %f", k_des_[i]);
            }
        }
        for (int i = 0; i < 6; ++i){
            if ( !nh_.getParam("damping_gains", d_des_[i]) ){
                ROS_WARN("Damping gain not set in yaml file, Using %f", d_des_[i]);
            }
        }

        max_trans_speed_ = 0.1; // m/s
        max_rot_speed_ = 0.5;   // rad/s

        // get params for speed limit
        if ( !nh_.getParam("max_trans_speed", max_trans_speed_) ){
            ROS_WARN("Max translation speed not set in yaml file, Using %f", max_trans_speed_);
        }
        if ( !nh_.getParam("max_rot_speed", max_rot_speed_) ){
            ROS_WARN("Max rotation speed not set in yaml file, Using %f", max_rot_speed_);
        }

        std::vector<double> cur_T_FRI;

        fromKDLtoFRI(x_des_, cur_T_FRI);

        // set initial commands already here:
        for(int c = 0; c < 30; ++c)
        {
            if(c < 12)
                cart_handles_.at(c).setCommand(cur_T_FRI.at(c));
            if(c > 11 && c < 18)
                cart_handles_.at(c).setCommand(k_des_[c-12]);
            if(c > 17 && c < 24)
                cart_handles_.at(c).setCommand(d_des_[c-18]);
            if(c > 23 && c < 30)
                cart_handles_.at(c).setCommand(f_des_[c-24]);
        }

        cmd_flag_ = false;

        return true;
    }

    void ITRCartesianImpedanceController::starting(const ros::Time& time)
    {
        KDL::Rotation cur_R(cart_handles_.at(0).getPosition(),
                            cart_handles_.at(1).getPosition(),
                            cart_handles_.at(2).getPosition(),
                            cart_handles_.at(4).getPosition(),
                            cart_handles_.at(5).getPosition(),
                            cart_handles_.at(6).getPosition(),
                            cart_handles_.at(8).getPosition(),
                            cart_handles_.at(9).getPosition(),
                            cart_handles_.at(10).getPosition());
        KDL::Vector cur_p(cart_handles_.at(3).getPosition(),
                            cart_handles_.at(7).getPosition(),
                            cart_handles_.at(11).getPosition());
        KDL::Frame cur_T( cur_R, cur_p );
        x_ref_ = cur_T;
        x_des_ = cur_T;
        x_prev_ = cur_T;

        x_prev_quat_ = Eigen::Quaterniond(Eigen::Matrix3d(x_prev_.M.data).transpose());
        x_prev_quat_.normalize();

        // Initial Cartesian stiffness
        KDL::Stiffness k( 800.0, 800.0, 800.0, 50.0, 50.0, 50.0 );
        k_des_ = k;

        // Initial force/torque measure
        KDL::Wrench w(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));
        f_des_ = w;

        std::vector<double> cur_T_FRI;

        fromKDLtoFRI(x_des_, cur_T_FRI);
        // forward commands to hwi
        for(int c = 0; c < 30; ++c)
        {
            if(c < 12)
                cart_handles_.at(c).setCommand(cur_T_FRI.at(c));
            if(c > 11 && c < 18)
                cart_handles_.at(c).setCommand(k_des_[c-12]);
            if(c > 17 && c < 24)
                cart_handles_.at(c).setCommand(d_des_[c-18]);
            if(c > 23 && c < 30)
                cart_handles_.at(c).setCommand(f_des_[c-24]);
        }

    }

    void ITRCartesianImpedanceController::pose(const geometry_msgs::PoseConstPtr &msg)
    {
        // Compute a KDL frame out of the message
        tf::poseMsgToKDL( *msg, x_des_ );

        cmd_flag_ = true;
    }

    void ITRCartesianImpedanceController::pose_base_link(const geometry_msgs::PoseConstPtr &msg)
    {
        // Compute a KDL frame out of the message
        tf::poseMsgToKDL( *msg, x_base_link_ );

        // Transformation from base_link KS into robot KS:
        // Translation world -> robot
        x_des_.p = x_base_link_.p - base_link2robot_.p;
        // Rotation from world -> robot
        x_des_.p = base_link2robot_.M.Inverse() * x_des_.p;
        // Orientation world -> robot
        x_des_.M = base_link2robot_.M.Inverse() * x_base_link_.M;

        cmd_flag_ = true;
    }

    bool ITRCartesianImpedanceController::command_cb(SetCartesianImpedanceCommand::Request &req, SetCartesianImpedanceCommand::Response &res)
    {
        return true;
    }

    void ITRCartesianImpedanceController::additionalFT(const geometry_msgs::WrenchStamped::ConstPtr &msg)
    {
        // Convert Wrench msg to KDL wrench
        geometry_msgs::Wrench f_meas = msg->wrench;
        tf::wrenchMsgToKDL( f_meas, f_cur_);
    }

    void ITRCartesianImpedanceController::gains(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
        if (msg.get()->data.size() == 12) {
            for (int i=0; i < 6; i++) {
                k_des_[i] = msg.get()->data.at(i);
                d_des_[i] = msg.get()->data.at(i + 6);
            }
        }
        else {
            ROS_ERROR("ITRCartesianImpedanceController::gains() received wrong message dimensions. Format is: [0 ... 5] stiffness, [6 ... 11] damping");
        }
    }

    void ITRCartesianImpedanceController::update(const ros::Time& time, const ros::Duration& period)
    {
        // get current values
        //std::cout << "Update current values" << std::endl;
        KDL::Rotation cur_R(cart_handles_.at(0).getPosition(),
                            cart_handles_.at(1).getPosition(),
                            cart_handles_.at(2).getPosition(),
                            cart_handles_.at(4).getPosition(),
                            cart_handles_.at(5).getPosition(),
                            cart_handles_.at(6).getPosition(),
                            cart_handles_.at(8).getPosition(),
                            cart_handles_.at(9).getPosition(),
                            cart_handles_.at(10).getPosition());
        KDL::Vector cur_p(cart_handles_.at(3).getPosition(),
                            cart_handles_.at(7).getPosition(),
                            cart_handles_.at(11).getPosition());
        KDL::Frame cur_T( cur_R, cur_p );
        x_cur_ = cur_T;

        std::vector<double> cur_T_FRI;


        // Cartesian speed limit for new positions:

        // create quaternions for current and desired orientation, normalize them to get correct angular distance later!
        x_cur_quat_ = Eigen::Matrix3d(x_cur_.M.data).transpose();
        x_cur_quat_.normalize();
        x_des_quat_ = Eigen::Matrix3d(x_des_.M.data).transpose();
        x_des_quat_.normalize();
        x_set_quat_ = x_des_quat_;

        // get linear desired velocity
        x_dot_.vel = (x_des_.p-x_set_.p) / period.toSec();

        // limit linear velocity to desired position
        double x_dot_vel_norm;
        x_dot_vel_norm = x_dot_.vel.Norm();
        if (x_dot_vel_norm > max_trans_speed_) {
            x_dot_.vel = x_dot_.vel / x_dot_vel_norm * max_trans_speed_;
        }       

        if (cmd_flag_) {

            // Interpolate position
            x_set_.p = x_prev_.p + (x_dot_.vel * period.toSec());

            // Interpolate orientation with SLERP
            double slerp_ratio = max_rot_speed_ / (x_des_quat_.angularDistance(x_prev_quat_)) * period.toSec();
            slerp_ratio = std::min(slerp_ratio, 1.0);
            x_set_quat_ = x_prev_quat_.slerp(slerp_ratio, x_des_quat_);

            // convert quaternion to rot matrix
            x_set_quat_.normalize(); // need to be normalized to get right rotation matrix
            x_set_.M = KDL::Rotation::Quaternion(x_set_quat_.x(), x_set_quat_.y(), x_set_quat_.z(), x_set_quat_.w());
        }

        fromKDLtoFRI(x_set_, cur_T_FRI);
        // forward commands to hwi
        for(int c = 0; c < 30; ++c)
        {
            if(c < 12)
                cart_handles_.at(c).setCommand(cur_T_FRI.at(c));
            if(c > 11 && c < 18)
                cart_handles_.at(c).setCommand(k_des_[c-12]);
            if(c > 17 && c < 24)
                cart_handles_.at(c).setCommand(d_des_[c-18]);
            if(c > 23 && c < 30)
                cart_handles_.at(c).setCommand(f_cur_[c-24]);
        }

        x_prev_ = x_set_;
        x_prev_quat_ = x_set_quat_;
    }

    void ITRCartesianImpedanceController::stopping(const ros::Time& time)
    {

    }

    void ITRCartesianImpedanceController::multiplyJacobian(const KDL::Jacobian& jac, const KDL::Wrench& src, KDL::JntArray& dest)
    {
        Eigen::Matrix<double,6,1> w;
        w(0) = src.force(0);
        w(1) = src.force(1);
        w(2) = src.force(2);
        w(3) = src.torque(0);
        w(4) = src.torque(1);
        w(5) = src.torque(2);

        Eigen::MatrixXd j(jac.rows(), jac.columns());
        j = jac.data;
        j.transposeInPlace();

        Eigen::VectorXd t(jac.columns());
        t = j*w;

        dest.resize(jac.columns());
        for (unsigned i=0; i<jac.columns(); i++)
        dest(i) = t(i);
    }

    void ITRCartesianImpedanceController::fromKDLtoFRI(const KDL::Frame& in, std::vector<double>& out)
    {
        out.resize(12);
        out.at(0) = in.M.UnitX().x();
        out.at(1) = in.M.UnitY().x();
        out.at(2) = in.M.UnitZ().x();
        out.at(3) = in.p.x();
        out.at(4) = in.M.UnitX().y();
        out.at(5) = in.M.UnitY().y();
        out.at(6) = in.M.UnitZ().y();
        out.at(7) = in.p.y();
        out.at(8) = in.M.UnitX().z();
        out.at(9) = in.M.UnitY().z();
        out.at(10) = in.M.UnitZ().z();
        out.at(11) = in.p.z();

    }

    void ITRCartesianImpedanceController::fromKDLtoFRI(const KDL::Stiffness& in, std::vector<double>& out)
    {
        out.resize(6);
        for( int d = 0; d < 6; ++d)
        {
            out.at(d) = in[d];
        }
    }

    void ITRCartesianImpedanceController::fromKDLtoFRI(const KDL::Wrench& in, std::vector<double>& out)
    {
        out.resize(6);
        out.at(0) = in.force.x();
        out.at(1) = in.force.y();
        out.at(2) = in.force.z();
        out.at(3) = in.torque.x();
        out.at(4) = in.torque.y();
        out.at(5) = in.torque.z();
    }

    void ITRCartesianImpedanceController::fromFRItoKDL(const std::vector<double>& in, KDL::Frame& out)
    {
        KDL::Rotation R(in.at(0), in.at(1), in.at(2), in.at(4), in.at(5), in.at(6), in.at(8), in.at(9), in.at(10));
        KDL::Vector p(in.at(3), in.at(7), in.at(11));
        KDL::Frame T(R, p);
        out = T;
    }

    void ITRCartesianImpedanceController::fromFRItoKDL(const std::vector<double>& in, KDL::Stiffness& out)
    {
        KDL::Stiffness s(in.at(0), in.at(1), in.at(2), in.at(3), in.at(4), in.at(5));
        out = s;
    }

    void ITRCartesianImpedanceController::fromFRItoKDL(const std::vector<double>& in, KDL::Wrench& out)
    {
        KDL::Wrench w(KDL::Vector(in.at(0), in.at(1), in.at(2)), KDL::Vector(in.at(3), in.at(4), in.at(5)));
        out = w;
    }

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::ITRCartesianImpedanceController, controller_interface::ControllerBase)
