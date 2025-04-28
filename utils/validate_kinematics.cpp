#include <pinocchio/algorithm/model.hpp>

#include <robot_env_evaluator/robot_env_evaluator.hpp>
#include <robot_env_evaluator/robot_env_evaluator_path.h>
#include <robot_env_evaluator/robot_presets.hpp>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>

// Function to publish a single TF frame
void publishTFFrame(const Eigen::Matrix4d& T, 
                    const std::string& parent_frame, 
                    const std::string& child_frame, 
                    tf2_ros::TransformBroadcaster& tf_broadcaster) {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    transform.transform.translation.x = T(0, 3);
    transform.transform.translation.y = T(1, 3);
    transform.transform.translation.z = T(2, 3);
    Eigen::Quaterniond quat(T.block<3, 3>(0, 0));
    transform.transform.rotation.x = quat.x();
    transform.transform.rotation.y = quat.y();
    transform.transform.rotation.z = quat.z();
    transform.transform.rotation.w = quat.w();
    tf_broadcaster.sendTransform(transform);
}

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "validate_kinematics");
    ros::NodeHandle nh;

    // Step 1: Get the robot preset and joint names
    pinocchio::Model model;
    pinocchio::GeometryModel collision_model;
    std::string ee_name;
    std::vector<std::string> joint_names;

    auto preset = robot_env_evaluator::RobotPresetFactory::createRobotPreset("panda");
    if (!preset) {
        ROS_ERROR("Failed to create robot preset for 'panda'.");
        return -1;
    }
    preset->getPresetRobot(model, ee_name, joint_names, collision_model);
    auto env_evaluator_ptr_ = std::make_unique<robot_env_evaluator::RobotEnvEvaluator>(model, ee_name, joint_names, collision_model);

    joint_names.push_back("panda_finger_joint1");
    joint_names.push_back("panda_finger_joint2");

    ROS_INFO("Robot preset loaded. End-effector: %s", ee_name.c_str());
    ROS_INFO("Joint names:");
    for (const auto& joint : joint_names) {
        ROS_INFO("  %s", joint.c_str());
    }

    // Step 2: Setup a node that publishes /joint_states for Franka joints
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/x_guide", 10); // Publisher for x_guide
    int rate = 100;
    ros::Rate loop_rate(rate); 

    Eigen::VectorXd q = Eigen::VectorXd::Zero(7); // 7-vector for joint positions
    q << 0, -0.785398, 0, -2.35619, 0, 1.5707, 0.785398;

    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.name = joint_names;
    joint_state_msg.position.resize(joint_names.size());
    joint_state_msg.position[7] = 0.03;
    joint_state_msg.position[8] = 0.03;

    tf2_ros::TransformBroadcaster tf_broadcaster;

    while (ros::ok()) {
        joint_state_msg.header.stamp = ros::Time::now();
        for (size_t i = 0; i < 7; ++i) {
            joint_state_msg.position[i] = q(i);
        }
        joint_state_pub.publish(joint_state_msg);

        // Check 1: The forward Kinematics 
        // Publish TF frames
        int EE_index = -1;
        Eigen::Matrix4d T;
        env_evaluator_ptr_->forwardKinematics(q, T, EE_index);
        publishTFFrame(T, "panda_link0", "EE", tf_broadcaster);

        // Publish global orientation frame
        Eigen::Matrix4d global_orientation_frame = Eigen::Matrix4d::Identity();
        global_orientation_frame.block<3, 1>(0, 3) = T.block<3, 1>(0, 3); // Extract position
        publishTFFrame(global_orientation_frame, "panda_link0", "global_orientation", tf_broadcaster);

        // Publish other frames from FK
        for (int i = 0; i <= 7; ++i) {
            Eigen::Matrix4d T_i;
            env_evaluator_ptr_->forwardKinematics(q, T_i, i);
            std::string frame_name = "frame_" + std::to_string(i);
            publishTFFrame(T_i, "panda_link0", frame_name, tf_broadcaster);
        }

        // Check 2: Check if Jacobian serves as as correct velocity mapping
        Eigen::MatrixXd J(6, 7);
        env_evaluator_ptr_->jacobian(q, J, EE_index);
        Eigen::MatrixXd Jpos = J.block<3, 7>(0, 0);
        Eigen::VectorXd v(6);
        
        // repeat with following values
        // 0 - 1 second, forward
        // 1 - 2 second, backward
        // 2 - 3 second, left
        // 3 - 4 second, right
        // 4 - 5 second, up
        // 5 - 6 second, down
        double time = ros::Time::now().toSec();
        int phase = static_cast<int>(time) % 6;

        switch (phase) {
            case 0: // Forward
            v << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0;
            break;
            case 1: // Backward
            v << -0.1, 0.0, 0.0, 0.0, 0.0, 0.0;
            break;
            case 2: // Left
            v << 0.0, 0.1, 0.0, 0.0, 0.0, 0.0;
            break;
            case 3: // Right
            v << 0.0, -0.1, 0.0, 0.0, 0.0, 0.0;
            break;
            case 4: // Up
            v << 0.0, 0.0, 0.1, 0.0, 0.0, 0.0;
            break;
            case 5: // Down
            v << 0.0, 0.0, -0.1, 0.0, 0.0, 0.0;
            break;
        }

        //Eigen::VectorXd q_dot = J.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(v);            // pinv with whole Jacobian
        Eigen::VectorXd q_dot = Jpos.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(v.head<3>()); // pinv with only position part of Jacobian
        Eigen::VectorXd q_rotate = Eigen::VectorXd::Zero(7);
        q_rotate(4) = 0.5; // Rotate around the z-axis
        q_dot = q_dot + (Eigen::MatrixXd::Identity(7, 7) - Jpos.completeOrthogonalDecomposition().pseudoInverse() * Jpos) * q_rotate;

        // Display the joint velocities
        Eigen::VectorXd x_guide = J * q_dot;

        // Publish x_guide as a TwistStamped message
        geometry_msgs::TwistStamped twist_msg;
        twist_msg.header.stamp = ros::Time::now();
        twist_msg.header.frame_id = "global_orientation"; // Use global orientation frame
        twist_msg.twist.linear.x = x_guide(0);
        twist_msg.twist.linear.y = x_guide(1);
        twist_msg.twist.linear.z = x_guide(2);
        twist_msg.twist.angular.x = x_guide(3);
        twist_msg.twist.angular.y = x_guide(4);
        twist_msg.twist.angular.z = x_guide(5);
        twist_pub.publish(twist_msg);

        // each 1s for J, q_dot, x_guide
        {
            static ros::Time last_time = ros::Time::now();
            if ((ros::Time::now() - last_time).toSec() > 1.0) {
                ROS_INFO_STREAM("Jacobian:\n" << J);
                ROS_INFO_STREAM("Joint velocities:\n" << q_dot.transpose());
                ROS_INFO_STREAM("x_guide:\n" << x_guide.transpose());
                last_time = ros::Time::now();
            }
        }

        // Update joint positions
        q += q_dot * (1.0 / rate); 

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
