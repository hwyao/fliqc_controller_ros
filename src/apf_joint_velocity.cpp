#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/model.hpp>

#include "fliqc_controller_ros/apf_joint_velocity.hpp"
#include "fliqc_controller_ros/helpers.hpp"
#include <robot_env_evaluator/robot_env_evaluator_path.h>
#include <robot_env_evaluator/robot_presets.hpp>

#include <cmath>
#include <array>
#include <mutex>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>

#ifdef CONTROLLER_DEBUG
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#endif // CONTROLLER_DEBUG

namespace fliqc_controller_ros {

bool APFJointVelocity::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) 
{
  // Set the variables for this controller
  std::string controller_name = "APFJointVelocity";
                                            
  // Get robot arm id, joint names and EE names parameters
  std::string arm_id;
  READ_PARAM(node_handle, controller_name, "arm_id", arm_id);

  std::vector<std::string> joint_names;
  READ_PARAM_SILENT(node_handle, controller_name, "joint_names", joint_names);
  
  dim_q_ = joint_names.size();

  // Get the control interface of the robot joints
  auto velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();

  CHECK_NOT_EMPTY(controller_name, velocity_joint_interface_ == nullptr);

  velocity_joint_handles_.resize(dim_q_);

  for (size_t i = 0; i < dim_q_; ++i) 
  {
    CATCH_BLOCK(controller_name,velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    );
  }

  // Initialize the robot environment evaluator in robot_env_evaluator
  pinocchio::Model model;
  std::string ee_name_preset;
  std::vector<std::string> joint_names_preset;
  
  pinocchio::GeometryModel collision_model;
  auto preset = robot_env_evaluator::RobotPresetFactory::createRobotPreset("panda");
  CHECK_NOT_EMPTY(controller_name, preset == nullptr);
  preset->getPresetRobot(model, ee_name_preset, joint_names_preset, collision_model);

  env_evaluator_ptr_ = std::make_unique<robot_env_evaluator::RobotEnvEvaluator>(model, ee_name_preset, joint_names_preset, collision_model);

  // subscribe to the targeted velocity and goal to distance from the multi-agent system
  targeted_velocity_sub_ = node_handle.subscribe("/agent_twist_global", 1, &APFJointVelocity::targetedVelocityCallback, this);
  dist_to_goal_sub_ = node_handle.subscribe("/distance_to_goal", 1, &APFJointVelocity::distanceToGoalCallback, this);
  goal_pos_sub_ = node_handle.subscribe("/goal", 1, &APFJointVelocity::goalPosCallback, this);
  // subscribe to the planning scene information and wait for the first received message
  planning_scene_sub_ = node_handle.subscribe("/planning_scene", 1, &APFJointVelocity::planningSceneCallback, this);
  
  // wait until first message received from all subscribers
  ros::Rate rate(10);
  while (ros::ok() && (obstacles_.empty() || targeted_velocity_ == Eigen::Vector3d(-100,-100,-100) || distance_to_goal_ == -100)) {
      ros::spinOnce();
      ROS_INFO_STREAM_THROTTLE(1, controller_name << ": Waiting for first of all messages...");
      rate.sleep();
  }

  ROS_INFO_STREAM(controller_name << "is initialized with " << dim_q_ << " joints.");

  return true;
}

void APFJointVelocity::starting(const ros::Time& /* time */) 
{
  elapsed_time_ = ros::Duration(0.0);
  ROS_INFO_STREAM("Starting APFJointVelocity controller.");
}

void APFJointVelocity::planningSceneCallback(const moveit_msgs::PlanningScene::ConstPtr& msg) 
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  obstacles_.clear();
  for (const auto& obj : msg->world.collision_objects) {
      if (obj.primitives.empty()){
        ROS_WARN_STREAM("The object " << obj.id << " in PlanningScene has no primitive.");
        continue;
      }
      
      const auto& pose = obj.primitive_poses[0];
      Eigen::Matrix4d pose_mat = Eigen::Matrix4d::Identity();
      pose_mat.block<3, 1>(0, 3) = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
      pose_mat.block<3, 3>(0, 0) = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).toRotationMatrix();

      if (obj.primitives[0].type == shape_msgs::SolidPrimitive::SPHERE){
        obstacles_.push_back({coal::Sphere(obj.primitives[0].dimensions[0]), pose_mat});
      }
      else{
        ROS_ERROR_STREAM("The obstacle type " << obj.primitives[0].type << "of object " << obj.id << " is not supported.");
      }
  }
}

void APFJointVelocity::goalPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // Store the goal position
    goal_pos_ = Eigen::Vector3d(msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z);

  // Store the goal orientation
  goal_orientation_ = Eigen::Quaterniond(
      msg->pose.orientation.w,
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z);

  // Log the goal position and orientation
  ROS_INFO_STREAM("Goal Position: " << goal_pos_.transpose());
  ROS_INFO_STREAM("Goal Orientation: " << goal_orientation_.coeffs().transpose());
}

void APFJointVelocity::targetedVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) 
{
  targeted_velocity_ = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
}

void APFJointVelocity::distanceToGoalCallback(const std_msgs::Float64::ConstPtr& msg) {
  distance_to_goal_ = msg->data;
}

void APFJointVelocity::update(const ros::Time& /* time */,const ros::Duration& period) 
{
  elapsed_time_ += period;

  //STEP 0 - get current joint position and velocity

  Eigen::VectorXd q(dim_q_);
  Eigen::VectorXd qdot_joint(dim_q_);

  for (size_t i = 0; i < dim_q_; i++) 
  {
    q(i) = velocity_joint_handles_[i].getPosition();
    qdot_joint(i) = velocity_joint_handles_[i].getVelocity();
  }

  //STEP 1 - use env_evaluator to calculate FK and jacobian 
  Eigen::Matrix4d T_ee = Eigen::Matrix4d::Identity();
  Eigen::MatrixXd J_full_ee(6, dim_q_);
  env_evaluator_ptr_->forwardKinematics(q, T_ee); // default argument is -1 (EE)
  env_evaluator_ptr_->jacobian(q, J_full_ee);

  Eigen::MatrixXd J_ee = J_full_ee.block(0, 0, 3, dim_q_);

  //STEP 2 - calculate EE position and attractive force (translational attract)
  //the one that from the papar 
  // ---------------------Translational Attractive Force--------------------------
  Eigen::Vector3d ee_pos = T_ee.block<3,1>(0,3);
  Eigen::Vector3d diff = goal_pos_ - ee_pos;

  double kp = 2.0;
  double kv = 2.0;
  double Vmax = 0.1; 
  Eigen::Vector3d xdot_d = (kp / kv) * diff;
  Eigen::Vector3d xdot = J_ee * qdot_joint;

  double nu = std ::min(1.0, Vmax / (std::sqrt(xdot_d.transpose() * xdot_d) + 1e-6));
  Eigen::Vector3d attract = -kv * (xdot - nu * xdot_d);
  // simple realization of attraction force
  // double k_att = 2.0; 

  // Eigen::Vector3d attract = k_att * diff; 

  // double max_speed = 0.1;
  // if(attract.norm() > max_speed){
  //   attract = attract.normalized() * max_speed;
  // }


  //STEP 3 - calculate the repulsive force from the obstacles
  //--------------------------------Repulsive Force--------------------------
  Eigen::Vector3d repulsive(0,0,0);
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    for(const auto& obs : obstacles_)
    {
      
      Eigen::Vector3d obs_center = obs.obstacle_pose.block<3,1>(0,3);
      double r = 0.05;
      Eigen::Vector3d diff_ee = ee_pos - obs_center;
      double dist = diff_ee.norm();
      double real_d = dist - r;
      double d_thresh = 0.2; 
      if(real_d < d_thresh && real_d>1e-5){
        double k_rep = 1.0;
        double rep_val = k_rep*(1.0/real_d - 1.0/d_thresh)/(real_d*real_d);
        repulsive += rep_val*(diff_ee/dist);
      }
    }
  }

  //---------------------Rotational Attractive Force--------------------------
  Eigen::Matrix3d R_ee = T_ee.block<3,3>(0,0);
  Eigen::Quaterniond q_curr(R_ee);
  // To do : get it from topic
  //Eigen::Quaterniond q_goal(0.96593, 0.25882, 0.0, 0.0);
  //Eigen::Quaterniond q_goal(1, 0.0, 0.0, 0.0);

  double p0 = q_curr.w();
  Eigen::Vector3d pim = q_curr.vec();
  double pg0 = goal_orientation_.w();
  Eigen::Vector3d pgim = goal_orientation_.vec();

  Eigen::Vector3d orientation_error = p0 * pgim - pg0 * pim - pim.cross(pgim);

  double k_att_r = 2.0; // orientation position gain
  double k_vel_r = 1.0; // orientation velocity gain
  double w_max = 0.5;   // max angular velocity

  Eigen::Vector3d w_d = (k_att_r / k_vel_r) * orientation_error;
  double nu_r = std::min(1.0, w_max / (w_d.norm() + 1e-6)); 
  Eigen::Vector3d f_vlcr = k_vel_r * nu_r * w_d; // rotational attraction
  Eigen::VectorXd dq_ori = J_full_ee.block(3, 0, 3, dim_q_).transpose() * f_vlcr;


  Eigen::Vector3d combined_vel_ee = attract + repulsive;

// ------------------------------------EE main task -----------------------------------
  Eigen::VectorXd dq_main = J_ee.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(combined_vel_ee);

  Eigen::MatrixXd J_ee_pinv = J_ee.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(3,3));
  Eigen::MatrixXd N_ee = Eigen::MatrixXd::Identity(dim_q_, dim_q_) - J_ee_pinv * J_ee;

 // --------------------------------Joints minor Task ----------------------------------- 

 //  Control site -> joint 3 
  Eigen::Matrix4d T_j3 = Eigen::Matrix4d::Identity();
  Eigen::MatrixXd J_full_j3(6, dim_q_);
  env_evaluator_ptr_->forwardKinematics(q, T_j3, 3);
  env_evaluator_ptr_->jacobian(q, J_full_j3, 3);

  Eigen::MatrixXd J_j3 = J_full_j3.block(0,0, 3, dim_q_);
  Eigen::Vector3d j3_pos = T_j3.block<3,1>(0,3);


  // Joint 3 repulsive force from the obstacles

  Eigen::Vector3d F_rep_j3(0,0,0);
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    for(const auto& obs : obstacles_)
    {
      Eigen::Vector3d obs_center = obs.obstacle_pose.block<3,1>(0,3);
      //double r = obs.obstacle.radius;
      double r = 0.05;
      Eigen::Vector3d diff_j3 = j3_pos - obs_center;
      double dist = diff_j3.norm();
      double real_d = dist - r;
      double d_thresh = 0.2; 

      if(real_d < d_thresh && real_d>1e-5)
      {
        double k_rep = 1.0; 
        double rep_val = k_rep * (1.0/real_d - 1.0/d_thresh)/(real_d*real_d);
        F_rep_j3 += rep_val*(diff_j3/dist);
      }
    }
  }


  // Joint 3 pinv
  Eigen::MatrixXd J_j3_pinv = J_j3.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(3,3));

  Eigen::VectorXd dq_j3 = N_ee * (J_j3_pinv * F_rep_j3);

  //STEP 4 - calculate the joint velocity command
  // --------------------------------main task + minor task -----------------------------------
  Eigen::VectorXd dq = dq_main + dq_j3 + dq_ori;

  if(distance_to_goal_ > 0.01)
  {
    for(int i=0; i<dim_q_; i++)
    {
      velocity_joint_handles_[i].setCommand(dq(i));
    }
  } else 
  {
    // when reached 
    for(int i=0; i<dim_q_; i++){
      velocity_joint_handles_[i].setCommand(0.0);
    }
  }
  
}

void APFJointVelocity::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace fliqc_controller_ros

PLUGINLIB_EXPORT_CLASS(fliqc_controller_ros::APFJointVelocity,
                       controller_interface::ControllerBase)
