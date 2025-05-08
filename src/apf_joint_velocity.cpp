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
#define DBGNPROF_USE_ROS
#define DBGNPROF_ENABLE_DEBUG
#endif // CONTROLLER_DEBUG

#ifdef CONTROLLER_PROFILE
#define DBGNPROF_USE_ROS
#define DBGNPROF_ENABLE_DEBUG
#define DBGNPROF_ENABLE_PROFILE
#endif // CONTROLLER_PROFILE

#include <debug_and_profile_helper/helper_macros.hpp>

namespace fliqc_controller_ros {

// Set the variables for this controller
static std::string controller_name = "APFJointVelocity";

bool APFJointVelocity::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) 
{
  // Get robot arm id, joint names and EE names parameters
  std::string arm_id;
  READ_PARAM(node_handle, controller_name, "arm_id", arm_id);

  std::vector<std::string> joint_names;
  READ_PARAM_VECTOR(node_handle, controller_name, "joint_names", joint_names);
  
  dim_q_ = joint_names.size();

  // Get the control interface of the robot joints
  auto velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  CHECK_NOT_NULLPTR(controller_name, velocity_joint_interface_);
  velocity_joint_handles_.resize(dim_q_);
  for (size_t i = 0; i < dim_q_; ++i) {
    CATCH_BLOCK(controller_name,
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    );
  }

  // Initialize the robot environment evaluator in robot_env_evaluator
  pinocchio::Model model;
  std::string ee_name_preset;
  std::vector<std::string> joint_names_preset;
  pinocchio::GeometryModel collision_model;
  std::string robot_preset;
  READ_PARAM(node_handle, controller_name, "robot_preset", robot_preset);
  auto preset = robot_env_evaluator::RobotPresetFactory::createRobotPreset(robot_preset);
  CHECK_NOT_NULLPTR(controller_name, preset);
  preset->getPresetRobot(model, ee_name_preset, joint_names_preset, collision_model);

  env_evaluator_ptr_ = std::make_unique<robot_env_evaluator::RobotEnvEvaluator>(model, ee_name_preset, joint_names_preset, collision_model);

  // subscribe to the planning scene information and wait for the first received message
  planning_scene_sub_ = node_handle.subscribe("/planning_scene", 1, &APFJointVelocity::planningSceneCallback, this);
  goal_pos_sub_ = node_handle.subscribe("/goal", 1, &APFJointVelocity::goalPosCallback, this);

  // Get controller parameters: fliqc_controller_ros parameters
  double diagnostic_period;
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_ros/position_convergence_threshold", position_convergence_threshold_);
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_ros/velocity_convergence_threshold", velocity_convergence_threshold_);
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_ros/diagnostic_period", diagnostic_period);
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_ros/robust_pinv_lambda", robust_pinv_lambda_);
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_ros/velocity_input_threshold", velocity_input_threshold_);
  
  // wait until first message received from necessary subscribers
  ros::Rate rate(10);
  while (ros::ok() && (!(first_receive_obstacle_ && first_receive_goal_))) {
      ros::spinOnce();
      ROS_INFO_STREAM_THROTTLE(1, controller_name << ": Waiting for messages... scene: [" << first_receive_obstacle_ << 
                                                                              "] goal: [" << first_receive_goal_ << "]");
      rate.sleep();
  }

  // Cold start the controller
  Eigen::VectorXd q = Eigen::VectorXd::Zero(dim_q_);
  for (size_t i = 0; i < dim_q_; ++i) {
    q(i) = velocity_joint_handles_[i].getPosition();
  }
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  env_evaluator_ptr_ -> forwardKinematics(q, T);
  Eigen::Vector3d now_ = T.block<3, 1>(0, 3);
  position_error_norm_ = (goal_pos_ - now_).norm();
  velocity_norm_ = 0.0;

  // Initialize diagnostic updater
  ros::NodeHandle ph("~");
  ph.setParam("diagnostic_period", diagnostic_period);
  diag_updater_ = std::make_unique<diagnostic_updater::Updater>(ros::NodeHandle(), ph, ros::this_node::getName());
  diag_updater_->setHardwareID(arm_id);
  diag_updater_->add("Position convergence", this, &APFJointVelocity::checkPositionConvergence);
  diag_updater_->add("Velocity convergence", this, &APFJointVelocity::checkVelocityConvergence);
  diag_updater_->update();

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

  if(first_receive_obstacle_ == false){ first_receive_obstacle_ = true; }
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
  
  if(first_receive_goal_ == false){ first_receive_goal_ = true; }
}

void APFJointVelocity::checkPositionConvergence(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if (position_error_norm_ < position_convergence_threshold_) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Position convergence");
  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Position not converged");
  }
  stat.add("Position error norm", position_error_norm_);
  stat.add("Position convergence threshold", position_convergence_threshold_);
}

void APFJointVelocity::checkVelocityConvergence(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if (velocity_norm_ < velocity_convergence_threshold_) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Velocity convergence");
  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Velocity not converged");
  }
  stat.add("Velocity norm", velocity_norm_);
  stat.add("Velocity convergence threshold", velocity_convergence_threshold_);
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
  DBGNPROF_START_CLOCK;
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
      double d_thresh = 0.12; 
      if(real_d < d_thresh && real_d>1e-3){
        double k_rep = 0.8;
        double rep_val = k_rep*(1.0/real_d - 1.0/d_thresh)/(real_d*real_d);
        repulsive += rep_val*(diff_ee/dist);
      }
    }
  }

  //---------------------Rotational Attractive Force--------------------------
  // Eigen::Matrix3d R_ee = T_ee.block<3,3>(0,0);
  // Eigen::Quaterniond q_curr(R_ee);

  // double p0 = q_curr.w();
  // Eigen::Vector3d pim = q_curr.vec();
  // double pg0 = goal_orientation_.w();
  // Eigen::Vector3d pgim = goal_orientation_.vec();

  // Eigen::Vector3d orientation_error = p0 * pgim - pg0 * pim - pim.cross(pgim);

  // double k_att_r = 2.0; // orientation position gain
  // double k_vel_r = 1.0; // orientation velocity gain
  // double w_max = 0.5;   // max angular velocity

  // Eigen::Vector3d w_d = (k_att_r / k_vel_r) * orientation_error;
  // double nu_r = std::min(1.0, w_max / (w_d.norm() + 1e-6)); 
  // Eigen::Vector3d f_vlcr = k_vel_r * nu_r * w_d; // rotational attraction
  // Eigen::VectorXd dq_ori = J_full_ee.block(3, 0, 3, dim_q_).transpose() * f_vlcr;

// ------------------------------------EE main task -----------------------------------
  Eigen::Vector3d combined_vel_ee = attract + repulsive;
  if (combined_vel_ee.norm() > velocity_input_threshold_){
    combined_vel_ee = combined_vel_ee.normalized() * velocity_input_threshold_;
  }
  //ROS_INFO_STREAM_THROTTLE(1, "Combined velocity: " << combined_vel_ee.transpose());

  Eigen::MatrixXd JJt_ee = J_ee * J_ee.transpose();
  Eigen::MatrixXd damped_identity_ee = robust_pinv_lambda_ * robust_pinv_lambda_ * Eigen::MatrixXd::Identity(JJt_ee.rows(), JJt_ee.cols());
  Eigen::VectorXd dq_main = J_ee.transpose() * (JJt_ee + damped_identity_ee).ldlt().solve(combined_vel_ee);
  //Eigen::VectorXd dq_main = J_ee.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(combined_vel_ee);

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
      double r = 0.2;
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
  Eigen::MatrixXd JJt_j3 = J_j3 * J_j3.transpose();
  Eigen::MatrixXd damped_identity_j3 = robust_pinv_lambda_ * robust_pinv_lambda_ * Eigen::MatrixXd::Identity(JJt_j3.rows(), JJt_j3.cols());
  Eigen::VectorXd dq_j3 = N_ee * J_j3.transpose() * (JJt_j3 + damped_identity_j3).ldlt().solve(F_rep_j3);
  // Eigen::MatrixXd J_j3_pinv = J_j3.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(3,3));
  // Eigen::VectorXd dq_j3 = N_ee * (J_j3_pinv * F_rep_j3);

  //STEP 4 - calculate the joint velocity command
  // --------------------------------main task + minor task -----------------------------------
  Eigen::VectorXd dq = dq_main + dq_j3; // + dq_ori; 
  DBGNPROF_STOP_CLOCK("runController");

  #if defined(CONTROLLER_DEBUG) || defined(CONTROLLER_PROFILE)
  DBGNPROF_LOG("q_dot_real", qdot_joint);
  DBGNPROF_LOG("q_dot_command", dq);
  #endif // CONTROLLER_DEBUG

  //STEP 5 - send the joint velocity command to the robot
  // Update diagnostics
  position_error_norm_ = diff.norm();
  velocity_norm_ = qdot_joint.norm();
  diag_updater_->update();

  for(int i=0; i<dim_q_; i++)
  {
    velocity_joint_handles_[i].setCommand(dq(i));
  }
  if (position_error_norm_ <= position_convergence_threshold_ && velocity_norm_ <= velocity_convergence_threshold_) {
    ROS_INFO_STREAM_THROTTLE(1, controller_name << ": Goal reached with tolerance [" << position_error_norm_ << 
                                                                 "], and velocity [" << velocity_norm_ << "]");
  }
  
}

void APFJointVelocity::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
  diag_updater_->force_update();
}

}  // namespace fliqc_controller_ros

PLUGINLIB_EXPORT_CLASS(fliqc_controller_ros::APFJointVelocity,
                       controller_interface::ControllerBase)
