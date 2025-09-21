/**
 * @file fliqc_joint_velocity_standard.cpp
 * @brief Implementation file for the FLIQCJointVelocityStandard controller class.
 * 
 * Copyright (c) 2025, Haowen Yao
 * Use of this source code is governed by the MIT license, see LICENSE.
 */
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/model.hpp>

#include "fliqc_controller_ros/fliqc_joint_velocity_standard.hpp"
#include "fliqc_controller_ros/helpers.hpp"
#include <robot_env_evaluator/robot_env_evaluator_path.h>
#include <robot_env_evaluator/robot_presets.hpp>

#include <cmath>
#include <array>
#include <mutex>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>
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

// Redirect profiling macros to wrapper names to support both modes like no_env_node
#define PROFILE_DBGNPROF_START_CLOCK
#define PROFILE_DBGNPROF_STOP_CLOCK
#define REALTIME_DBGNPROF_START_CLOCK
#define REALTIME_DBGNPROF_STOP_CLOCK

#ifdef CONTROLLER_PROFILE
#define DBGNPROF_USE_ROS
#define DBGNPROF_ENABLE_PROFILE
#undef PROFILE_DBGNPROF_START_CLOCK
#undef PROFILE_DBGNPROF_STOP_CLOCK
#define PROFILE_DBGNPROF_START_CLOCK DBGNPROF_START_CLOCK
#define PROFILE_DBGNPROF_STOP_CLOCK DBGNPROF_STOP_CLOCK
#endif // CONTROLLER_PROFILE

#ifdef CONTROLLER_REALTIME_PROFILE
#define DBGNPROF_USE_ROS
#define DBGNPROF_ENABLE_DEBUG
#define DBGNPROF_ENABLE_PROFILE
#undef REALTIME_DBGNPROF_START_CLOCK
#undef REALTIME_DBGNPROF_STOP_CLOCK
#define REALTIME_DBGNPROF_START_CLOCK DBGNPROF_START_CLOCK
#define REALTIME_DBGNPROF_STOP_CLOCK DBGNPROF_STOP_CLOCK
#endif // CONTROLLER_REALTIME_PROFILE

#include <debug_and_profile_helper/helper_macros.hpp>

namespace fliqc_controller_ros {

// Set the variables for this controller
static std::string controller_name = "FLIQCJointVelocityStandard";

bool FLIQCJointVelocityStandard::init(hardware_interface::RobotHW* robot_hardware,
                                                   ros::NodeHandle& node_handle) {
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

  // Initialize the FLIQC controller in FLIQC_controller_core
  controller_ptr_ = std::make_unique<FLIQC_controller_core::FLIQC_controller_joint_velocity_basic>(dim_q_);
  
  // Get controller parameters: lcqpow parameters
  READ_PARAM(node_handle, controller_name, 
      "/lcqpow/complementarityTolerance", controller_ptr_->lcqp_solver.complementarityTolerance);
  READ_PARAM(node_handle, controller_name,
      "/lcqpow/stationarityTolerance", controller_ptr_->lcqp_solver.stationarityTolerance);
  READ_PARAM(node_handle, controller_name,
      "/lcqpow/initialPenaltyParameter", controller_ptr_->lcqp_solver.initialPenaltyParameter);
  READ_PARAM(node_handle, controller_name,
      "/lcqpow/penaltyUpdateFactor", controller_ptr_->lcqp_solver.penaltyUpdateFactor);
  controller_ptr_->lcqp_solver.updateOptions();

  // Get controller parameters: fliqc_controller_core parameters
  READ_PARAM_ENUM(node_handle, controller_name,
    "/fliqc_controller_core/quad_cost_type", controller_ptr_->quad_cost_type, FLIQC_controller_core::FLIQC_quad_cost_type);
  READ_PARAM_ENUM(node_handle, controller_name,
    "/fliqc_controller_core/linear_cost_type", controller_ptr_->linear_cost_type, FLIQC_controller_core::FLIQC_linear_cost_type);
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_core/lambda_cost_penalty", controller_ptr_->lambda_cost_penalty);
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_core/enable_lambda_constraint_in_L", controller_ptr_->enable_lambda_constraint_in_L);
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_core/enable_lambda_constraint_in_x", controller_ptr_->enable_lambda_constraint_in_x);
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_core/lambda_max", controller_ptr_->lambda_max);
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_core/enable_esc_vel_constraint", controller_ptr_->enable_esc_vel_constraint);
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_core/esc_vel_max", controller_ptr_->esc_vel_max);
  READ_PARAM(node_handle, controller_name,
    "/fliqc_controller_core/enable_nullspace_projector_in_A", controller_ptr_->enable_nullspace_projector_in_A);
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_core/dt", controller_ptr_->dt);
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_core/eps", controller_ptr_->eps);
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_core/active_threshold", controller_ptr_->active_threshold);
  READ_PARAM_EIGEN(node_handle, controller_name,
      "/fliqc_controller_core/q_dot_max", controller_ptr_->q_dot_max, dim_q_);
  READ_PARAM_EIGEN(node_handle, controller_name,
      "/fliqc_controller_core/weight_on_mass_matrix", controller_ptr_->weight_on_mass_matrix, dim_q_);

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

  // Get controller parameters: realtime thread pool (optional)
  READ_PARAM(node_handle, controller_name,
    "/fliqc_controller_ros/enable_realtime_thread_pool", enable_realtime_thread_pool_);
  READ_PARAM(node_handle, controller_name,
    "/fliqc_controller_ros/realtime_thread_pool_config/max_threads", realtime_thread_pool_size_);
  READ_PARAM(node_handle, controller_name,
    "/fliqc_controller_ros/realtime_thread_pool_config/thread_rt_priority", realtime_thread_priority_);
  READ_PARAM(node_handle, controller_name,
    "/fliqc_controller_ros/realtime_thread_pool_config/main_rt_priority", realtime_main_priority_);
  READ_PARAM_VECTOR(node_handle, controller_name,
    "/fliqc_controller_ros/realtime_thread_pool_config/thread_affinity", realtime_thread_affinity_);
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_ros/realtime_thread_pool_config/main_affinity", realtime_main_affinity_);
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_ros/realtime_thread_pool_config/require_thread_rt_priority", require_thread_rt_priority_);
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_ros/realtime_thread_pool_config/require_main_rt_priority", require_main_rt_priority_);
  READ_PARAM(node_handle, controller_name,
    "/fliqc_controller_ros/realtime_thread_pool_wait_us", realtime_thread_pool_wait_us_);

  // Get controller parameters: fliqc_controller_joint_velocity_standard parameters
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_joint_velocity_standard/switch_to_disable_multi_agent", switch_to_disable_multi_agent_);

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
  READ_PARAM(node_handle, controller_name,
    "/robot_env_evaluator/calculate_self_collision", env_evaluator_ptr_->calculate_self_collision_);
  READ_PARAM(node_handle, controller_name,
    "/robot_env_evaluator/projector_dist_to_control_enable", env_evaluator_ptr_->projector_dist_to_control_enable_);
  READ_PARAM(node_handle, controller_name,
    "/robot_env_evaluator/projector_dist_to_control_with_zero_orientation", env_evaluator_ptr_->projector_dist_to_control_with_zero_orientation_);
  READ_PARAM(node_handle, controller_name,
    "/robot_env_evaluator/robust_pinv_lambda", env_evaluator_ptr_->robust_pinv_lambda_);
  READ_PARAM(node_handle, controller_name,
    "/robot_env_evaluator/enable_broad_phase_search", env_evaluator_ptr_->broad_phase_search_enable_);
  READ_PARAM(node_handle, controller_name,
    "/robot_env_evaluator/broad_phase_collision_padding", env_evaluator_ptr_->broad_phase_collision_padding_);
  if (preset->isFrankaRobot()){
    franka_hw::FrankaModelInterface* model_interface_ = robot_hardware->get<franka_hw::FrankaModelInterface>();
    CHECK_NOT_NULLPTR(controller_name, model_interface_);
    CATCH_BLOCK(controller_name,
      mass_matrix_bridge_ = std::make_unique<FrankaModelInterfaceBridge>(model_interface_, arm_id);
    )
  }

  // subscribe to the targeted velocity from the multi-agent system
  targeted_velocity_sub_ = node_handle.subscribe("/agent_twist_global", 1, &FLIQCJointVelocityStandard::targetedVelocityCallback, this);

  // subscribe to the planning scene and environment information
  planning_scene_sub_ = node_handle.subscribe("/planning_scene", 1, &FLIQCJointVelocityStandard::planningSceneCallback, this, ros::TransportHints().unreliable());
  goal_sub_ = node_handle.subscribe("/goal", 1, &FLIQCJointVelocityStandard::goalPoseCallback, this, ros::TransportHints().unreliable());

  // clear and warm up the data until the controller started
  ROS_INFO_STREAM(controller_name << ": Controller starting...");
  error_flag_ = false;
  targeted_velocity_ = Eigen::Vector3d::Zero();
  first_receive_obstacle_ = false;
  goal_position_ = Eigen::Vector3d::Zero();
  first_receive_goal_ = false;
  obstacles_.clear();

  // wait until first message received from necessary subscribers
  ros::Rate rate(10);
  while (ros::ok() && !(first_receive_obstacle_ && first_receive_goal_)) {
      ROS_INFO_STREAM_THROTTLE(1, controller_name << ": Waiting for messages... scene: [" << first_receive_obstacle_ << 
                                                                              "] goal: [" << first_receive_goal_ << "]");
      ros::spinOnce();
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
  position_error_norm_ = (goal_position_ - now_).norm();
  velocity_norm_ = 0.0;
  
  // Initialize diagnostic updater
  ros::NodeHandle ph("~");
  ph.setParam("diagnostic_period", diagnostic_period);
  diag_updater_ = std::make_unique<diagnostic_updater::Updater>(ros::NodeHandle(), ph, ros::this_node::getName());
  diag_updater_->setHardwareID(arm_id);
  diag_updater_->add("Controller state", this, &FLIQCJointVelocityStandard::checkControllerState);
  diag_updater_->add("Position convergence", this, &FLIQCJointVelocityStandard::checkPositionConvergence);
  diag_updater_->add("Velocity convergence", this, &FLIQCJointVelocityStandard::checkVelocityConvergence);
  diag_updater_->force_update();
  diag_updater_->update();

  double period = diag_updater_->getPeriod();
  ROS_INFO_STREAM(controller_name << ": Diagnostic name: " << ros::this_node::getName());
  ROS_INFO_STREAM(controller_name << ": Diagnostic period: " << period);

  ROS_INFO_STREAM(controller_name << ": Controller started.");

  // Initialize realtime compute thread pool if enabled
  if (enable_realtime_thread_pool_) {
    try {
      realtime_calc_thread_pool::Config realtime_config(
        realtime_thread_pool_size_,
        realtime_thread_priority_,
        realtime_main_priority_,
        realtime_thread_affinity_,
        realtime_main_affinity_,
        require_thread_rt_priority_,
        require_main_rt_priority_
      );
      thread_pool_ptr_ = std::make_unique<realtime_calc_thread_pool::RealtimeThreadPool<StandardControllerComputationResult>>(realtime_config);
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM(controller_name << ": Failed to create realtime thread pool: " << e.what());
      enable_realtime_thread_pool_ = false;  // fallback to single-threaded
    }
  }
  return true;
}

void FLIQCJointVelocityStandard::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
}

void FLIQCJointVelocityStandard::planningSceneCallback(const moveit_msgs::PlanningScene::ConstPtr& msg) {
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

void FLIQCJointVelocityStandard::targetedVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  targeted_velocity_ = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
}

void FLIQCJointVelocityStandard::goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  goal_position_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  if(first_receive_goal_ == false){ first_receive_goal_ = true; }
}

void FLIQCJointVelocityStandard::checkPositionConvergence(diagnostic_updater::DiagnosticStatusWrapper &stat){
  if (position_error_norm_ < position_convergence_threshold_) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Position convergence");
  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Position not converged");
  }
  stat.add("Position error norm", position_error_norm_);
  stat.add("Position convergence threshold", position_convergence_threshold_);
}

void FLIQCJointVelocityStandard::checkVelocityConvergence(diagnostic_updater::DiagnosticStatusWrapper &stat){
  if (velocity_norm_ < velocity_convergence_threshold_) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Velocity convergence");
  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Velocity not converged");
  }
  stat.add("Velocity norm", velocity_norm_);
  stat.add("Velocity convergence threshold", velocity_convergence_threshold_);
}

void FLIQCJointVelocityStandard::checkControllerState(diagnostic_updater::DiagnosticStatusWrapper &stat){
  if (error_flag_) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Controller error");
  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Controller running");
  }
}

void FLIQCJointVelocityStandard::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;
  static int submit_counter = 0;
  bool submit_success = true;

  REALTIME_DBGNPROF_START_CLOCK;
  // Snapshot robot state and inputs
  Eigen::VectorXd q = Eigen::VectorXd::Zero(dim_q_);
  Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(dim_q_);
  for (size_t i = 0; i < dim_q_; ++i) {
    q(i) = velocity_joint_handles_[i].getPosition();
    q_dot(i) = velocity_joint_handles_[i].getVelocity();
  }
  std::vector<robot_env_evaluator::obstacleInput> obstacles_local;
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    obstacles_local = obstacles_;
  }
  Eigen::Vector3d goal_position_local = goal_position_;
  Eigen::Vector3d targeted_velocity_local = targeted_velocity_;

  // Build compute function
  const auto *controller_ptr_raw = this->controller_ptr_.get();
  const auto *env_evaluator_ptr_raw = this->env_evaluator_ptr_.get();
  const auto *mass_matrix_bridge_raw = this->mass_matrix_bridge_.get();
  auto compute_func = [q, obstacles_local, goal_position_local, targeted_velocity_local,
                       dim_q_ = this->dim_q_,
                       controller_ptr = controller_ptr_raw,
                       env_evaluator_ptr = env_evaluator_ptr_raw,
                       mass_matrix_bridge = mass_matrix_bridge_raw,
                       robust_pinv_lambda = this->robust_pinv_lambda_,
                       velocity_input_threshold = this->velocity_input_threshold_,
                       switch_to_disable_multi_agent = this->switch_to_disable_multi_agent_]
                      (uint64_t task_id) -> StandardControllerComputationResult {
    StandardControllerComputationResult result;
    result.task_id = task_id;
    try {
      // Thread-local copies of heavy components
      auto* controller_local = realtime_calc_thread_pool::ThreadLocalObjectManager<FLIQC_controller_core::FLIQC_controller_joint_velocity_basic>::getOrCreate(
          "controller", *controller_ptr);
      auto* env_local = realtime_calc_thread_pool::ThreadLocalObjectManager<robot_env_evaluator::RobotEnvEvaluator>::getOrCreate(
          "env_evaluator", *env_evaluator_ptr);
      auto* mass_matrix_bridge_local = realtime_calc_thread_pool::ThreadLocalObjectManager<fliqc_controller_ros::FrankaModelInterfaceBridge>::getOrCreate(
          "mass_matrix_bridge", *mass_matrix_bridge);

      // Distances
      std::vector<robot_env_evaluator::distanceResult> distances;
      PROFILE_DBGNPROF_START_CLOCK;
      env_local->computeDistances(q, obstacles_local, distances);
      PROFILE_DBGNPROF_STOP_CLOCK("computeDistances");

      // Kinematics
      PROFILE_DBGNPROF_START_CLOCK;
      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, dim_q_);
      env_local->forwardKinematics(q, T);
      env_local->jacobian(q, J);

      // Guidance and goal diff
      Eigen::Vector3d now_pos = T.block<3,1>(0,3);
      Eigen::Vector3d goal_diff = goal_position_local - now_pos;
      Eigen::Vector3d goal_diff_regularized = Eigen::Vector3d::Zero();
      Eigen::Vector3d targeted_velocity = targeted_velocity_local; // keep a copy
      if (goal_diff.norm() >= switch_to_disable_multi_agent){
        goal_diff_regularized = targeted_velocity;
      } else {
        goal_diff_regularized = goal_diff;
      }
      if (goal_diff_regularized.norm() > velocity_input_threshold){
        goal_diff_regularized = goal_diff_regularized.normalized() * velocity_input_threshold;
      }
      Eigen::MatrixXd Jpos = J.block(0, 0, 3, dim_q_);
      Eigen::MatrixXd JJt = Jpos * Jpos.transpose();
      Eigen::MatrixXd damped_identity = robust_pinv_lambda * robust_pinv_lambda * Eigen::MatrixXd::Identity(JJt.rows(), JJt.cols());
      Eigen::VectorXd q_dot_guide = Jpos.transpose() * (JJt + damped_identity).ldlt().solve(goal_diff_regularized);
      PROFILE_DBGNPROF_STOP_CLOCK("kinematics");

      // Organize state input
      PROFILE_DBGNPROF_START_CLOCK;
      FLIQC_controller_core::FLIQC_state_input state_input;
      state_input.q_dot_guide = q_dot_guide;
      if (controller_local->quad_cost_type == FLIQC_controller_core::FLIQC_quad_cost_type::FLIQC_QUAD_COST_MASS_MATRIX ||
          controller_local->quad_cost_type == FLIQC_controller_core::FLIQC_quad_cost_type::FLIQC_QUAD_COST_MASS_MATRIX_VELOCITY_ERROR){
        if (mass_matrix_bridge_local) {
          mass_matrix_bridge_local->getMassMatrix(q, state_input.M);
        } else {
          state_input.M = Eigen::MatrixXd::Identity(dim_q_, dim_q_);
        }
      }
      state_input.J = J;

      // Distances to controller inputs
      std::vector<FLIQC_controller_core::FLIQC_distance_input> distance_inputs;
      distance_inputs.reserve(distances.size());
      for (size_t i = 0; i < distances.size(); ++i){
        FLIQC_controller_core::FLIQC_distance_input di;
        di.id = i;
        di.distance = distances[i].distance;
        di.projector_control_to_dist = distances[i].projector_jointspace_to_dist.transpose();
        di.projector_dist_to_control = distances[i].projector_dist_to_jointspace;
        distance_inputs.push_back(std::move(di));
      }
      PROFILE_DBGNPROF_STOP_CLOCK("organizeData");

      // Run controller
      FLIQC_controller_core::FLIQC_control_output control_output;
      control_output.x = Eigen::VectorXd::Zero(0);
      control_output.y = Eigen::VectorXd::Zero(0);
      PROFILE_DBGNPROF_START_CLOCK;
      Eigen::VectorXd q_dot_command = controller_local->runController(state_input, distance_inputs, control_output);
      PROFILE_DBGNPROF_STOP_CLOCK("runController");

      // Fill result
      result.q_dot_command = q_dot_command;
      result.goal_diff = goal_diff;
      result.computation_success = true;

      #if defined(CONTROLLER_DEBUG) || defined(CONTROLLER_PROFILE)
      result.distances = std::move(distances);
      result.T = T;
      result.J = J;
      result.q_dot_guide = q_dot_guide;
      result.goal_position = goal_position_local;
      result.current_position = now_pos;
      result.Jpos = Jpos;
      #endif

    } catch (const FLIQC_controller_core::LCQPowException& e) {
      result.computation_success = false;
      result.error_message = std::string("LCQPowException: ") + e.what();
    } catch (const std::exception& e) {
      result.computation_success = false;
      result.error_message = e.what();
    } catch (...) {
      result.computation_success = false;
      result.error_message = "Unknown exception in compute task";
    }
    return result;
  };

  // Execute compute
  StandardControllerComputationResult computation_result;
  if (enable_realtime_thread_pool_) {
    submit_success = thread_pool_ptr_->submitTask(compute_func);
    computation_result = thread_pool_ptr_->getLatestResult(std::chrono::microseconds(realtime_thread_pool_wait_us_));
    if (submit_success) {
      submit_counter++;
    }
  } else {
    computation_result = compute_func(++submit_counter);
  }
  REALTIME_DBGNPROF_STOP_CLOCK("totalComputation");

  #if defined(CONTROLLER_REALTIME_PROFILE)
  static int pool_delay = 0;
  if (submit_success){
    pool_delay = submit_counter - computation_result.task_id;
  } else {
    pool_delay++;
  }
  DBGNPROF_LOG("submit_success", submit_success);
  DBGNPROF_LOG("pool_delay", pool_delay);
  #endif

  // Extract q_dot_command and other fields using original variable names for debug blocks
  Eigen::VectorXd q_dot_command = computation_result.q_dot_command;
  Eigen::Vector3d goal_diff = computation_result.goal_diff;
  #if defined(CONTROLLER_DEBUG) || defined(CONTROLLER_PROFILE)
  // Export variables matching original debug code
  std::vector<robot_env_evaluator::distanceResult> distances = computation_result.distances;
  Eigen::Matrix4d T = computation_result.T;
  Eigen::MatrixXd J = computation_result.J;
  Eigen::VectorXd q_dot_guide = computation_result.q_dot_guide;
  Eigen::Vector3d now_ = computation_result.current_position;
  Eigen::MatrixXd Jpos = computation_result.Jpos;
  #endif

  // publish and visualize the controller start and goal information
  #ifdef CONTROLLER_DEBUG
  do{
    static ros::NodeHandle node_handle;
    static ros::Time last_publish_time = ros::Time::now();
    if (ros::Time::now() - last_publish_time > ros::Duration(1.0/30) || error_flag_ == true){
      last_publish_time = ros::Time::now();
      static ros::Publisher obs_pub;
      if (!obs_pub){
        obs_pub = node_handle.advertise<visualization_msgs::MarkerArray>("controller_info", 10);
      }
      visualization_msgs::MarkerArray obs_marker_array;
      geometry_msgs::Point point_helper;

      // The velocity guide vector
      visualization_msgs::Marker q_dot_guide_marker;
      q_dot_guide_marker.header.frame_id = "panda_link0";
      q_dot_guide_marker.header.stamp = ros::Time::now();
      q_dot_guide_marker.ns = "controller_info";
      q_dot_guide_marker.id = 2;
      q_dot_guide_marker.type = visualization_msgs::Marker::ARROW;
      q_dot_guide_marker.action = visualization_msgs::Marker::ADD;
      point_helper.x = now_(0);
      point_helper.y = now_(1);
      point_helper.z = now_(2);
      q_dot_guide_marker.points.push_back(point_helper);
      Eigen::Vector3d q_dot_guide_vel = now_ + Jpos * q_dot_guide;
      point_helper.x = q_dot_guide_vel(0);
      point_helper.y = q_dot_guide_vel(1);
      point_helper.z = q_dot_guide_vel(2);
      q_dot_guide_marker.points.push_back(point_helper);
      q_dot_guide_marker.pose.orientation.w = 1.0;
      q_dot_guide_marker.scale.x = 0.005;
      q_dot_guide_marker.scale.y = 0.01;
      q_dot_guide_marker.scale.z = 0.01;
      q_dot_guide_marker.color.a = 0.5;
      q_dot_guide_marker.color.r = 0.0;
      q_dot_guide_marker.color.g = 0.0;
      q_dot_guide_marker.color.b = 1.0;
      obs_marker_array.markers.push_back(q_dot_guide_marker);

      // visualize the real EE velocity
      Eigen::Vector3d real_EE_velocity = J * q_dot_command;
      visualization_msgs::Marker real_EE_velocity_marker;
      real_EE_velocity_marker.header.frame_id = "panda_link0";
      real_EE_velocity_marker.header.stamp = ros::Time::now();
      real_EE_velocity_marker.ns = "controller_result";
      real_EE_velocity_marker.id = 1;
      real_EE_velocity_marker.type = visualization_msgs::Marker::ARROW;
      real_EE_velocity_marker.action = visualization_msgs::Marker::ADD;
      point_helper.x = now_(0);
      point_helper.y = now_(1);
      point_helper.z = now_(2);
      real_EE_velocity_marker.points.push_back(point_helper);
      Eigen::Vector3d real_EE_velocity_end = now_ + real_EE_velocity.head(3);
      point_helper.x = real_EE_velocity_end(0);
      point_helper.y = real_EE_velocity_end(1);
      point_helper.z = real_EE_velocity_end(2);
      real_EE_velocity_marker.points.push_back(point_helper);
      real_EE_velocity_marker.pose.orientation.w = 1.0;
      real_EE_velocity_marker.scale.x = 0.005;
      real_EE_velocity_marker.scale.y = 0.01;
      real_EE_velocity_marker.scale.z = 0.01;
      real_EE_velocity_marker.color.a = 0.9;
      real_EE_velocity_marker.color.r = 0.0;
      real_EE_velocity_marker.color.g = 1.0;
      real_EE_velocity_marker.color.b = 0.0;
      obs_marker_array.markers.push_back(real_EE_velocity_marker);
      
      obs_pub.publish(obs_marker_array);
    }
  } while(false);
  #endif // CONTROLLER_DEBUG
  
  // publish and visualize the world enviroment calculation information
  #ifdef CONTROLLER_DEBUG
    do{
      static ros::NodeHandle node_handle;
      static ros::Time last_publish_time = ros::Time::now();
      if (ros::Time::now() - last_publish_time > ros::Duration(1.0/30) || error_flag_ == true){
        last_publish_time = ros::Time::now();
        static ros::Publisher obs_pub;
        if (!obs_pub){
          obs_pub = node_handle.advertise<visualization_msgs::MarkerArray>("environment_result", 10);
        }
        visualization_msgs::MarkerArray obs_marker_array;
        geometry_msgs::Point point_helper;
        
        // Track previous number of markers to handle shrinking arrays
        static size_t prev_markers_used = 0;
        size_t current_markers = distances.size();
        
        for (size_t i = 0; i < distances.size(); ++i){
          // nearest point on object
          visualization_msgs::Marker nearest_point_marker;
          nearest_point_marker.header.frame_id = "panda_link0";
          nearest_point_marker.header.stamp = ros::Time::now();
          nearest_point_marker.ns = "env_info_obs";
          nearest_point_marker.id = i;
          nearest_point_marker.type = visualization_msgs::Marker::SPHERE;
          nearest_point_marker.action = visualization_msgs::Marker::ADD;
          nearest_point_marker.pose.position.x = distances[i].nearest_point_on_object(0);
          nearest_point_marker.pose.position.y = distances[i].nearest_point_on_object(1);
          nearest_point_marker.pose.position.z = distances[i].nearest_point_on_object(2);
          nearest_point_marker.pose.orientation.w = 1.0;
          nearest_point_marker.scale.x = 0.0075;
          nearest_point_marker.scale.y = 0.0075;
          nearest_point_marker.scale.z = 0.0075;
          nearest_point_marker.color.a = 1.0; 
          nearest_point_marker.color.r = 0.0;
          nearest_point_marker.color.g = 1.0;
          nearest_point_marker.color.b = 0.0;
          obs_marker_array.markers.push_back(nearest_point_marker);

          // nearest point on robot
          nearest_point_marker.ns = "env_info_rbt";
          nearest_point_marker.pose.position.x = distances[i].nearest_point_on_robot(0);
          nearest_point_marker.pose.position.y = distances[i].nearest_point_on_robot(1);
          nearest_point_marker.pose.position.z = distances[i].nearest_point_on_robot(2);
          obs_marker_array.markers.push_back(nearest_point_marker);

          // line connecting two points
          visualization_msgs::Marker connection_line;
          connection_line.header.frame_id = "panda_link0";
          connection_line.header.stamp = ros::Time::now();
          connection_line.ns = "env_info_connection";
          connection_line.id = i;
          connection_line.type = visualization_msgs::Marker::LINE_LIST;
          connection_line.action = visualization_msgs::Marker::ADD;
          point_helper.x = distances[i].nearest_point_on_robot(0);
          point_helper.y = distances[i].nearest_point_on_robot(1);
          point_helper.z = distances[i].nearest_point_on_robot(2);
          connection_line.points.push_back(point_helper);
          point_helper.x = distances[i].nearest_point_on_object(0);
          point_helper.y = distances[i].nearest_point_on_object(1);
          point_helper.z = distances[i].nearest_point_on_object(2);
          connection_line.points.push_back(point_helper);
          connection_line.pose.orientation.w = 1.0;
          connection_line.scale.x = 0.0025;
          connection_line.color.a = 0.1;
          if (distances[i].distance > controller_ptr_->active_threshold){
            connection_line.color.r = 0.0;
            connection_line.color.g = 0.1;
            connection_line.color.b = 0.0;
          } else if (distances[i].distance > controller_ptr_->eps) {
            connection_line.color.r = 1.0;
            connection_line.color.g = 1.0;
            connection_line.color.b = 0.0;
          } else {
            connection_line.color.r = 1.0;
            connection_line.color.g = 0.0;
            connection_line.color.b = 0.0;
          }
          obs_marker_array.markers.push_back(connection_line);

          // normal vector to avoid the obstacle
          visualization_msgs::Marker normal_vector_marker;
          normal_vector_marker.header.frame_id = "panda_link0";
          normal_vector_marker.header.stamp = ros::Time::now();
          normal_vector_marker.ns = "env_info_normal";
          normal_vector_marker.id = i;
          normal_vector_marker.type = visualization_msgs::Marker::ARROW;
          normal_vector_marker.action = visualization_msgs::Marker::ADD;
          point_helper.x = distances[i].nearest_point_on_robot(0);
          point_helper.y = distances[i].nearest_point_on_robot(1);
          point_helper.z = distances[i].nearest_point_on_robot(2);
          normal_vector_marker.points.push_back(point_helper);
          Eigen::Vector3d normal_vector_end = distances[i].nearest_point_on_robot + distances[i].normal_vector * 0.1;
          point_helper.x = normal_vector_end(0);
          point_helper.y = normal_vector_end(1);
          point_helper.z = normal_vector_end(2);
          normal_vector_marker.points.push_back(point_helper);
          normal_vector_marker.pose.orientation.w = 1.0;
          normal_vector_marker.scale.x = 0.005;
          normal_vector_marker.scale.y = 0.01;
          normal_vector_marker.scale.z = 0.01;
          normal_vector_marker.color.a = 0.6;
          if (distances[i].distance > controller_ptr_->active_threshold){
            normal_vector_marker.color.r = 0.0;
            normal_vector_marker.color.g = 0.1;
            normal_vector_marker.color.b = 0.0;
          } else if (distances[i].distance > controller_ptr_->eps) {
            normal_vector_marker.color.r = 1.0;
            normal_vector_marker.color.g = 1.0;
            normal_vector_marker.color.b = 0.0;
          } else {
            normal_vector_marker.color.r = 1.0;
            normal_vector_marker.color.g = 0.0;
            normal_vector_marker.color.b = 0.0;
          }
          obs_marker_array.markers.push_back(normal_vector_marker);
        }
        
        // If current number of markers is less than previous, delete the excess markers
        if (current_markers < prev_markers_used) {
          for (size_t i = current_markers; i < prev_markers_used; ++i) {
            // Delete markers for each namespace
            visualization_msgs::Marker delete_marker;
            delete_marker.header.frame_id = "panda_link0";
            delete_marker.header.stamp = ros::Time::now();
            delete_marker.action = visualization_msgs::Marker::DELETE;
            delete_marker.id = i;
            
            // Delete from each namespace
            delete_marker.ns = "env_info_obs";
            obs_marker_array.markers.push_back(delete_marker);
            
            delete_marker.ns = "env_info_rbt";
            obs_marker_array.markers.push_back(delete_marker);
            
            delete_marker.ns = "env_info_connection";
            obs_marker_array.markers.push_back(delete_marker);
            
            delete_marker.ns = "env_info_normal";
            obs_marker_array.markers.push_back(delete_marker);
          }
        }
        
        // Update previous markers count
        prev_markers_used = current_markers;
        
        obs_pub.publish(obs_marker_array);
      }
    } while(false);
  #endif // CONTROLLER_DEBUG

  #if defined(CONTROLLER_DEBUG) || defined(CONTROLLER_PROFILE)
  DBGNPROF_LOG("q_dot_real", q_dot);
  DBGNPROF_LOG("q_dot_guide", q_dot_guide);
  DBGNPROF_LOG("q_dot_command", q_dot_command);
  Eigen::VectorXd distances_vec = Eigen::VectorXd::Zero(distances.size());
  for (size_t i = 0; i < distances.size(); ++i){
    distances_vec(i) = distances[i].distance;
  }
  DBGNPROF_LOG("distances", distances_vec);

  // Note: lambda logging removed in pooled version to keep compute self-contained
  #endif // CONTROLLER_DEBUG

  // Handle errors from compute task and diagnostics
  if (!computation_result.computation_success && !computation_result.error_message.empty()) {
    error_flag_ = true;
    ROS_ERROR_STREAM_ONCE(controller_name << ": compute task failed: " << computation_result.error_message);
  }

  position_error_norm_ = goal_diff.norm();
  velocity_norm_ = q_dot.norm();
  diag_updater_->update();

  if (!error_flag_) {
    for (size_t i = 0; i < 7; ++i) {
      velocity_joint_handles_[i].setCommand(q_dot_command(i));
    }
    if (position_error_norm_ <= position_convergence_threshold_ && velocity_norm_ <= velocity_convergence_threshold_) {
      ROS_INFO_STREAM_THROTTLE(1, controller_name << ": Goal reached with tolerance [" << position_error_norm_ << 
                                                                   "], and velocity [" << velocity_norm_ << "]");
    }
  } else {
    // enter error handling mode here, now only emegency stop.
    for (size_t i = 0; i < 7; ++i) {
      velocity_joint_handles_[i].setCommand(0);
    }
    throw std::runtime_error("Error in controller, stopping the robot.");
  }
}

void FLIQCJointVelocityStandard::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
  diag_updater_->force_update();
}

}  // namespace fliqc_controller_ros

PLUGINLIB_EXPORT_CLASS(fliqc_controller_ros::FLIQCJointVelocityStandard,
                       controller_interface::ControllerBase)
