/**
 * @file fliqc_joint_velocity_no_env_node.cpp
 * @brief Implementation file for the FLIQCJointVelocityNoEnvNode controller class.
 * 
 * Copyright (c) 2025, Haowen Yao
 * Use of this source code is governed by the MIT license, see LICENSE.
 */
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/model.hpp>

#include "fliqc_controller_ros/fliqc_joint_velocity_no_env_node.hpp"
#include "fliqc_controller_ros/helpers.hpp"
#include <robot_env_evaluator/robot_env_evaluator_path.h>
#include <robot_env_evaluator/robot_presets.hpp>

#include <cmath>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>
#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

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
#define DBGNPROF_ENABLE_PROFILE
#undef REALTIME_DBGNPROF_START_CLOCK
#undef REALTIME_DBGNPROF_STOP_CLOCK
#define REALTIME_DBGNPROF_START_CLOCK DBGNPROF_START_CLOCK
#define REALTIME_DBGNPROF_STOP_CLOCK DBGNPROF_STOP_CLOCK
#endif // CONTROLLER_REALTIME_PROFILE

#include <debug_and_profile_helper/helper_macros.hpp>

namespace fliqc_controller_ros {

// Set the variables for this controller
static std::string controller_name = "FLIQCJointVelocityNoEnvNode";

bool FLIQCJointVelocityNoEnvNode::init(hardware_interface::RobotHW* robot_hardware,
                                                   ros::NodeHandle& node_handle) {
  // Set the variables for this controller
                                            
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
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_ros/robust_pinv_lambda", robust_pinv_lambda_);
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_ros/record_fliqc_throw_as_text", record_fliqc_throw_as_text_);
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

  // simulate virtual dynamic obstacle information
  obsList_.push_back(Eigen::Vector3d(0.25, 0.5, 0.6)); 
  obsRadiusList_.push_back(0.05);
  obsList_.push_back(Eigen::Vector3d(-0.13, -0.52, 0.5)); 
  obsRadiusList_.push_back(0.1);
  obsList_.push_back(Eigen::Vector3d(0.25, 0.3, 0.6)); 
  obsRadiusList_.push_back(0.05);
  obsList_.push_back(Eigen::Vector3d(0.30, 0.3, 0.6));
  obsRadiusList_.push_back(0.05); 

  // Initialize the error flag
  error_flag_ = false;

  // Initialize the thread pool
  if (enable_realtime_thread_pool_) {
    realtime_calc_thread_pool::Config realtime_config(
      realtime_thread_pool_size_, 
      realtime_thread_priority_, 
      realtime_main_priority_,
      realtime_thread_affinity_, 
      realtime_main_affinity_,
      require_thread_rt_priority_,
      require_main_rt_priority_
    );
    thread_pool_ptr_ = std::make_unique<realtime_calc_thread_pool::RealtimeThreadPool<ControllerComputationResult>>(realtime_config);
  }

  return true;
}

void FLIQCJointVelocityNoEnvNode::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
}

void FLIQCJointVelocityNoEnvNode::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  // simulate virutal dynamic obstacle information
  static int phase = 1;
  if (phase == 1){
    obsList_[1] = obsList_[1] + Eigen::Vector3d(0, 0.10 * 0.001, 0);
    if (obsList_[1](1) > - 0.1){
        phase = 2;
    }
  }
  else{
    obsList_[1] = obsList_[1] + Eigen::Vector3d(0, -0.05 * 0.001, 0);
    obsList_[2] = obsList_[2] + Eigen::Vector3d(0, -0.05 * 0.001, 0);
    obsList_[0] = obsList_[0] + Eigen::Vector3d(0, -0.05 * 0.001, 0);
    obsList_[3] = obsList_[3] + Eigen::Vector3d(0, -0.05 * 0.001, 0);
  }

  // collect the obstacle information. In this file, we make up our own obstacle information
  Eigen::VectorXd q = Eigen::VectorXd::Zero(dim_q_);
  std::vector<robot_env_evaluator::obstacleInput> obstacles;
  for (size_t i = 0; i < dim_q_; ++i) {
    q(i) = velocity_joint_handles_[i].getPosition();
  }
  for (size_t i = 0; i < obsList_.size(); ++i){
    coal::Sphere sphere(obsRadiusList_[i]);
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose << 1, 0, 0, obsList_[i](0),
            0, 1, 0, obsList_[i](1),
            0, 0, 1, obsList_[i](2),
            0, 0, 0, 1;
    obstacles.push_back({sphere, pose});
  }
  
  // publish and visualize the obstacles made by this controller
  // make a publisher for the obstacle, do it in 30Hz
  REALTIME_DBGNPROF_START_CLOCK;
  do{
    static ros::NodeHandle node_handle;
    static ros::Time last_publish_time = ros::Time::now();
    if (ros::Time::now() - last_publish_time > ros::Duration(1.0/30)){
      last_publish_time = ros::Time::now();
      static ros::Publisher obs_pub;
      if (!obs_pub){
        obs_pub = node_handle.advertise<visualization_msgs::MarkerArray>("obstacles", 10);
      }
      // publish the obstacle
      visualization_msgs::MarkerArray obs_marker_array;
      for (size_t i = 0; i < obsList_.size(); ++i){
        visualization_msgs::Marker obs_marker;
        obs_marker.header.frame_id = "panda_link0";
        obs_marker.header.stamp = ros::Time::now();
        obs_marker.ns = "obstacle";
        obs_marker.id = i;
        obs_marker.type = visualization_msgs::Marker::SPHERE;
        obs_marker.action = visualization_msgs::Marker::ADD;
        obs_marker.pose.position.x = obsList_[i](0);
        obs_marker.pose.position.y = obsList_[i](1);
        obs_marker.pose.position.z = obsList_[i](2);
        obs_marker.pose.orientation.x = 0.0;
        obs_marker.pose.orientation.y = 0.0;
        obs_marker.pose.orientation.z = 0.0;
        obs_marker.pose.orientation.w = 1.0;
        obs_marker.scale.x = 2 * obsRadiusList_[i];
        obs_marker.scale.y = 2 * obsRadiusList_[i];
        obs_marker.scale.z = 2 * obsRadiusList_[i];
        obs_marker.color.a = 1.0;
        obs_marker.color.r = 0.0;
        obs_marker.color.g = 0.0;
        obs_marker.color.b = 1.0;
        obs_marker_array.markers.push_back(obs_marker);
      }
      obs_pub.publish(obs_marker_array);
    }
  } while(false);

  // ========== Compute Function Creation ========== //
  const auto *controller_ptr_raw = this->controller_ptr_.get();
  const auto *env_evaluator_ptr_raw = this->env_evaluator_ptr_.get();
  const auto *mass_matrix_bridge_raw = this->mass_matrix_bridge_.get();
  auto compute_func = [q, obstacles, dim_q_ = this->dim_q_, 
                      controller_ptr = controller_ptr_raw, 
                      env_evaluator_ptr = env_evaluator_ptr_raw, 
                      mass_matrix_bridge = mass_matrix_bridge_raw]
                      (uint64_t task_id) -> ControllerComputationResult {
    ControllerComputationResult result;
    result.task_id = task_id;

    // Get or create thread-local controller copy
    auto* controller_local = realtime_calc_thread_pool::ThreadLocalObjectManager<FLIQC_controller_core::FLIQC_controller_joint_velocity_basic>::getOrCreate(
        "controller", *controller_ptr);
    
    // Get or create thread-local environment evaluator copy
    auto* env_evaluator_local = realtime_calc_thread_pool::ThreadLocalObjectManager<robot_env_evaluator::RobotEnvEvaluator>::getOrCreate(
        "env_evaluator", *env_evaluator_ptr);

    // Get or create thread-local mass matrix bridge copy (if available)
    FrankaModelInterfaceBridge* mass_matrix_bridge_local = nullptr;
    if (mass_matrix_bridge != nullptr) {
        mass_matrix_bridge_local = realtime_calc_thread_pool::ThreadLocalObjectManager<FrankaModelInterfaceBridge>::getOrCreate(
            "mass_matrix_bridge", *mass_matrix_bridge);
    }

    try {
      std::vector<robot_env_evaluator::distanceResult> distances;

      PROFILE_DBGNPROF_START_CLOCK;
      env_evaluator_local->computeDistances(q, obstacles, distances);
      PROFILE_DBGNPROF_STOP_CLOCK("computeDistances");

      // //distances
      // for (size_t i = 0; i < distances.size(); i++){
      //   ROS_INFO_STREAM("[STEP1]FLIQCJointVelocityNoEnvNode: Distance " << i << " is "
      //       << distances[i].distance << " with projector " << std::endl 
      //       << distances[i].projector_jointspace_to_dist.transpose());
      // }

      PROFILE_DBGNPROF_START_CLOCK;
      // Get the kinematics information
      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, dim_q_);
      env_evaluator_local->forwardKinematics(q, T);
      env_evaluator_local->jacobian(q, J);

      // Calculate the targeted velocity goal
      Eigen::VectorXd q_dot_guide(dim_q_);
      Eigen::Vector3d goal_(0.1, 0.450, 0.55);
      Eigen::Vector3d now_ = T.block<3, 1>(0, 3);
      Eigen::Vector3d goal_diff = goal_ - now_;
      Eigen::Vector3d goal_diff_regularized = goal_diff;
      double vel = 0.05;
      if (goal_diff_regularized.norm() > (vel/0.5)){
        goal_diff_regularized = goal_diff_regularized.normalized() * 0.05;
      } else {
        goal_diff_regularized = goal_diff * 0.5;
      }
      Eigen::MatrixXd Jpos = J.block<3, 7>(0, 0);
      q_dot_guide = Jpos.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(goal_diff_regularized);
      PROFILE_DBGNPROF_STOP_CLOCK("kinematics");

      PROFILE_DBGNPROF_START_CLOCK;
      // Calculate the controller cost input
      FLIQC_controller_core::FLIQC_state_input state_input;
      state_input.q_dot_guide = q_dot_guide;
      if (controller_local->quad_cost_type == FLIQC_controller_core::FLIQC_quad_cost_type::FLIQC_QUAD_COST_MASS_MATRIX ||
          controller_local->quad_cost_type == FLIQC_controller_core::FLIQC_quad_cost_type::FLIQC_QUAD_COST_MASS_MATRIX_VELOCITY_ERROR){
        if (mass_matrix_bridge_local != nullptr) {
          mass_matrix_bridge_local->getMassMatrix(q, state_input.M);
        }
      }
      state_input.J = J;  //tbd = Eigen::VectorXd::Zero(dim_q_);

      // Get the obstacle distance information and convert it as the distance input for the controller
      std::vector<FLIQC_controller_core::FLIQC_distance_input> distance_inputs;
      for (size_t i = 0; i < distances.size(); ++i){
        FLIQC_controller_core::FLIQC_distance_input distance_input;
        distance_input.id = i;
        distance_input.distance = distances[i].distance;
        distance_input.projector_control_to_dist = distances[i].projector_jointspace_to_dist.transpose();
        distance_inputs.push_back(distance_input);
      }
      PROFILE_DBGNPROF_STOP_CLOCK("organizeData");

      // //debug: distance_inputs
      // for (size_t i = 0; i < distance_inputs.size(); ++i){
      //   ROS_INFO_STREAM("[STEP2]FLIQCJointVelocityNoEnvNode: Distance " << i << " is " 
      //       << distance_inputs[i].distance << " with projector " << std::endl << distance_inputs[i].projector_control_to_dist);
      // }

      // //debug: distance_inputs activated
      // for (size_t i = 0; i < distance_inputs.size(); ++i){
      //   if (distance_inputs[i].distance < controller_ptr_->active_threshold){
          
      //     ROS_INFO_STREAM("[STEP2_COND]FLIQCJointVelocityNoEnvNode: Distance " << i << " is " 
      //         << distance_inputs[i].distance << " with projector " << std::endl << distance_inputs[i].projector_control_to_dist);
      //   }
      // }

      // run the controller
      FLIQC_controller_core::FLIQC_control_output control_output;

      PROFILE_DBGNPROF_START_CLOCK;
      result.q_dot_command = controller_local->runController(state_input, distance_inputs, control_output);
      PROFILE_DBGNPROF_STOP_CLOCK("runController");
      result.computation_success = true;

      // Store debug information when debug mode is enabled
      result.goal_diff = goal_diff;

      #ifdef CONTROLLER_DEBUG
      result.distances = distances;
      result.T = T;
      result.J = J;
      result.q_dot_guide = q_dot_guide;
      result.goal_position = goal_;
      result.current_position = now_;
      result.Jpos = Jpos;
      #endif

    } catch (const FLIQC_controller_core::LCQPowException& e) {
      result.computation_success = false;
      result.error_message = std::string("LCQPowException: ") + e.what();

      // try {
      //   // Get the root path of the package using rospack
      //   std::string package_path = ros::package::getPath("fliqc_controller_ros");
      //   if (package_path.empty()) {
      //     ROS_ERROR_STREAM("Failed to find package path for fliqc_controller_ros.");
      //     throw std::runtime_error("Package path not found.");
      //   }

      //   // Create the log directory if it doesn't exist
      //   std::string log_dir = package_path + "/log";
      //   std::filesystem::create_directories(log_dir);

      //   // Create a subdirectory with the current time
      //   auto now = std::chrono::system_clock::now();
      //   auto time_t_now = std::chrono::system_clock::to_time_t(now);
      //   std::ostringstream time_stream;
      //   time_stream << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d_%H-%M-%S-%Z");
      //   std::string base_dir = log_dir + "/" + time_stream.str() + "_" + controller_name;
      //   std::filesystem::create_directories(base_dir);

      //   // Create subdirectories for different types of logs
      //   FLIQC_controller_core::logLCQPowExceptionAsFile(e, base_dir);

      // } catch (const std::exception& ex) {
      //   ROS_ERROR_STREAM("Failed to save exception to log: " << ex.what());
      // }
    } catch (const std::exception& e) {
      result.computation_success = false;
      result.error_message = std::string("std::exception: ") + e.what();
    } catch (...) {
      result.computation_success = false;
      result.error_message = "Unknown exception occurred";
    }
    
    return result;
  };

  // ========== Computation Execution ========== //
  ControllerComputationResult computation_result;
  
  if (enable_realtime_thread_pool_) {
    thread_pool_ptr_->submitTask(compute_func);
    computation_result = thread_pool_ptr_->getLatestResult(std::chrono::microseconds(realtime_thread_pool_wait_us_));
  } else {
    computation_result = compute_func(0);
  }
  REALTIME_DBGNPROF_STOP_CLOCK("totalComputation");

  // Extract q_dot_command and other commands for use by rest of the function
  Eigen::VectorXd q_dot_command = computation_result.q_dot_command;
  Eigen::Vector3d goal_diff = computation_result.goal_diff;
  #ifdef CONTROLLER_DEBUG
  // Extract debug variables with exact same names for visualization code
  std::vector<robot_env_evaluator::distanceResult> distances = computation_result.distances;
  Eigen::Matrix4d T = computation_result.T;
  Eigen::MatrixXd J = computation_result.J;
  Eigen::VectorXd q_dot_guide = computation_result.q_dot_guide;
  Eigen::Vector3d goal_ = computation_result.goal_position;
  Eigen::Vector3d now_ = computation_result.current_position;
  Eigen::MatrixXd Jpos = computation_result.Jpos;
  #endif

  // publish and visualize the controller goal information, EE guide and real velocity
  #ifdef CONTROLLER_DEBUG
  do{
    static ros::NodeHandle node_handle;
    static ros::Time last_publish_time = ros::Time::now();
    if (ros::Time::now() - last_publish_time > ros::Duration(1.0/30)|| error_flag_ == true){
      last_publish_time = ros::Time::now();
      static ros::Publisher obs_pub;
      if (!obs_pub){
        obs_pub = node_handle.advertise<visualization_msgs::MarkerArray>("controller_info", 10);
      }
      visualization_msgs::MarkerArray obs_marker_array;
      geometry_msgs::Point point_helper;
        // The goal position
        visualization_msgs::Marker goal_marker;
        goal_marker.header.frame_id = "panda_link0";
        goal_marker.header.stamp = ros::Time::now();
        goal_marker.ns = "controller_info";
        goal_marker.id = 0;
        goal_marker.type = visualization_msgs::Marker::SPHERE;
        goal_marker.action = visualization_msgs::Marker::ADD;
        goal_marker.pose.position.x = goal_(0);
        goal_marker.pose.position.y = goal_(1);
        goal_marker.pose.position.z = goal_(2);
        goal_marker.pose.orientation.w = 1.0;
        goal_marker.scale.x = 0.01;
        goal_marker.scale.y = 0.01;
        goal_marker.scale.z = 0.01;
        goal_marker.color.a = 1.0;
        goal_marker.color.r = 1.0;
        goal_marker.color.g = 0.0;
        goal_marker.color.b = 0.0;
        obs_marker_array.markers.push_back(goal_marker);

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
        Eigen::VectorXd real_EE_velocity = J * q_dot_command;
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
      if (ros::Time::now() - last_publish_time > ros::Duration(1.0/30)|| error_flag_ == true){
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
          connection_line.color.a = 0.6;
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

  // Handle errors from compute task and diagnostics
  if (!computation_result.computation_success && !computation_result.error_message.empty()) {
    error_flag_ = true;
    ROS_ERROR_STREAM_ONCE(controller_name << ": compute task failed: " << computation_result.error_message);
  }

  if (!error_flag_) {
    for (size_t i = 0; i < 7; ++i) {
      velocity_joint_handles_[i].setCommand(q_dot_command(i));
    }
    if (goal_diff.norm() <= 0.005) {
      ROS_INFO_STREAM_THROTTLE(1, controller_name << ": Goal reached with tolerance " << goal_diff.norm());
    }
  } else {
    // enter error handling mode here, now only emegency stop.
    for (size_t i = 0; i < 7; ++i) {
      velocity_joint_handles_[i].setCommand(0);
    }
    throw std::runtime_error("Error in controller, stopping the robot.");
  }
}

void FLIQCJointVelocityNoEnvNode::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace fliqc_controller_ros

PLUGINLIB_EXPORT_CLASS(fliqc_controller_ros::FLIQCJointVelocityNoEnvNode,
                       controller_interface::ControllerBase)
