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
#endif // CONTROLLER_DEBUG

#ifdef CONTROLLER_PROFILE
#define DBGNPROF_USE_ROS
#define DBGNPROF_ENABLE_DEBUG
#define DBGNPROF_ENABLE_PROFILE
#endif // CONTROLLER_PROFILE

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
  if (preset->isFrankaRobot()){
    franka_hw::FrankaModelInterface* model_interface_ = robot_hardware->get<franka_hw::FrankaModelInterface>();
    CHECK_NOT_NULLPTR(controller_name, model_interface_);
    CATCH_BLOCK(controller_name,
      mass_matrix_bridge_ = std::make_unique<FrankaModelInterfaceBridge>(model_interface_, arm_id);
    )
  }

  // subscribe to the targeted velocity and goal to distance from the multi-agent system
  targeted_velocity_sub_ = node_handle.subscribe("/agent_twist_global", 1, &FLIQCJointVelocityStandard::targetedVelocityCallback, this);
  dist_to_goal_sub_ = node_handle.subscribe("/distance_to_goal", 1, &FLIQCJointVelocityStandard::distanceToGoalCallback, this);

  // subscribe to the planning scene information and wait for the first received message
  planning_scene_sub_ = node_handle.subscribe("/planning_scene", 1, &FLIQCJointVelocityStandard::planningSceneCallback, this);

  // clear and warm up the data until the controller started
  ROS_INFO_STREAM(controller_name << ": Controller starting...");
  error_flag_ = false;
  targeted_velocity_ = Eigen::Vector3d::Zero();
  distance_to_goal_ = 100.0;
  first_receive_obstacle_ = false;
  obstacles_.clear();

  // wait until first message received from all subscribers
  ros::Rate rate(10);
  while (ros::ok() && (!first_receive_obstacle_)) {
      ROS_INFO_STREAM_THROTTLE(1, controller_name << ": Waiting for scene messages...");
      ros::spinOnce();
      rate.sleep();
  }
  ROS_INFO_STREAM(controller_name << ": Controller started.");

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

void FLIQCJointVelocityStandard::distanceToGoalCallback(const std_msgs::Float64::ConstPtr& msg) {
  distance_to_goal_ = msg->data;
}

void FLIQCJointVelocityStandard::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  // collect the obstacle information. In this file, we make up our own obstacle information
  Eigen::VectorXd q = Eigen::VectorXd::Zero(dim_q_);
  std::vector<robot_env_evaluator::distanceResult> distances;
  for (size_t i = 0; i < dim_q_; ++i) {
    q(i) = velocity_joint_handles_[i].getPosition();
  }
  
  DBGNPROF_START_CLOCK; 
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    env_evaluator_ptr_->computeDistances(q, obstacles_, distances);
  }
  DBGNPROF_STOP_CLOCK("computeDistances");

  // //distances
  // for (size_t i = 0; i < distances.size(); i++) {
  //   ROS_INFO_STREAM("[STEP1]FLIQCJointVelocityStandard: Distance " << i << " is "
  //       << distances[i].distance << "\n"
  //       << "Contacted Robot Link: " << distances[i].contacted_robot_link << "\n"
  //       << "Projector Jointspace to Distance: " << distances[i].projector_jointspace_to_dist.transpose() << "\n"
  //       << "Projector Distance to Jointspace: " << distances[i].projector_dist_to_jointspace.transpose() << "\n");
  // }

  DBGNPROF_START_CLOCK;
  // Calculate the targeted velocity goal
  Eigen::VectorXd q_dot_guide(dim_q_);
  
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, dim_q_);
  env_evaluator_ptr_ -> forwardKinematics(q, T);
  env_evaluator_ptr_ -> jacobian(q, J);

  Eigen::Vector3d now_ = T.block<3, 1>(0, 3);
  Eigen::Vector3d goal_diff = targeted_velocity_;
  Eigen::Vector3d goal_diff_regularized = goal_diff;
  double vel = 0.05;
  if (goal_diff_regularized.norm() > (vel/0.5)){
    goal_diff_regularized = goal_diff_regularized.normalized() * 0.05;
  } else {
    goal_diff_regularized = goal_diff * 0.5;
  }
  Eigen::MatrixXd Jpos = J.block<3, 7>(0, 0);
  q_dot_guide = Jpos.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(goal_diff_regularized);
  DBGNPROF_STOP_CLOCK("Kinematics");

  DBGNPROF_START_CLOCK;
  // Calculate the controller cost input
  FLIQC_controller_core::FLIQC_state_input state_input;
  if (controller_ptr_->quad_cost_type == FLIQC_controller_core::FLIQC_quad_cost_type::FLIQC_QUAD_COST_MASS_MATRIX ||
      controller_ptr_->quad_cost_type == FLIQC_controller_core::FLIQC_quad_cost_type::FLIQC_QUAD_COST_MASS_MATRIX_VELOCITY_ERROR){
    mass_matrix_bridge_->getMassMatrix(q, state_input.M);
  }
  state_input.J = J;  

  // Get the obstacle distance information and convert it as the distance input for the controller
  std::vector<FLIQC_controller_core::FLIQC_distance_input> distance_inputs;
  for (size_t i = 0; i < distances.size(); ++i){
    FLIQC_controller_core::FLIQC_distance_input distance_input;
    distance_input.id = i;
    distance_input.distance = distances[i].distance;
    distance_input.projector_control_to_dist = distances[i].projector_jointspace_to_dist.transpose();
    distance_input.projector_dist_to_control = distances[i].projector_dist_to_jointspace;
    distance_inputs.push_back(distance_input);
  }
  DBGNPROF_STOP_CLOCK("organizeData");

  // //debug: distance_inputs
  // for (size_t i = 0; i < distance_inputs.size(); ++i){
  //   ROS_INFO_STREAM("[STEP2]FLIQCJointVelocityStandard: Distance " << i << " is " 
  //       << distance_inputs[i].distance << " with projector " << std::endl << distance_inputs[i].projector_control_to_dist << std::endl
  //       << "and reverse projector " << std::endl << distance_inputs[i].projector_dist_to_control.transpose());
  // }

  // //debug: distance_inputs activated
  // for (size_t i = 0; i < distance_inputs.size(); ++i){
  //   if (distance_inputs[i].distance < controller_ptr_->active_threshold){
  //     ROS_INFO_STREAM("[STEP2_COND]FLIQCJointVelocityStandard: Distance " << i << " is " 
  //         << distance_inputs[i].distance << " with projector " << std::endl << distance_inputs[i].projector_control_to_dist);
  //   }
  // }

  Eigen::VectorXd q_dot_command;

  // run the controller
  if (!error_flag_){
    try {
      DBGNPROF_START_CLOCK;
      q_dot_command = controller_ptr_->runController(q_dot_guide, state_input, distance_inputs);
      DBGNPROF_STOP_CLOCK("runController");

    } catch (const FLIQC_controller_core::LCQPowException& e) {
      error_flag_ = true;
      ROS_ERROR_STREAM_ONCE(controller_name << ": LCQPowException caught during runController:\n" << e.what());
  
      try {
        // Get the root path of the package using rospack
        std::string package_path = ros::package::getPath("fliqc_controller_ros");
        if (package_path.empty()) {
          ROS_ERROR_STREAM("Failed to find package path for fliqc_controller_ros.");
          throw std::runtime_error("Package path not found.");
        }
  
        // Create the log directory if it doesn't exist
        std::string log_dir = package_path + "/log";
        std::filesystem::create_directories(log_dir);
  
        // Create a subdirectory with the current time
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        std::ostringstream time_stream;
        time_stream << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d_%H-%M-%S-%Z");
        std::string base_dir = log_dir + "/" + time_stream.str() + "_" + controller_name;
        std::filesystem::create_directories(base_dir);
  
        // Create subdirectories for different types of logs
        FLIQC_controller_core::logLCQPowExceptionAsFile(e, base_dir);
  
      } catch (const std::exception& ex) {
        ROS_ERROR_STREAM("Failed to save exception to log: " << ex.what());
      }
    } catch (const std::exception& e) {
      error_flag_ = true;
      ROS_ERROR_STREAM(controller_name << ": std::exception caught during runController:\n" << e.what());
    } catch (...) {
      error_flag_ = true;
      ROS_ERROR_STREAM(controller_name << ": Unknown exception caught during runController.");
    }
  }

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
        // The current EE position
        visualization_msgs::Marker diff_arrow;
        diff_arrow.header.frame_id = "panda_link0";
        diff_arrow.header.stamp = ros::Time::now(); 
        diff_arrow.ns = "controller_info";
        diff_arrow.id = 1;
        diff_arrow.type = visualization_msgs::Marker::ARROW;
        diff_arrow.action = visualization_msgs::Marker::ADD;
        point_helper.x = now_(0);
        point_helper.y = now_(1);
        point_helper.z = now_(2);
        diff_arrow.points.push_back(point_helper);
        Eigen::Vector3d goal_diff_regularized_after = now_ + goal_diff_regularized;
        point_helper.x = goal_diff_regularized_after(0);
        point_helper.y = goal_diff_regularized_after(1);
        point_helper.z = goal_diff_regularized_after(2);
        diff_arrow.points.push_back(point_helper);
        diff_arrow.pose.orientation.w = 1.0;
        diff_arrow.scale.x = 0.005;
        diff_arrow.scale.y = 0.01;
        diff_arrow.scale.z = 0.01;
        diff_arrow.color.a = 0.6;
        diff_arrow.color.r = 0.0;
        diff_arrow.color.g = 1.0;
        diff_arrow.color.b = 0.0;
        obs_marker_array.markers.push_back(diff_arrow);

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
        q_dot_guide_marker.color.a = 0.4;
        q_dot_guide_marker.color.r = 0.0;
        q_dot_guide_marker.color.g = 0.0;
        q_dot_guide_marker.color.b = 1.0;
        obs_marker_array.markers.push_back(q_dot_guide_marker);
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
        obs_pub.publish(obs_marker_array);
      }
    } while(false);
  #endif // CONTROLLER_DEBUG

  // publish and visualize the controller output information
  #ifdef CONTROLLER_DEBUG
  do{
    static ros::NodeHandle node_handle;
    static ros::Time last_publish_time = ros::Time::now();
    if (ros::Time::now() - last_publish_time > ros::Duration(1.0/30) || error_flag_ == true){
      last_publish_time = ros::Time::now();
      static ros::Publisher obs_pub;
      if (!obs_pub){
        obs_pub = node_handle.advertise<visualization_msgs::MarkerArray>("controller_result", 10);
      }
      visualization_msgs::MarkerArray obs_marker_array;
      geometry_msgs::Point point_helper;
      
      Eigen::Vector3d real_EE_velocity = J * q_dot_command;
      // visualize the real EE velocity
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

      visualization_msgs::Marker real_EE_velocity_marker_rotation = real_EE_velocity_marker;
      real_EE_velocity_marker_rotation.id = 2;
      real_EE_velocity_marker_rotation.points.clear();
      point_helper.x = now_(0);
      point_helper.y = now_(1);
      point_helper.z = now_(2);
      real_EE_velocity_marker_rotation.points.push_back(point_helper);
      Eigen::Vector3d real_EE_velocity_end_rotation = now_ + real_EE_velocity.tail(3);
      point_helper.x = real_EE_velocity_end_rotation(0);
      point_helper.y = real_EE_velocity_end_rotation(1);
      point_helper.z = real_EE_velocity_end_rotation(2);
      real_EE_velocity_marker_rotation.points.push_back(point_helper);
      real_EE_velocity_marker.scale.x = 0.003;
      real_EE_velocity_marker.color.g = 1.0;
      real_EE_velocity_marker.color.b = 1.0;
      obs_marker_array.markers.push_back(real_EE_velocity_marker_rotation);
      
      // visualize the goal velocity
      obs_pub.publish(obs_marker_array);
    }
  } while(false);
  #endif // CONTROLLER_DEBUG

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

void FLIQCJointVelocityStandard::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace fliqc_controller_ros

PLUGINLIB_EXPORT_CLASS(fliqc_controller_ros::FLIQCJointVelocityStandard,
                       controller_interface::ControllerBase)
