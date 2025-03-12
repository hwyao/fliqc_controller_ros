#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/model.hpp>

#include "fliqc_controller_ros/fliqc_joint_velocity_standard.hpp"
#include "fliqc_controller_ros/helpers.hpp"
#include <robot_env_evaluator/robot_env_evaluator_path.h>
#include <robot_env_evaluator/robot_presets.hpp>

#include <cmath>
#include <array>

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

bool FLIQCJointVelocityStandard::init(hardware_interface::RobotHW* robot_hardware,
                                                   ros::NodeHandle& node_handle) {
  // Set the variables for this controller
  std::string controller_name = "FLIQCJointVelocityStandard";
                                            
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
  READ_PARAM(node_handle, controller_name, 
      "/fliqc_controller_core/buffer_history", controller_ptr_->buffer_history);
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
      "/fliqc_controller_core/dt", controller_ptr_->dt);
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_core/eps", controller_ptr_->eps);
  READ_PARAM(node_handle, controller_name,
      "/fliqc_controller_core/active_threshold", controller_ptr_->active_threshold);
  
  std::vector<double> q_dot_max;
  READ_PARAM_SILENT(node_handle, controller_name, "/fliqc_controller_core/q_dot_max", q_dot_max);
  controller_ptr_->q_dot_max = Eigen::Map<Eigen::VectorXd>(q_dot_max.data(), q_dot_max.size());
  ROS_INFO_STREAM(controller_name << ": Getting parameter q_dot_max: " << controller_ptr_->q_dot_max.transpose());

  // Initialize the robot environment evaluator in robot_env_evaluator
  pinocchio::Model model;
  std::string ee_name;
  pinocchio::GeometryModel collision_model;
  auto preset = robot_env_evaluator::RobotPresetFactory::createRobotPreset("FrankaEmika");
  CHECK_NOT_EMPTY(controller_name, preset == nullptr);
  preset->getPresetRobot(model, ee_name, collision_model);

  env_evaluator_ptr_ = std::make_unique<robot_env_evaluator::RobotEnvEvaluator>(model, ee_name, collision_model);

  // subscribe to the planning scene information and wait for the first received message
  ros::Subscriber planning_scene_sub = node_handle.subscribe("/planning_scene", 1, &FLIQCJointVelocityStandard::planningSceneCallback, this);
  // wait until first message received and obstacles_ is not empty
  ros::Rate rate(10);
  while (ros::ok() && obstacles_.empty()) {
      ros::spinOnce();
      ROS_INFO_STREAM_THROTTLE(1, controller_name << "Waiting for first planning_scene message...");
      rate.sleep();
  }

  // subscribe to the targeted velocity of the multi-agent system
  ros::Subscriber targeted_velocity_sub = node_handle.subscribe("/agent_twist", 1, &FLIQCJointVelocityStandard::targetedVelocityCallback, this);

  return true;
}

void FLIQCJointVelocityStandard::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
}

void FLIQCJointVelocityStandard::planningSceneCallback(const moveit_msgs::PlanningScene::ConstPtr& msg) {
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

void FLIQCJointVelocityStandard::targetedVelocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  targeted_velocity_ = Eigen::Vector3d(msg->linear.x, msg->linear.y, msg->linear.z);
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
  env_evaluator_ptr_ -> computeDistances(q, obstacles_, distances);
  env_evaluator_ptr_ -> InspectGeomModelAndData();

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
  
  // publish and visualize the controller start and goal information
  #ifdef CONTROLLER_DEBUG
  do{
    static ros::NodeHandle node_handle;
    static ros::Time last_publish_time = ros::Time::now();
    if (ros::Time::now() - last_publish_time > ros::Duration(1.0/30)){
      last_publish_time = ros::Time::now();
      static ros::Publisher obs_pub;
      if (!obs_pub){
        obs_pub = node_handle.advertise<visualization_msgs::MarkerArray>("obstacles", 1);
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
        q_dot_guide_marker.color.a = 0.6;
        q_dot_guide_marker.color.r = 0.0;
        q_dot_guide_marker.color.g = 0.0;
        q_dot_guide_marker.color.b = 1.0;
        obs_marker_array.markers.push_back(q_dot_guide_marker);
      obs_pub.publish(obs_marker_array);
    }
  } while(false);
  #endif // CONTROLLER_DEBUG

  // Calculate the controller cost input
  FLIQC_controller_core::FLIQC_cost_input cost_input;
  cost_input.Q = Eigen::MatrixXd::Identity(dim_q_, dim_q_);
  cost_input.g = Eigen::VectorXd::Zero(dim_q_);

  // //distances
  // for (size_t i = 0; i < distances.size(); i++){
  //   ROS_INFO_STREAM("[STEP1]FLIQCJointVelocityStandard: Distance " << i << " is "
  //       << distances[i].distance << " with projector " << std::endl 
  //       << distances[i].projector_jointspace_to_dist.transpose());
  // }
  
  // publish and visualize the world enviroment calculation information
  #ifdef CONTROLLER_DEBUG
    do{
      static ros::NodeHandle node_handle;
      static ros::Time last_publish_time = ros::Time::now();
      if (ros::Time::now() - last_publish_time > ros::Duration(1.0/30)){
        last_publish_time = ros::Time::now();
        static ros::Publisher obs_pub;
        if (!obs_pub){
          obs_pub = node_handle.advertise<visualization_msgs::MarkerArray>("obstacles", 1);
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

  // Get the obstacle distance information and convert it as the distance input for the controller
  std::vector<FLIQC_controller_core::FLIQC_distance_input> distance_inputs;
  for (size_t i = 0; i < distances.size(); ++i){
    FLIQC_controller_core::FLIQC_distance_input distance_input;
    distance_input.id = i;
    distance_input.distance = distances[i].distance;
    distance_input.projector_control_to_dist = distances[i].projector_jointspace_to_dist.transpose();
    distance_inputs.push_back(distance_input);
  }

  // //debug: distance_inputs
  // for (size_t i = 0; i < distance_inputs.size(); ++i){
  //   ROS_INFO_STREAM("[STEP2]FLIQCJointVelocityStandard: Distance " << i << " is " 
  //       << distance_inputs[i].distance << " with projector " << std::endl << distance_inputs[i].projector_control_to_dist);
  // }

  //debug: distance_inputs activated
  // for (size_t i = 0; i < distance_inputs.size(); ++i){
  //   if (distance_inputs[i].distance < controller_ptr_->active_threshold){
      
  //     ROS_INFO_STREAM("[STEP2_COND]FLIQCJointVelocityStandard: Distance " << i << " is " 
  //         << distance_inputs[i].distance << " with projector " << std::endl << distance_inputs[i].projector_control_to_dist);
  //   }
  // }

  // run the controller
  Eigen::VectorXd q_dot_command = controller_ptr_->runController(q_dot_guide, cost_input, distance_inputs);
  if (goal_diff.norm() >= 0.01) {
    for (size_t i = 0; i < 7; ++i) {
      velocity_joint_handles_[i].setCommand(q_dot_command(i));
    }
  }else{
    ROS_INFO_ONCE("LCQPControllerFrankaModel: Goal reached with tolerance %f", goal_diff.norm());
    for (size_t i = 0; i < 7; ++i) {
      velocity_joint_handles_[i].setCommand(0);
    }
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
