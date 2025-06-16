/**
 * @file apf_joint_velocity.hpp
 * @brief Header file for the APFJointVelocity controller class.
 *
 * Copyright (c) 2025, Bingkun Huang, Haowen Yao
 * Use of this source code is governed by the MIT license, see LICENSE.
 */
#pragma once

#include <string>
#include <vector>
#include <mutex>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <FLIQC_controller_core/FLIQC_controllers.hpp>
#include <robot_env_evaluator/robot_env_evaluator.hpp>

#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <moveit_msgs/PlanningScene.h>
#include <geometry_msgs/PoseStamped.h>


namespace fliqc_controller_ros {

class APFJointVelocity : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

  void planningSceneCallback(const moveit_msgs::PlanningScene::ConstPtr& msg);
  void goalPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void checkPositionConvergence(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void checkVelocityConvergence(diagnostic_updater::DiagnosticStatusWrapper &stat);
 private:
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  ros::Duration elapsed_time_;

  std::unique_ptr<robot_env_evaluator::RobotEnvEvaluator> env_evaluator_ptr_;
  std::unique_ptr<diagnostic_updater::Updater> diag_updater_;

  int dim_q_; //< The dimension of the joint q 
  
  // the subscriber list
  ros::Subscriber planning_scene_sub_;
  ros::Subscriber goal_pos_sub_; 
  
  // the subscriber variables
  std::vector<robot_env_evaluator::obstacleInput> obstacles_;
  bool first_receive_obstacle_ = false;
  Eigen::Vector3d goal_pos_ = Eigen::Vector3d::Zero(); 
  Eigen::Quaterniond  goal_orientation_ = Eigen::Quaterniond(1, 0.0, 0.0, 0.0);
  bool first_receive_goal_ = false;
  
  // the mutex for the obstacles
  std::mutex obstacles_mutex_;

  // state and diagnostics variables
  double position_error_norm_ = 100.0;
  double velocity_norm_ = 0.0;

  // the parameters for the controller
  double position_convergence_threshold_ = 0.005;
  double velocity_convergence_threshold_ = 0.05;
  double robust_pinv_lambda_ = 0.004;
  double velocity_input_threshold_ = 0.05;

  // the parameters for the potential field
  double attraction_kp_ = 3.0;
  double repulsion_agent_r_ = 0.05;
  double repulsion_kp_ = 0.8;
  double repulsion_dist_ = 0.12;
  double repulsion_wholebody_kp_ = 1.0;
  double repulsion_wholebody_dist_ = 0.2;
  double regularization_vmax_ = 0.1;
  double regularization_kv_ = 2.0;
};

}  // namespace fliqc_controller_ros