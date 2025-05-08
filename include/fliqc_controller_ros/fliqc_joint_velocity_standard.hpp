#pragma once

#include <string>
#include <vector>
#include <mutex>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <franka_hw/franka_model_interface.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <FLIQC_controller_core/FLIQC_controllers.hpp>
#include <robot_env_evaluator/robot_env_evaluator.hpp>
#include "fliqc_controller_ros/fliqc_state_source_bridge.hpp"

#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/PlanningScene.h>

namespace fliqc_controller_ros {

class FLIQCJointVelocityStandard : public controller_interface::MultiInterfaceController
                                                <hardware_interface::VelocityJointInterface,
                                                 franka_hw::FrankaModelInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

  void planningSceneCallback(const moveit_msgs::PlanningScene::ConstPtr& msg);
  void targetedVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void checkPositionConvergence(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void checkVelocityConvergence(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void checkControllerState(diagnostic_updater::DiagnosticStatusWrapper &stat);

 private:
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  ros::Duration elapsed_time_;

  std::unique_ptr<FLIQC_controller_core::FLIQC_controller_joint_velocity_basic> controller_ptr_;
  std::unique_ptr<robot_env_evaluator::RobotEnvEvaluator> env_evaluator_ptr_;
  std::unique_ptr<robot_env_evaluator::KinematicDataBridge> mass_matrix_bridge_;
  std::unique_ptr<diagnostic_updater::Updater> diag_updater_;

  int dim_q_;                          ///< The dimension of the joint q 

  // the subscriber list
  ros::Subscriber targeted_velocity_sub_;
  ros::Subscriber planning_scene_sub_;
  ros::Subscriber goal_sub_;

  // the subscriber variables: the obstacle list, targeted velocity and goal
  Eigen::Vector3d targeted_velocity_ = Eigen::Vector3d::Zero(); 
  std::vector<robot_env_evaluator::obstacleInput> obstacles_;
  bool first_receive_obstacle_ = false;
  Eigen::Vector3d goal_position_ = Eigen::Vector3d::Zero();
  bool first_receive_goal_ = false;

  // the mutex for the obstacles
  std::mutex obstacles_mutex_;

  // state and diagnostics variables
  bool error_flag_ = false;
  double position_error_norm_ = 100.0;
  double velocity_norm_ = 0.0;

  // the parameters for the controller
  double position_convergence_threshold_ = 0.005;
  double velocity_convergence_threshold_ = 0.05;
  double robust_pinv_lambda_ = 0.001;
  double velocity_input_threshold_ = 0.05;
  double switch_to_disable_multi_agent_  = 0.025;
};

}  // namespace fliqc_controller_ros