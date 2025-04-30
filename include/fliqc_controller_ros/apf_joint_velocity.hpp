#pragma once

#include <string>
#include <vector>
#include <mutex>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

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
  void targetedVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void distanceToGoalCallback(const std_msgs::Float64::ConstPtr& msg);
  void goalPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
 private:
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  ros::Duration elapsed_time_;

  std::unique_ptr<robot_env_evaluator::RobotEnvEvaluator> env_evaluator_ptr_;

  int dim_q_; //< The dimension of the joint q 

  // the obstacle list
  std::vector<robot_env_evaluator::obstacleInput> obstacles_;
  // the subscriber list
  ros::Subscriber targeted_velocity_sub_;
  ros::Subscriber dist_to_goal_sub_;
  ros::Subscriber planning_scene_sub_;
  ros::Subscriber goal_pos_sub_; 
  // the targeted velocity and distance to goal
  Eigen::Vector3d targeted_velocity_ = Eigen::Vector3d(-100,-100,-100);
  double distance_to_goal_ = -100;

  Eigen::Vector3d goal_pos_ = Eigen::Vector3d::Zero(); // store goal pos 
  Eigen::Quaterniond  goal_orientation_ = Eigen::Quaterniond(1, 0.0, 0.0, 0.0); // store goal orientation
  // the mutex for the obstacles
  std::mutex obstacles_mutex_;
};

}  // namespace fliqc_controller_ros