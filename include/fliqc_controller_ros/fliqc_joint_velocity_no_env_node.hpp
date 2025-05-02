#pragma once

#include <string>
#include <vector>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <franka_hw/franka_model_interface.h>

#include <FLIQC_controller_core/FLIQC_controllers.hpp>
#include <robot_env_evaluator/robot_env_evaluator.hpp>
#include "fliqc_controller_ros/fliqc_state_source_bridge.hpp"

namespace fliqc_controller_ros {

class FLIQCJointVelocityNoEnvNode : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaModelInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

 private:
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  ros::Duration elapsed_time_;

  std::unique_ptr<FLIQC_controller_core::FLIQC_controller_joint_velocity_basic> controller_ptr_;
  std::unique_ptr<robot_env_evaluator::RobotEnvEvaluator> env_evaluator_ptr_;
  std::unique_ptr<robot_env_evaluator::KinematicDataBridge> mass_matrix_bridge_;

  int dim_q_;                          ///< The dimension of the joint q 

  // simulated obstacle list
  std::vector<Eigen::Vector3d> obsList_;   
  std::vector<double> obsRadiusList_; 

  // error flag
  bool error_flag_ = false;
};

}  // namespace fliqc_controller_ros
