/**
 * @file fliqc_joint_velocity_no_env_node.hpp
 * @brief Header file for the FLIQCJointVelocityNoEnvNode controller class.
 * 
 * Copyright (c) 2025, Haowen Yao
 * Use of this source code is governed by the MIT license, see LICENSE.
 */
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
#include <realtime_calc_thread_pool/thread_pool.hpp>
#include "fliqc_controller_ros/fliqc_state_source_bridge.hpp"

namespace fliqc_controller_ros {

/**
 * @brief Result structure for controller computation tasks
 */
struct ControllerComputationResult {
    // Primary result
    Eigen::Matrix<double, 7, 1> q_dot_command;
    Eigen::Vector3d goal_diff;
    
    // Debugging information
    #ifdef CONTROLLER_DEBUG
    std::vector<robot_env_evaluator::distanceResult> distances;
    Eigen::Matrix4d T;
    Eigen::MatrixXd J;
    Eigen::VectorXd q_dot_guide;
    Eigen::Vector3d goal_position;
    Eigen::Vector3d current_position;
    Eigen::MatrixXd Jpos;
    #endif

    // Status information
    bool computation_success = false;
    std::string error_message;
    uint64_t task_id = 0;
    
    // Default constructor
    ControllerComputationResult() : computation_success(false) {
        q_dot_command = Eigen::Matrix<double, 7, 1>::Zero();
    }
    
    void setZero() {
        q_dot_command = Eigen::Matrix<double, 7, 1>::Zero();
        computation_success = false;
    }
};

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

  std::unique_ptr<realtime_calc_thread_pool::RealtimeThreadPool<ControllerComputationResult>> thread_pool_ptr_;

  std::unique_ptr<FLIQC_controller_core::FLIQC_controller_joint_velocity_basic> controller_ptr_;
  std::unique_ptr<robot_env_evaluator::RobotEnvEvaluator> env_evaluator_ptr_;
  std::unique_ptr<fliqc_controller_ros::FrankaModelInterfaceBridge> mass_matrix_bridge_;

  int dim_q_;                          ///< The dimension of the joint q 

  // simulated obstacle list
  std::vector<Eigen::Vector3d> obsList_;   
  std::vector<double> obsRadiusList_; 

  // state and diagnostics variables
  bool error_flag_ = false;

  // the parameters for the controller
  double robust_pinv_lambda_ = 0.001;
  bool record_fliqc_throw_as_text_ = false;
  bool enable_realtime_thread_pool_ = true;
  int realtime_thread_pool_size_ = 4;
  int realtime_thread_priority_ = 50;
  int realtime_main_priority_ = 98;
  std::vector<int> realtime_thread_affinity_ = {};
  int realtime_main_affinity_ = 0;
  bool require_thread_rt_priority_ = false;
  bool require_main_rt_priority_ = true;
  int realtime_thread_pool_wait_us_ = 300;
};

}  // namespace fliqc_controller_ros
