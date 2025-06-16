/**
 * @file fliqc_state_source_bridge.hpp
 * @brief Header file for the FrankaModelInterfaceBridge class, which provides an 
 * interface to robot_env_evaluator::KinematicDataBridge
 * 
 * Copyright (c) 2025, Haowen Yao
 * Use of this source code is governed by the MIT license, see LICENSE.
 */
#ifndef FLIQC_CONTROLLER_ROS_FLIQC_STATE_SOURCE_BRIDGE_HPP
#define FLIQC_CONTROLLER_ROS_FLIQC_STATE_SOURCE_BRIDGE_HPP

#include <robot_env_evaluator/state_source_bridge.hpp>

#include <memory>
#include <franka_hw/franka_model_interface.h>

namespace fliqc_controller_ros
{
  class FrankaModelInterfaceBridge : public robot_env_evaluator::KinematicDataBridge {
    public:
        /**
         * @brief Construct a new FrankaModelInterfaceBridge object
         * 
         * @param model_handle The Franka model handle
         */
        FrankaModelInterfaceBridge(franka_hw::FrankaModelInterface* model_interface, const std::string& arm_id);

        /**
         * @brief Destroy the FrankaModelInterfaceBridge object
         */
        ~FrankaModelInterfaceBridge() override = default;

        /**
         * @brief Get the mass matrix for the given joint configuration
         * 
         * @param[in] q The joint configuration
         * @param[out] result The mass matrix
         */
        void getMassMatrix(const Eigen::VectorXd& q, Eigen::MatrixXd& result) override;
    
    protected:
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    };
}

#endif // FLIQC_CONTROLLER_ROS_FLIQC_STATE_SOURCE_BRIDGE_HPP