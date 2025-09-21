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
         * @param model_interface The Franka model interface
         * @param arm_id The arm identifier
         */
        FrankaModelInterfaceBridge(franka_hw::FrankaModelInterface* model_interface, const std::string& arm_id);

        /**
         * @brief Copy constructor for FrankaModelInterfaceBridge object
         * 
         * @param other The FrankaModelInterfaceBridge object to copy from
         * @note Creates a new handle to the same model interface and arm_id
         */
        FrankaModelInterfaceBridge(const FrankaModelInterfaceBridge& other);

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
        franka_hw::FrankaModelInterface* model_interface_;  ///< Pointer to the model interface (not owned)
        std::string arm_id_;                                ///< The arm identifier
    };
}

#endif // FLIQC_CONTROLLER_ROS_FLIQC_STATE_SOURCE_BRIDGE_HPP