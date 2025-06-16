/**
 * @file fliqc_state_source_bridge.hpp
 * @brief Header file for the FrankaModelInterfaceBridge class, which provides an 
 * interface to robot_env_evaluator::KinematicDataBridge
 * 
 * Copyright (c) 2025, Haowen Yao
 * Use of this source code is governed by the MIT license, see LICENSE.
 */
#ifndef FLIQC_CONTROLLER_ROS_HELPERS_HPP_
#define FLIQC_CONTROLLER_ROS_HELPERS_HPP_

#include <type_traits>
#include <sstream>

#define READ_PARAM_SILENT(node_handle, controller_name, param_name, param_variable) \
  if (!node_handle.getParam(param_name, param_variable)) { \
    ROS_ERROR_STREAM(controller_name << ": Could not get parameter " << param_name); \
    return false; \
  }

#define READ_PARAM(node_handle, controller_name, param_name, param_variable) \
  if (!node_handle.getParam(param_name, param_variable)) { \
    ROS_ERROR_STREAM(controller_name << ": Could not get parameter " << param_name); \
    return false; \
  } \
  else{ \
    ROS_INFO_STREAM(controller_name << ": Getting parameter " << param_name << ": " << param_variable); \
  }

#define READ_PARAM_ENUM(node_handle, controller_name, param_name, param_variable, enum_type) \
  { \
    int temp; \
    if (!node_handle.getParam(param_name, temp)) { \
      ROS_ERROR_STREAM(controller_name << ": Could not get parameter " << param_name); \
      return false; \
    } \
    else{ \
      param_variable = static_cast<enum_type>(temp); \
      ROS_INFO_STREAM(controller_name << ": Getting parameter " << param_name << ": " << param_variable << " by enum type " << #enum_type); \
    } \
  }

#define READ_PARAM_VECTOR(node_handle, controller_name, param_name, param_variable) \
  if (!node_handle.getParam(param_name, param_variable)) { \
    ROS_ERROR_STREAM(controller_name << ": Could not get parameter " << param_name); \
    return false; \
  } \
  else { \
    std::ostringstream display_stream; \
    display_stream << "[ "; \
    for (size_t i = 0; i < param_variable.size(); ++i) { \
      display_stream << param_variable[i]; \
      if (i != param_variable.size() - 1) { \
        display_stream << ", "; \
      } \
    } \
    display_stream << " ]"; \
    ROS_INFO_STREAM(controller_name << ": Getting parameter " << param_name << ": " << display_stream.str()); \
  }

#define READ_PARAM_EIGEN(node_handle, controller_name, param_name, param_variable, expected_size) \
  { \
    std::vector<double> temp; \
    if (!node_handle.getParam(param_name, temp)) { \
      ROS_ERROR_STREAM(controller_name << ": Could not get parameter " << param_name); \
      return false; \
    } \
    else { \
      param_variable = Eigen::VectorXd::Map(temp.data(), temp.size()); \
      ROS_INFO_STREAM(controller_name << ": Getting parameter " << param_name << ": [" << param_variable.transpose()<<"]"); \
      if (temp.size() != expected_size) { \
        ROS_ERROR_STREAM(controller_name << ": Parameter " << param_name << " has size " << temp.size() << ", expected " << expected_size); \
        return false; \
      } \
    } \
  }

#define CHECK_NOT_NULLPTR(controller_name, variable) \
  if (variable == nullptr) { \
    ROS_ERROR_STREAM(controller_name << ": Empty evaluation by" << #variable); \
    return false; \
  } 

#define CATCH_BLOCK(controller_name, action_block) \
  try { \
    action_block \
  } catch (const std::exception& ex) {  \
    ROS_ERROR_STREAM(controller_name << ": Exception when executing\n" << #action_block << "\n" << ex.what());  \
    return false;  \
  }

#endif  // FLIQC_CONTROLLER_ROS_HELPERS_HPP_