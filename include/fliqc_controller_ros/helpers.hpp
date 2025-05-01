#ifndef FLIQC_CONTROLLER_ROS_HELPERS_HPP_
#define FLIQC_CONTROLLER_ROS_HELPERS_HPP_

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

#define CHECK_NOT_EMPTY(controller_name, empty_evaluation) \
  if (empty_evaluation) { \
    ROS_ERROR_STREAM(controller_name << ": Empty evaluation by" << #empty_evaluation); \
    return false; \
  } \

#define CATCH_BLOCK(controller_name, action_block) \
  try { \
    action_block \
  } catch (const std::exception& ex) {  \
    ROS_ERROR_STREAM(controller_name << ": Exception when executing\n" << #action_block << "\n" << ex.what());  \
    return false;  \
  }

#endif  // FLIQC_CONTROLLER_ROS_HELPERS_HPP_