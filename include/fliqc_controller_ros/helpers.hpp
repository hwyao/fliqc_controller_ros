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

#endif  // FLIQC_CONTROLLER_ROS_HELPERS_HPP_