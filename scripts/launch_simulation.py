#!/usr/bin/env python3
"""
launch_simulation.py
Author: Haowen Yao
This script is a ROS node designed to launch a Gazebo simulation for a specified robot configuration.
Usage:
    Run this script as a ROS node, ensuring all required parameters are set in the private namespace.
"""
import roslaunch
import rospy
from rospkg import RosPack
import os
import yaml

def main():
    rospy.init_node('launch_simulation_node', anonymous=True)

    # Retrieve all private parameters and pass them as arguments
    private_params = rospy.get_param_names()
    args = {}
    for param in private_params:
        if param.startswith(rospy.get_name() + '/'):  # Check for parameters in the node's namespace
            param_name = param.split('/')[-1]         # Extract the parameter name
            args[param_name] = rospy.get_param(param)
    rospy.loginfo(f"[launch_simulation_node] Private parameters: {args}")

    # Check for required parameters in args
    try:
        param_robot = args['robot']
        param_arm_id = args['arm_id']
        param_controller = args['controller']
        param_env_scene = args['env_scene']
    except KeyError as e:
        rospy.logerr(f"Parameter '~{e.args[0]}' is required but not set.")
        raise rospy.ROSInitException(f"Missing required parameter: '~{e.args[0]}'")

    # Load the package path of robot_env_publisher YAML and find initial joint positions
    rospack = RosPack()
    try:
        robot_env_publisher_path = rospack.get_path('robot_env_publisher')
    except Exception as e:
        rospy.logerr(f"Failed to find package 'robot_env_publisher': {e}")
        raise rospy.ROSInitException("Could not locate 'robot_env_publisher' package.")
    
    scene_file_path = f"{robot_env_publisher_path}/scene/{param_env_scene}"
    if not os.path.isfile(scene_file_path):
        rospy.logerr(f"[launch_simulation_node] Scene file not found: {scene_file_path}")
        raise rospy.ROSInitException(f"Scene file not found: {scene_file_path}")

    try:
        with open(scene_file_path, 'r') as file:
            scene_config = yaml.safe_load(file)
    except Exception as e:
        rospy.logerr(f"Failed to load scene file as YAML '{scene_file_path}': {e}")
        raise rospy.ROSInitException(f"Error loading scene file: {scene_file_path}")

    # Check for initial_configuration and initial_configuration[arm_id]
    initial_joint_positions = None
    if 'initial_configuration' in scene_config:
        if param_arm_id in scene_config['initial_configuration']:
            initial_joint_positions = scene_config['initial_configuration'][param_arm_id]
            rospy.loginfo(f"[launch_simulation_node] Loaded initial joint positions for '{param_arm_id}': {initial_joint_positions}")
        else:
            rospy.logwarn(f"[launch_simulation_node] 'initial_configuration' exists but does not contain '{param_arm_id}'.")
    else:
        rospy.logwarn(f"[launch_simulation_node] 'initial_configuration' not found in the scene configuration.")

    # Phrase the initial joint positions to a string that Gazebo can understand according to the robot type
    initial_joint_positions_str = None
    if initial_joint_positions is not None:
        if param_robot == 'panda':
            # Expecting a list of 7 joint angles
            if len(initial_joint_positions) != 7:
                rospy.logerr(f"[launch_simulation_node] Expected 7 joint angles for 'panda', but got {len(initial_joint_positions)}.")
                raise rospy.ROSInitException("Invalid number of joint angles for 'panda'.")
            
            # Construct the joint positions string
            initial_joint_positions_str = ' '.join(
                [f"-J {param_arm_id}_joint{i+1} {angle}" for i, angle in enumerate(initial_joint_positions)]
            )
            # Add finger joints with default values
            initial_joint_positions_str += f" -J {param_arm_id}_finger_joint1 0.001 -J {param_arm_id}_finger_joint2 0.001"
        else:
            rospy.logerr(f"[launch_simulation_node] Unsupported robot type: {param_robot}")
            raise rospy.ROSInitException(f"Unsupported robot type: {param_robot}")
        rospy.loginfo(f"[launch_simulation_node] Initial joint positions string: {initial_joint_positions_str}")

    # Launch the simulation
    package = 'fliqc_controller_ros'
    launch_file = 'gazebo_sim.launch'
    
    # Retrieve uuid and configure logging
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Create the launch path with arguments
    launch_file = roslaunch.rlutil.resolve_launch_arguments([package, launch_file])[0]
    launch_args = ['robot:=' + param_robot, 
                   'arm_id:=' + param_arm_id, 
                   'controller:=' + param_controller]
    if initial_joint_positions_str is not None:
        launch_args.append('initial_joint_positions:=' + initial_joint_positions_str)

    roslaunch_file = [(launch_file, launch_args)]  # Fix the incorrect slicing

    rospy.loginfo(f"[launch_simulation_node] Launching file: {roslaunch_file}")

    # Create a roslaunch parent and start it
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start()

    rospy.loginfo("[launch_simulation_node] Launching Gazebo simulation...")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[launch_simulation_node] Shutting down...")
        parent.shutdown()

if __name__ == '__main__':
    main()