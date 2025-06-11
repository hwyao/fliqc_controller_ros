#!/usr/bin/env python3
import rospy
import roslaunch
from rospkg import RosPack
from diagnostic_msgs.msg import DiagnosticArray
import time

# Global variables
is_error_detected = False
is_position_convergence = False
is_velocity_convergence = False
MAX_RUNTIME = 60  # Maximum runtime in seconds

def diagnostics_callback(msg, parent):
    global is_position_convergence, is_velocity_convergence, is_error_detected
    for status in msg.status:
        if status.hardware_id == 'panda':
            if status.name == 'gazebo: Position convergence':
                if status.level == 0:
                    if not is_position_convergence:
                        rospy.loginfo("[launch_simulation_node] Position convergence detected.")
                    is_position_convergence = True
                elif status.level == 1:
                    if is_position_convergence:
                        rospy.loginfo("[launch_simulation_node] Position convergence lost.")
                    is_position_convergence = False

            elif status.name == 'gazebo: Velocity convergence':
                if status.level == 0:
                    if not is_velocity_convergence:
                        rospy.loginfo("[launch_simulation_node] Velocity convergence detected.")
                    is_velocity_convergence = True
                elif status.level == 1:
                    if is_velocity_convergence:
                        rospy.loginfo("[launch_simulation_node] Velocity convergence lost.")
                    is_velocity_convergence = False

            elif status.name == 'gazebo: Controller state':
                if status.level == 2:
                    is_error_detected = True
                    rospy.logwarn("[launch_simulation_node] Error detected, shutting down...")
                    parent.shutdown()
                    rospy.signal_shutdown("Node shutdown because of error condition.")
                else:
                    is_error_detected = False

def start_simulation():
    rospy.init_node('automatic_start_stop_simulation', anonymous=True)

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
        param_env_scene = args['env_scene']
        param_controller = args['controller']
        param_record_bag_path = args['rosbag_record_path']
        param_record_bag_name = args['rosbag_record_name']
    except KeyError as e:
        rospy.logerr(f"Parameter '~{e.args[0]}' is required but not set.")
        raise rospy.ROSInitException(f"Missing required parameter: '~{e.args[0]}'")

    # Launch the simulation
    package = 'fliqc_controller_ros'
    launch_file = 'sim_collect_benchmark.launch'

    # Retrieve uuid and configure logging
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Create the launch path with arguments
    launch_file = roslaunch.rlutil.resolve_launch_arguments([package, launch_file])[0]
    launch_args = ['robot:=panda', 
                   'arm_id:=panda', 
                   'env_scene:=' + param_env_scene,
                   'controller:=' + param_controller,
                   'record_bag_path:=' + param_record_bag_path,
                   'record_bag_name:=' + param_record_bag_name]
    roslaunch_file = [(launch_file, launch_args)]  

    rospy.loginfo(f"[launch_simulation_node] Launching file: {roslaunch_file}")
    rospy.loginfo(f"[launch_simulation_node] Arguments: {launch_args}")

    # Create a roslaunch parent and start it
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start()
    start_time = time.time()
    rospy.loginfo("[launch_simulation_node] Simulation launched.")

    # Subscribe to /diagnostics
    rospy.Subscriber('/diagnostics', DiagnosticArray, diagnostics_callback, parent)

    # Run in 20Hz to check for convergence
    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            global is_position_convergence, is_velocity_convergence, is_error_detected

            # Check for position and velocity convergence
            if is_position_convergence and is_velocity_convergence:
                rospy.loginfo("[launch_simulation_node] Position and velocity convergence detected, shutting down...")
                parent.shutdown()
                rospy.signal_shutdown("Node shutdown because of condition satisfied.")

            # Check for maximum runtime
            elapsed_time = time.time() - start_time
            if elapsed_time > MAX_RUNTIME:
                rospy.logwarn(f"[launch_simulation_node] Maximum runtime of {MAX_RUNTIME}s exceeded, shutting down...")
                parent.shutdown()
                rospy.signal_shutdown("Node shutdown because maximum runtime exceeded.")

            rate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("[launch_simulation_node] Shutting down with user interrupt...")
        parent.shutdown()
        rospy.signal_shutdown("Node shutdown because of user interrupt.")


if __name__ == '__main__':
    try:
        start_simulation()
    except rospy.ROSInterruptException:
        pass
