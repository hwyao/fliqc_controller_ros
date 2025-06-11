import rospy
import os
import subprocess
import datetime

# Constants
ROSBAG_RECORD_PATH = "/home/geriatronics/FLIQC_example_workspace_ros/src/fliqc_controller_ros/benchmark/data-202505080942"
ENV_SCENES = ["figureA1_11.yaml",
              "figureA1_12.yaml",
              "figureA1_13.yaml",
              "figureA1_14.yaml",
              "figureA1_15.yaml",
              "figureA1_16.yaml",
              "figureA1_17.yaml",
              "figureA1_18.yaml",
              "figureA1_19.yaml",
              "figureA1_20.yaml",
              "figureA1_21.yaml",
              "figureA1_22.yaml",
              "figureA1_23.yaml",
              "figureA1_24.yaml",
              "figureA1_25.yaml",
              "figureA1_26.yaml",
              "figureA1_27.yaml",
              "figureA1_28.yaml",
              "figureA1_29.yaml",
              "figureA1_30.yaml",
              "figureB2_11.yaml",
              "figureB2_12.yaml",
              "figureB2_13.yaml",
              "figureB2_14.yaml",
              "figureB2_15.yaml",
              "figureB2_16.yaml",
              "figureB2_17.yaml",
              "figureB2_18.yaml",
              "figureB2_19.yaml",
              "figureB2_20.yaml",
              "figureB2_21.yaml",
              "figureB2_22.yaml",
              "figureB2_23.yaml",
              "figureB2_24.yaml",
              "figureB2_25.yaml",
              "figureB2_26.yaml",
              "figureB2_27.yaml",
              "figureB2_28.yaml",
              "figureB2_29.yaml",
              "figureB2_30.yaml",]
NUM_RUNNING = 20
CONTROLLER = ["fliqc_joint_velocity_standard",
              "apf_joint_velocity",]

# Ensure the ROSBAG_RECORD_PATH directory exists
if not os.path.exists(ROSBAG_RECORD_PATH):
    os.makedirs(ROSBAG_RECORD_PATH)
    print(f"Created directory: {ROSBAG_RECORD_PATH}")

# Log file path
LOG_FILE = os.path.join(ROSBAG_RECORD_PATH, "command_output.log")

# Append file with start time
with open(LOG_FILE, "a") as log_file:
    log_file.write("Benchmark Log at " + str(datetime.datetime.now()) + "\n")
    log_file.write("=" * 50 + "\n")

rospy.set_param("/use_sim_time", True)

# Main script
for controller in CONTROLLER:
    for env_scene in ENV_SCENES:
            for i in range(1, NUM_RUNNING + 1):
                rosbag_record_name = f"{controller}-{env_scene.split('.')[0]}-{i}.bag"
                command = [
                    "roslaunch",
                    "fliqc_controller_ros",
                    "sim_automatic_run_benchmark.launch",
                    f"rosbag_record_path:={ROSBAG_RECORD_PATH}",
                    f"rosbag_record_name:={rosbag_record_name}",
                    f"env_scene:={env_scene}",
                    f"controller:={controller}",
                ]
                print(f"Running: {' '.join(command)}")
                try:
                    with open(LOG_FILE, "a") as log_file:
                        log_file.write(f"Running: {' '.join(command)}\n")
                        subprocess.run(command, check=True, stdout=log_file, stderr=log_file)
                except subprocess.CalledProcessError as e:
                    error_message = f"Error occurred while running command: {e}. Skipping to the next iteration.\n"
                    print(error_message)
                    with open(LOG_FILE, "a") as log_file:
                        log_file.write(error_message)