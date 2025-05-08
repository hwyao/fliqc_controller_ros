import os
import subprocess
import datetime

# Constants
ROSBAG_RECORD_PATH = "/home/geriatronics/FLIQC_example_workspace_ros/src/fliqc_controller_ros/benchmark/data-202505071512"
ENV_SCENES = ["figureA1_11.yaml",
              "figureB2_11.yaml",]
NUM_RUNNING = 2
CONTROLLER = ["fliqc_joint_velocity_standard",
              "apf_joint_velocity",]

# Ensure the ROSBAG_RECORD_PATH directory exists
if not os.path.exists(ROSBAG_RECORD_PATH):
    os.makedirs(ROSBAG_RECORD_PATH)
    print(f"Created directory: {ROSBAG_RECORD_PATH}")

# Log file path
LOG_FILE = os.path.join(ROSBAG_RECORD_PATH, "command_output.log")

# Clear the log file at the start of the script
with open(LOG_FILE, "w") as log_file:
    log_file.write("Benchmark Log at " + str(datetime.datetime.now()) + "\n")
    log_file.write("=" * 50 + "\n")

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