import os
import subprocess

# Constants
ROSBAG_RECORD_PATH = "/home/geriatronics/FLIQC_example_workspace_ros/src/fliqc_controller_ros/benchmark/data-202505071512"
ENV_SCENES = ["figureA1_11.yaml",
              "figureB2_11.yaml",]
NUM_RUNNING = 2

# Ensure the ROSBAG_RECORD_PATH directory exists
if not os.path.exists(ROSBAG_RECORD_PATH):
    os.makedirs(ROSBAG_RECORD_PATH)
    print(f"Created directory: {ROSBAG_RECORD_PATH}")

# Main script
for env_scene in ENV_SCENES:
    for i in range(1, NUM_RUNNING + 1):
        rosbag_record_name = f"{env_scene.split('.')[0]}-{i}.bag"
        command = [
            "roslaunch",
            "fliqc_controller_ros",
            "sim_automatic_run_benchmark.launch",
            f"rosbag_record_path:={ROSBAG_RECORD_PATH}",
            f"rosbag_record_name:={rosbag_record_name}",
            f"env_scene:={env_scene}"
            f"controller:=fliqc_joint_velocity_standard",
        ]
        print(f"Running: {' '.join(command)}")
        try:
            subprocess.run(command, check=True)
        except subprocess.CalledProcessError as e:
            print(f"Error occurred while running command: {e}. Skipping to the next iteration.")