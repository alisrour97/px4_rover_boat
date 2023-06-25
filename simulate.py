#!/usr/bin/env python3

import subprocess
import time
import argparse

parser = argparse.ArgumentParser(description='Multiple px4 trajectory simulations')

# Required positional argument
parser.add_argument('n_iter', type=int,
                    help='Number of simulations')

parser.add_argument('traj_length', type=int,
                    help='Trajectory duration in seconds')

args = parser.parse_args()

client = gazebo = subprocess.Popen("gnome-terminal -- ./../Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888",
                              stdout=subprocess.PIPE, stdin=subprocess.PIPE, encoding='utf8', shell=True)

for i in range(args.n_iter) :
    
    start = subprocess.Popen("docker start px4_ros2",stdout=subprocess.PIPE,universal_newlines=True, shell=True)

    gazebo = subprocess.Popen("gnome-terminal -- docker exec -it px4_ros2 bash -c 'cd PX4-Autopilot; HEADLESS=1 make px4_sitl gazebo-classic'",
                              stdout=subprocess.PIPE, stdin=subprocess.PIPE, encoding='utf8', shell=True)
    time.sleep(10)
    traj = subprocess.Popen("gnome-terminal -- docker exec -it px4_ros2 bash -c 'cd sensitivity_experiment; source install/setup.bash; ros2 run trajectory_publisher trajectory_publisher'",
                            stdout=subprocess.PIPE, stdin=subprocess.PIPE, encoding='utf8', shell=True)

    time.sleep(args.traj_length + 5)
    stop = subprocess.Popen("docker stop px4_ros2",stdout=subprocess.PIPE,universal_newlines=True, shell=True)
    time.sleep(2)
