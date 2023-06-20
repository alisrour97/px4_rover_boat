#!/usr/bin/env python3

import subprocess
import time

# pid_simu = subprocess.check_output("HEADLESS=1 make px4_sitl gazebo-classic &", cwd="../PX4-Autopilot/", shell=True)
# time.sleep(3)
#pid_agent = subprocess.check_output("./MicroXRCEAgent udp4 -p 8888 ", cwd="../Micro-XRCE-DDS-Agent/build/", shell=True)
# time.sleep(2)
#traj_pid = subprocess.check_output("chmod +x ros_setup.sh ; ./ros_setup.sh ; ros2 run trajectory_publisher trajectory_publisher &", shell=True)

output = subprocess.check_output("find $(pwd)/../PX4-Autopilot/build/px4_sitl_default/rootfs/log/ -type f -printf \"%T@ %p\n\" | sort -n | cut -d' ' -f 2- | tail -n 1", shell=True);

log_file = str(output)[2:-3];

print(log_file)

subprocess.call("plotjuggler -n -d "+log_file+" -l px4_layout.xml", shell=True)
