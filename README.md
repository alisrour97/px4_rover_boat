## Jetski Simulations PX4
This repo serves as prove of concept for guidance, navigation and control for jetskies powered by the PX4 autopilot in Gazebo.

## Getting started
We advise to use Docker to run both PX4 and the offboard application. 
One can refer to the following link for the details of docker installation
[PX4 ROS2 Docker](https://github.com/alisrour97/px4-dev-simulation-ros2-humble.git)



### 2) Clone this repository
Open a terminal, go in the folder where you want to clone this repository and run the command
```
git clone https://github.com/alisrour97/px4_jet_ski.git --recursive
```
The **--recursive** flag is required to fetch all of the PX4 submodules.


### 3) Clone PX4 firmware
Clone the firmware repository, in the same folder where you cloned this repository
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

### 4) Clone the uXRCE-DDS client 
This is the client needed to communicate with PX4 from ROS2.
```
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
```


## Run the simulation
Now that everything is set up, you need to compile PX4 in order to run the simulation, start the uXRCE-DDS client and then start the controller in ROS2.
Remember that to build PX4 and to run the controller you must be inside the docker container, otherwise you will miss the dependencies to do that (to start the uXRCE-DDS client you must not be in the container if you compiled it outside the container).

### 1) Start the PX4 simulation
Considering that we will use the Gazebo GUI, in every terminal where you will start the container or open a terminal into it, first run the command
```
xhost +
```

If you closed the container, then start it with the command
```
docker start px4_ros2
docker attach px4_ros2
```
If instead you have already started the container in a terminal and want to open another terminal in the container, run the command
```
docker exec -u 0 -it px4_ros2 bash
```
In the container, go in the firmware folder and build it to start the simulation
```
cd PX4-Autopilot
make px4_sitl gazebo-classic
```
At the end of the building process you will see the Gazebo simulation opened.

### 2) Start the communication client
Open a new terminal and go into the repository and start the client
```
cd Micro-XRCE-DDS-Agent/build
./MicroXRCEAgent udp4 -p 8888
```


### 3) Start the ROS2 application
Open a new terminal in the container and, only for the first time, build the ROS2 workspace
```
xhost +
docker exec -u 0 -it px4_ros2 bash
source ../../opt/ros/foxy/setup.bash
cd PX4_Experiments
colcon build
```
At the end of the building process, or if you already have built before, run the commands
```
source install/setup.bash
ros2 run trajectory_publisher rover_publisher
```
You should see Rover Ackermann moving with toward specific position

![Alt text](image/gazebo_iris.png)



### 4) Log files

The log files are found when navigating to

```
cd ../PX4-Autopilot/build/px4_sitl_default/rootfs

```

Sometimes you have to run the following to get permission to plot logs in plotjuggler 

```
sudo chown -R tesla:tesla *

```
my username is tesla


## Authors
Ali Srour <br>

