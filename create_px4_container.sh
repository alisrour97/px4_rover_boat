# enable access to xhost from the container
xhost +

# Run docker and open bash shell
docker run -it --privileged \
-u root \
--env "DISPLAY" \
-v $(pwd)/../PX4-Autopilot/:/home/user/PX4-Autopilot/:rw \
-v $(pwd)/../PX4_Experiments/:/home/user/PX4_Experiments/:rw \
--network host \
--workdir="/home/user/" \
--name=px4_ros2 dock_px4:Humble


#--env=LOCAL_USER_ID="$(id -u)" \
