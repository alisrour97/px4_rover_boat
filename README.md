## Sensitivity experiment
Trajectory planner based on sensitivity to let a drone pass through a window safely

# Getting started
We advise to use Docker to run both PX4 and the offboard application.

## Install Docker
There is a convinient script that facilitate everything
```
curl -fsSL get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```
We suggest to use docker as a non-root user, that way your build folder won't be owned by root after using docker. In a new terminal run
```
# Create docker group (may not be required)
sudo groupadd docker
# Add your user to the docker group.
sudo usermod -aG docker $USER
```
**Now log out and in again before using docker!!**

## Clone PX4 repository
Here do we want to have our forked repository or do we use the official one ? 

## Authors
Ali Srour <br>
Salvatore Marcellini <br>
Tommaso Belvedere <br>

