# Martian Mines
Our system for the 2024 Droniada Challenge competition in the Martian Mines category.

### Overview
The Martian Mines project is designed to participate in the 2024 Droniada Challenge. It involves a system that integrates various components to detect and manage objects in a simulated Martian environment. The system uses ROS (Robot Operating System) and includes functionalities for object detection, trajectory tracking, and precision landing.

### Features
- **Object Detection:** Utilizes YOLO and ArUco markers for detecting objects in the environment.
- **Trajectory Tracking:** Implement pure pursuit alogrithm.
- **Precision Landing:** Use of PX4 precision landing mode to land on detected objects.
- **RealSense Integration:** Supports RealSense cameras for real-world applications.
- **RVIZ Visualization:** Real-time insights of the mission and the environment.
- **Mission Control:** State machine to control flow of different nodes.
- **ROS Integration:** Fully integrated with ROS for communication and control.

## Docker
Build martian mines system image (optional) - if the newest image is not avalible on docker hub or you introduced some modifications in Dockerfile. 
```bash
docker build -f docker/Dockerfile-minimal-intel-ros -t highflyers/martian-minimal-intel-ros .
```
### Run - with main simulation
Run uav_simulation container according to a readme from repo [uav_simulation](https://github.com/High-Flyers/uav_simulation)

Run the container with martian mines main system:

```bash
    # replace <path_to_repo> with the absolute path to your repo, for example: /home/user/Documents/repos/martian-mines-object-detection
docker run --privileged --rm --gpus all -it --net host --ipc host \                  
    -e DISPLAY=${DISPLAY} \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e ROS_DOMAIN_ID=0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v <path_to_repo>:/home/docker/ws/src/martian-mines \
    highflyers/martian-minimal-intel-ros /bin/bash
```
Build ros workspace and our package:
```
source src/martian-mines/scripts/setup.sh

```
Run our figure_finder - it should show a preview of video from simulation, with marked detected objects.
```
roslaunch martian-mines figure_finder.launch 
```

### Run container dedicated for Xavier
#### Emulated ARM container on x86

```bash
docker run --platform=linux/arm64 -it --net host -v <path_to_repo>:/home/user/ws/src/martian-mines highflyers/martian-mines-jetson:realsense
```

#### Run compose on jetson (onetime container which will be removed after exit):
```bash
cd docker
sudo docker compose run martian-mines /bin/bash  
```

#### Run tmux sesion on jetson, with persistent docker compose and mavlink router
```bash
sudo ./scripts/tmux_jetson.sh
```


#### Build ros workspace and run figure_finder
```bash
catkin build
source devel/setup.bash
roslaunch martian-mines realsense.launch # to run only realsense
roslaunch martian-mines figure_finder.launch real_world:=true # to run figure_finder with realworld config
```
#### Setup scrip can be used build workspace and optionally setup IP adresses to allow ros communication between two computers
```bash
# only build and source
source src/martian-mines/scripts/setup.sh
# for setup ip for master
source src/martian-mines/scripts/setup.sh master <master_ip>
# for setup ip for client
source src/martian-mines/scripts/setup.sh client <master_ip> <client_ip>
```
