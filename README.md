# martian-mines
Our system for 2024 Droniada Challenge competition Martian Mines category.

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
Run our detector - it should show a preview of video from simulation, with marked detected objects.
```
roslaunch martian-mines detector.launch 
```

### Run container dedicated for Xavier
#### Emulated ARM container on x86

```bash
docker run --platform=linux/arm64 -it --net host -v <path_to_repo>:/home/user/ws/src/martian-mines highflyers/martian-mines-jetson:realsense
```

#### Run compose on jetson:
```
cd docker
sudo docker compose run martian-mines /bin/bash
```

#### Build ros workspace and run detector
```bash
catkin build
source devel/setup.bash
roslaunch martian-mines realsense.launch # to run only realsense
roslaunch martian-mines detector.launch real_world:=true # to run detector with realworld config
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