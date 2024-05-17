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
    -v <path_to_repo>/ros:/home/docker/ws/src/ \
    highflyers/martian-minimal-intel-ros /bin/bash
```
Build ros workspace and source setup script
```
# inside ROS workspace
catkin build
source devel/setup.bash
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
#### Running on Xavier with realsense connected (to be tested)
```bash
docker run -it --net host --privileged --runtime nvidia -v /dev/bus/usb/:/dev/bus/usb/ -v <path_to_repo>:/home/user/ws/src/martian-mines highflyers/martian-mines-jetson:realsense
```

#### Better: run compose on jetson
```
sudo docker compose run martian-mines /bin/bash
```

#### Build ros workspace and run detector
```bash
catkin_make
source devel/setup.bash
roslaunch martian-mines realsense.launch # to run only realsense
roslaunch martian-mines detector.launch real_world:=true # to run detector with realsense
```
