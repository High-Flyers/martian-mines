# Martian Mines ROS2

### Requirements
- **Docker** (Docker CLI recommended)
  - Add your user to the Docker group to grant the necessary permissions; otherwise, you must prefix commands with sudo.
  
- **Visual Studio Code** (recommended) with the following extensions:
  - [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker)
  - [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### Docker build
```bash
docker build -t highflyers/martian_mines_ros2:latest . --build-arg USER_UID=$(id -u)
```

### Run for development
Running docker with GPU support requires [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), but if you don't have NVIDIA GPU, you can remove the `--gpus=all` flag in `devcontainer.json` and run this container only with CPU.

Open repository in vscode:
```bash
cd /path/to/martian_mines_ros2/
code .
```
Click `Ctrl+Shift+P` and select `Dev Containers: Rebuild and Reopen in Container`. 
This will open the repository in the container and you can start developing.

To rebuild workspace use shortcut `Ctrl+Shift+B` in the vscode.

To execute offboard example, open terminal in vscode and run:
```bash
ros2 run offboard offboard_example
```
### TODO  List
- Nodes
  - [x] bbox_publisher
  - [x] detection
  - [x] detection visualization
  - [ ] environment_visualization
  - [ ] figure_finder
  - [ ] precision_landing
  - [ ] tf_start_pose
  - [ ] trajectory_generator
  - [ ] trajectory_tracker


- Other classes/files etc
  - [ ] detector package
    - [x] aruco detector
    - [ ] yolo detector


- NOT tested yet
  - color_detection
  - data_collector
  - ...
### Throubleshoting
If you will encounter a problem with "xcb" (GUI applications won't open) just run `xhost +local:docker` on your host machine.
