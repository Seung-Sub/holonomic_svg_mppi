
# Stein Variational Guided Model Predictive Path Integral Control (SVG-MPPI)

Based on [Stein Variational Guided Model Predictive Path Integral Control: Proposal and Experiments with Fast Maneuvering Vehicles](https://arxiv.org/abs/2309.11040), modified to a holonomic-based code

## Tested Environment

- Ubuntu Focal 20.04 (LTS)
- ROS Noetic

<details>
<summary>Basic Instllation</summary>

## Install ROS noetic
[Installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu)

```bash
# Set up your computer to accept software from packages.ros.org
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up your keys
sudo apt install -y curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update

# install ROS
sudo apt install -y ros-noetic-desktop-full

# Install other tools 
sudo apt install python3-osrf-pycommon python3-catkin-tools
```

## Install Docker
[Installation guide](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)

```bash
# Install from get.docker.com
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo groupadd docker
sudo usermod -aG docker $USER
```

</details>

## Install Dependencies

```bash
cd proj-svg_mppi
make setup
```

## Build Controllers

```bash
cd proj-svg_mppi
make build
```

# Since it is holonomic now, you can follow simulation(2)

## Run Simulation with Visualization(1)
**(This for Ackermann based simulation with f1tenth_gym)**

Launch simulator in the Docker container
```bash
cd proj-svg_mppi
./script/launch_simulator.sh
```

Launch controllers in another terminal
```bash
cd proj-svg_mppi
./script/launch_controllers.sh 
```

You can change the all MPPI parameters and settings in [the yaml file](./src/mppi_controller/config/mppi_controller.yaml)


## Run Simulation with Visualization(2)
**(Simulation in progress to control holonomics with Summit_xl_sim)**

For the simulation environment, please clone and set up the following link:

(https://github.com/RobotnikAutomation/summit_xl_sim/tree/noetic-devel)

Launch simulator
```bash
cd /your_ws(summit_xl_sim)
source ~/your_ws/devel/setup.bash
roslaunch summit_xl_sim_bringup summit_xls_complete.launch
```

launch mppi (local-planner)
```bash
cd /proj-svg_mppi
source ~/pro-svg_mppi/devel/setup.bash
roslaunch mppi_controller mppi_controller.launch is_simulation:=true # is_simulation:=true for launch trigger
roslaunch local_costmap_generator local_costmap_generator.launch

rostopic pub /mppi/start std_msgs/Empty -1  # for start mppi
```

**you can choose mppi-mode at mppi_controller.yaml**
**Currently only forward_mppi and svg_mppi are available.**
