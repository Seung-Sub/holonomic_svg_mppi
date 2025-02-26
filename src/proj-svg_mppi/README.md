
# Stein Variational Guided Model Predictive Path Integral Control (SVG-MPPI)

Based on [Stein Variational Guided Model Predictive Path Integral Control: Proposal and Experiments with Fast Maneuvering Vehicles](https://arxiv.org/abs/2309.11040), (https://github.com/kohonda/proj-svg_mppi) I modified to a holonomic-based code

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
# or you can do clone in your workspace and catkin_make
cd $your_workspace
catkin_make
```


## Run Simulation with Visualization
**(Simulation in progress to control holonomics with Summit_xl_sim)**

For the simulation environment, please clone and set up the following link:

(https://github.com/RobotnikAutomation/summit_xl_sim/tree/noetic-devel)

And you should change the topic in simulation's local planner
go to /summit_xl_common/summit_xl_navigation/launch/move_base_teb.launch
and change cmd_vel_topic's default (whatever you like, just to avoid moving with existing local planner)

Launch simulator
```bash
cd /your_ws(summit_xl_sim)
source ~/your_ws/devel/setup.bash
roslaunch summit_xl_sim_bringup summit_xls_complete.launch
```

launch mppi (local-planner)
```bash
cd /$your_ws(holonomic_svg_mppi)
source ~/$your_ws/devel/setup.bash
roslaunch mppi_controller mppi_controller.launch is_simulation:=true # is_simulation:=true for launch trigger
roslaunch local_costmap_generator local_costmap_generator.launch

```

**you can choose mppi-mode at mppi_controller.yaml**

**Currently only forward_mppi and svg_mppi are available.**

**is_reference_less_mode: true = reference mode, false = Target point mode // less is a typo**


## Debug (changing guide_samples sampling)
After completing the simulation operation, check the plot related to each SVGD operation.

If comment // For debug (1) is enabled, guide_samples are sampled from the initial distribution, If // For debug (2) is enabled, guide_samples are sampled from the distribution for previous svgd results.

```bash
cd your_ws/src/proj-svg_mppi/script
python3 svgd_gradient_plot.py
python3 svgd_cost_log.py
python3 svgd_best_cov_plot.py
```
