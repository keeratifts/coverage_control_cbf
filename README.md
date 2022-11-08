# Adaptive coverage control with CBF

## This repository included
- Multiple RP mouse with laser range (indicating CBF safety range)
- Uniform adaptive coverage control with CBF
- Non-uniform adaptive coverage control with CBF via time-varying density function
- trajectory graph plotting script

## Requirements

- Computer
  - ROS
    - [Noetic Ninjemys](http://wiki.ros.org/noetic/Installation/Ubuntu
  - Gazebo
    - Gazebo 9.x
  - Package
    - [Multiple RaspberryPiMouse robots simulator](https://github.com/keeratifts/Multiple-RaspberryPiMouse-robots-simulator.git)
  - Python
    - Pyhton 3.x


## Installation

Download this package & install all system dependecies
```sh
cd ~/<your_workspace>/src
git clone https://github.com/ryuichiueda/raspimouse_ros_2.git
cd ~/<your_workspace>
catkin_make
source devel/setup.bash
```

## Getting started
First launch the simulator

![raspi_withsensor](https://user-images.githubusercontent.com/76491592/200635709-421dcced-eb3a-4471-b789-2def0510b794.png)

```sh
roslaunch coverage_control_cbf main_9robots.launch 
```

For uniform coverage control

![uniform](https://user-images.githubusercontent.com/76491592/200632960-8e25f0bd-45ff-4e7f-a4e3-36bc6aa02a7e.gif)

```sh
rosrun coverage_control_cbf main_uniform.py
```

For non-uniform coverage control

![non_uniform](https://user-images.githubusercontent.com/76491592/200632968-8d03929d-17f2-42ff-93c3-d2fb9350d6c5.gif)

```sh
rosrun coverage_control_cbf main_non_uniform.py
```

For non-uniform coverage control with real robots

![safe_real](https://user-images.githubusercontent.com/76491592/200639487-cca367aa-c869-4698-bc6c-52df862aad87.gif)

```sh
rosrun coverage_control_cbf main_real.py
```
