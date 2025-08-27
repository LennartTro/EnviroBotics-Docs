---
id: start simulation
title: Starting simulation
---

# Starting the BlueBoat Simulation

This guide explains how to simulate the BlueBoat using the [gazebosim_blueboat_ardupilot_sitl](https://github.com/markusbuchholz/gazebosim_blueboat_ardupilot_sitl) repository. The Docker container setup and build steps are described previously — this section starts with launching the container.

---

## Host Requirements

Make sure you have a working Docker installation and QGroundControl for mission planning and visual feedback.

---

## Step 1 – Run the Docker Container

After cloning the simulation repository and building the Docker image, you can start the container.


Navigate into the Docker folder in the 
and adjust these lines in run.sh:

```bash
local_gz_ws="/home/Lennart/blueboat_ardupilot_SITL/gz_ws"
local_SITL_Models="/home/Lennart/blueboat_ardupilot_SITL/SITL_Models"
```
to your specific path:
```bash
local_gz_ws="/YOURPATH/Summer_School/blueboat_ardupilot_SITL/gz_ws"
local_SITL_Models="/YOURPATH/Summer_School/blueboat_ardupilot_SITL/SITL_Models"
```

Now open a terminal and run the container:
#### 1. Go to the Docker folder:
```bash
cd gazebosim_blueboat_ardupilot_sitl/blueboat_sitl/docker
```
#### 2. Start the container:
```bash
sudo ./run.sh
```

This will launch an Ubuntu-based Docker container with all required dependencies and libraries for running the BlueBoat simulation.

Once inside the container, you’ll have access to all simulation and ROS 2 tools pre-installed.

---

## Step 2 – Build the ROS 2 Workspaces

Once you're inside the Docker container, you need to build the ROS 2 workspaces that contain the packages for simulation, BlueBoat control, and visualization.

### 1. Source ROS 2

Before building, make sure the ROS 2 environment is sourced:

```bash
source /opt/ros/humble/setup.bash
```

### 2. Build BlueBoat Workspace

Navigate to the BlueBoat workspace and build it:

```bash
colcon build
```

Source the installed file:
```bash
source install/setup.bash
```

### 3. Change the directory
Go to the directory "gz_ws". (The pre command "../" will go one file level up)
```bash
cd ../gz_ws
```
### 4. Build this workspace too:
```bash
colcon build --symlink-install --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON -DCMAKE_CXX_STANDARD=17
```
and source everything:

```bash
source install/setup.bash

source gazebo_exports.sh
```

---

## Step 3 – Start Start QGroundControl (outside Docker)

Just click on the icon

---

## Step 4 - Run GazeboSim inside Docker
Start a new terminal and enter the running docker container
```bash
sudo docker exec -it blueboat_sitl /bin/bash
```
Chang to gz-workspace
```bash
cd ../gz_ws
```

And launch the gazebo simulation:
```bash
ros2 launch move_blueboat launch_robot_simulation.launch.py
```

---

## Step 5 - Run SITL (Software in the Loop)

Start a new terminal and enter the running docker container (doesn't matter where do you open your terminal): 
```bash
sudo docker exec -it blueboat_sitl /bin/bash
```
Chang to ardupilot directory:
```bash
cd ../ardupilot
```
Run:
```bash
sim_vehicle.py -v Rover -f gazebo-rover --model JSON --map --console -l 48.214611,11.720278,0,0
```

---

## Summary

You're now ready to explore the BlueBoat simulation. You can modify ROS 2 nodes, interact with sensors, plan missions, and experiment with algorithms in a realistic simulated environment.