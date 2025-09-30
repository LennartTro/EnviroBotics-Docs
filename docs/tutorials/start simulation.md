---
id: start simulation
title: Starting simulation
---

# Starting the BlueBoat Simulation

This guide explains how to simulate the BlueBoat using the [gazebosim_blueboat_ardupilot_sitl](https://github.com/tum-erl/gazebosim_blueboat_ardupilot_sitl) repository. The Docker container setup and build steps are described previously — this section starts with launching the container.

---

## Host Requirements

Make sure you have a working Docker installation and QGroundControl for mission planning and visual feedback.

---

## Step 1 – Run the Docker Container

After cloning the simulation repository and building the Docker image, you can start the container.


But first navigate into the Docker folder:

```text {6}
home/
├── Summer_School
│   |── gazebosim_blueboat_ardupilot_sitl/
|   |       |── blueboat_sitl/
|   |       |       |── docker
|   |       |       |     |──  run.sh
|   |       |       |     |──  build.sh
|   |       |       |     └──  Dockerfile
|   |       |       └──...
|   |       └──...
|   └──...  
└──...
```
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
(You can open it with Visual Studio Code, for example, to adjust the lines)

### Now open a terminal to run the container:
#### 1. Go to the Docker folder:
```bash
cd ./gazebosim_blueboat_ardupilot_sitl/blueboat_sitl/docker
```
#### 2. Start the container:
```bash
sudo ./run.sh
```

This will launch an ubuntu-based Docker container with all required dependencies and libraries for running the BlueBoat simulation.

Once inside the container, you’ll have access to all simulation and ROS 2 tools pre-installed. That's how it looks like:


<img src="/EnviroBotics-Docs/img/Docker_container.png" alt="start simulation" style={{ width: '80%', marginBottom: '1rem' }} />


---

## Step 2 – Build the ROS 2 Workspaces

Once you're inside the Docker container, you need to build the ROS 2 workspaces that contain the packages for simulation, BlueBoat control, and visualization.


### 1. Build BlueBoat Workspace

Navigate to the folder colcon_ws and build it: 

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
(If you get any problems during the build-process - don't build the workspace. The files may already be there.)

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

## Step 3 – Start QGroundControl (outside Docker)

Just click on the icon in your QGroundControl folder. (You do not need the terminal here)

---

## Step 4 - Run GazeboSim inside Docker
Start a new terminal and enter the running docker container. It doesn't matter where you open your terminal - with that command you will jump into the container you started before: 
```bash
sudo docker exec -it blueboat_sitl /bin/bash
```
Chang to ardupilot directory:
```bash
cd ../gz_ws
```
(or go back to your previous terminal with the docker container) and launch the gazebo simulation:
```bash
ros2 launch move_blueboat launch_robot_simulation.launch.py
```
But not without sourcing everything in the first place ;-):
```bash
source install/setup.bash
source gazebo_exports.sh
```

Now you should see the Gazebo simulation with some physical models for water and our BlueBoat. If you look exactly you will see how the Boat is moving. To reduce the requirements for graphics we replaced the rendered water surface with a flat plane. However, a wave model runs in the background.
<img src="/EnviroBotics-Docs/img/Gazebo.png" alt="Concept Overview" style={{ width: '100%', marginBottom: '1rem' }} />

---

## Step 5 - Run SITL (Software in the Loop)

Start a new terminal and enter the running docker container. It doesn't matter where you open your terminal: 
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
After a few seconds your QGroundCrontrol is connected to a "virtual" BlueBoat swimming on Speichersee. That is were the GPS Position 48.214611 N and 11.720278 W leads us.

<img src="/EnviroBotics-Docs/img/Ardupilot_Speichersee.png" alt="starting simulation" style={{ width: '100%', marginBottom: '1rem' }} />


<details>

<summary>Click to expand the explanation of SITL and HITL</summary>

Software-in-the-Loop (SITL)

**SITL** stands for Software in the Loop. It allows you to run the full autopilot software (e.g. ArduPilot) on your PC, without any physical hardware involved. The software behaves as if it were running on a real flight controller. Instead of receiving sensor data from real IMUs or GPS, it receives simulated sensor data (e.g. from Gazebo). Instead of sending real motor signals, it sends them to the simulated motors.

Benefits of SITL:

- No hardware needed — fast and easy testing
- Safer than real-world testing
- Reproducible and scriptable
- Ideal for early development

In SITL, you can test complete missions and control strategies — just like in the real system — but from your laptop.

---

Hardware-in-the-Loop (HITL) - just for your information

**HITL** stands for Hardware in the Loop. Here, the real autopilot hardware (e.g. a Pixhawk) is used, but connected to a simulated environment.
The physical flight controller receives simulated sensor data. The motor outputs are processed as if they were sent to real actuators — but are intercepted by the simulation.

Benefits of HITL:

- Tests real hardware behavior
- Helps identify hardware-related issues (e.g. timing, I/O)
- Great final step before deploying in the real world


</details>

---

## Step 5 - Set up QGroundControl for your Boat

Make sure, that the behavior of your simulated Boat is similar to a real BlueBoat  


<div style={{ display: 'flex', gap: '1rem', justifyContent: 'space-between', flexWrap: 'wrap' }}>

  <div style={{ flex: '1 1 48%' }}>
    <h4>1. Click on the Q in the left upper corner:</h4>
    <img src="/EnviroBotics-Docs/img/QGC_1.png" alt="QGC Step 1" style={{ width: '100%', borderRadius: '8px' }} />
  </div>

  <div style={{ flex: '1 1 48%' }}>
    <h4>2. Go to Frame and select Boat instead of Rover:</h4>
    <img src="/EnviroBotics-Docs/img/QGC_3.png"  alt="QGC Step 2" style={{ width: '100%', borderRadius: '8px' }} />
  </div>

</div>


#### 4. Go further to Parameters and search for steer_type:
Choose "Two Paddles Input" in the Parameter Editor and save. This is important because the BlueBoat steers with its two propellers and not with a fin. 

<img src="/EnviroBotics-Docs/img/QGC_4.png" alt="starting simulation" style={{ width: '100%', marginBottom: '1rem' }} />

Go back by clicking on the left upper corner.

## Summary

You're now ready to explore the BlueBoat simulation. You can modify ROS 2 nodes, interact with sensors, plan missions, and experiment with algorithms in a realistic simulated environment - that is all part of the following section.