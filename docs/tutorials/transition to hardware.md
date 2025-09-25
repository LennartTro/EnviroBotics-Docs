---
id: transition to hardware
title: Transition to Hardware
---


# Transition to Hardware

After testing and developing your BlueBoat controller in simulation, it’s time to bring your setup into the real world. This page walks you through the essential steps for running your controller node on real hardware, including how to connect to the vehicle, arm it, test the motors, and send waypoint commands.

:::tip
The process is similar to what you've done in simulation — just with real data and a real boat now.
:::
---
## Accessing the BlueOS Interface on Your BlueBoat

### Prepare the connection

- Make sure the BlueBoat and BaseStation are powered on and paired.

- Connect the BaseStation to your computer.

 Please also pay attention to the notes and instructions on the official Blue Robotics pages:  
- [BlueBoat Software Setup](https://bluerobotics.com/learn/blueboat-software-setup/)  
- [BlueBoat Operator’s Guide](https://bluerobotics.com/learn/blueboat-operators-guide/)  

### Open the web interface

- Launch your preferred web browser (Chrome, Firefox, Edge, etc.).

- In the address bar, type **192.168.2.2** or **blueos.local**.

### Welcome to BlueOS

- You will see the BlueOS home screen.

- If the page does not load, check that the BaseStation is connected and powered, and that your computer is using the correct network.
- You should see a **Heartbeat** coming in from the vehicle.  
- If no heartbeat is detected, check cables, power supply, and endpoint configuration... or ask.  
---


## Control the BlueBoat via MAVROS

1. Once you have access to the BlueOS interface and the BaseStation is connected:

- Open a terminal on your computer.

- Start the MAVROS node and connect to the boat using UDP:

```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://@192.168.2.2:14600 -p system_id:=255 -p component_id:=190 -p tgt_system:=1 -p tgt_component:=1
```

:::note
Required (but actually already installed - so skippable): MAVROS

Make sure `mavros` and related dependencies are installed on your system:

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras
sudo apt install ros-${ROS_DISTRO}-mavros-msgs
```
```bash

sudo apt install geographiclib-tools
sudo geographiclib-get-geoids egm96-5
sudo geographiclib-get-gravity egm2008
sudo geographiclib-get-magnetic wmm2020

```
:::
- If the connection is successful, MAVROS will begin showing messages from the BlueBoat’s autopilot.


<details>
<summary> What is UPD and why do we use UDP?</summary>
What is UDP?

UDP (User Datagram Protocol) is a fast and lightweight network protocol used for sending data between systems on the same network.

In your project, we use it to connect the MAVROS node (running on your computer) to the ArduPilot firmware (running on the boat) using MAVLink.

### Why UDP is used in MAVLink communication

✅ Low latency — data is sent without delay

✅ Simple protocol — minimal overhead

❌ No delivery guarantees — packets may be lost or arrive out of order

This is acceptable in robotics, where fresh data is more important than guaranteed delivery.
</details>

2. (Recommended) Verify the link:

```bash
ros2 topic echo -n 1 /mavros/state
```
3. For GUIDED/global setpoints, make sure the boat has a GPS fix (≥2D):

``` bash
ros2 topic echo -n 3 /mavros/global_position/raw/fix
```
Latitude/longitude must be non-zero and status.status >= 0.

## Option A — Direct control (RC Override) (works like the simulation; powerful, but not recommended for general use)

1. Arm in MANUAL

Before the boat can move, you need to switch it to MANUAL mode and arm it:
```bash

ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'MANUAL'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

```

2. Quick motor test

Move the motors forward

```bash
ros2 topic pub -r 10 /mavros/rc/override mavros_msgs/msg/OverrideRCIn "{channels: [1600, 0, 1600, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"
```
Stop all motor movement

```bash
ros2 topic pub -r 10 /mavros/rc/override mavros_msgs/msg/OverrideRCIn "{channels: [1500, 0, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"
```

3. Run your controller node

Make sure the blueboat_control package is installed on your system and sourced via:

```bash
source ~/gz_ws/install/setup.bash
```

Now launch your real-time controller node for the BlueBoat:
```bash
ros2 run blueboat_control asv_pid_rc   --ros-args -p arrival_radius_m:=1.5              -p invert_throttle:=false              -p invert_steer:=false

```

4. Send a target waypoint to your controller

You can now send the boat to a specific location:

```bash
ros2 topic pub --once /asv/target geometry_msgs/msg/Point "{x: 48.28407529304395, y: 11.605825035798238, z: 0.0}"

```
5. Emergency stop / resumen

If something goes wrong or you want to stop movement immediately, publish a stop signal:

```bash

ros2 topic pub -r 5 /asv/stop std_msgs/msg/Bool "data: true"

```
to continue, send: 

```bash

ros2 topic pub -r 5 /asv/stop std_msgs/msg/Bool "data: false"

```
:::note 
With RC Override you are bypassing the autopilot regulator. You’re responsible for heading/speed control, failsafes, and tuning.
:::

## Option B — Autopilot GUIDED mode (recommended)

1. Switch to GUIDED and arm
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

2. Send periodic global position setpoints (2 Hz)
```bash
ros2 topic pub -r 2 /mavros/setpoint_raw/global mavros_msgs/msg/GlobalPositionTarget \
"{coordinate_frame: 6,  # MAV_FRAME_GLOBAL_INT
  type_mask: 0b0000111111111000,  # position-only
  latitude: 48.284812,
  longitude: 11.606132,
  altitude: 0.0}"
```
```bash
ros2 topic pub -r 2 /mavros/setpoint_position/global mavros_msgs/msg/GlobalPositionTarget \
"{latitude: 48.284812, longitude: 11.606132, altitude: 0.0}"
```
:::note
Setpoints must be sent repeatedly (e.g., 2 Hz).
:::
## Your controller is now live on the real BlueBoat hardware!

Continue refining your system and testing it safely in controlled environments before heading out to the lake.