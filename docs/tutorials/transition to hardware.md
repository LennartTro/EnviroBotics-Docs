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

## Required (but actually already installed - so skippable): MAVROS

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
## Connect to the Vehicle

Start the mavros node and connect to the boat using UDP:
```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://@192.168.2.2:14600 -p system_id:=255 -p component_id:=190 -p tgt_system:=1 -p tgt_component:=1
```

<details>
<summary> What is <code>udp://</code> and why do we use UDP?</summary>
What is UDP?

UDP (User Datagram Protocol) is a fast and lightweight network protocol used for sending data between systems on the same network.

In your project, we use it to connect the MAVROS node (running on your computer) to the ArduPilot firmware (running on the boat) using MAVLink.

### Why UDP is used in MAVLink communication

✅ Low latency — data is sent without delay

✅ Simple protocol — minimal overhead

❌ No delivery guarantees — packets may be lost or arrive out of order

This is acceptable in robotics, where fresh data is more important than guaranteed delivery.
</details>


## Arm the Vehicle

Before the boat can move, you need to switch it to MANUAL mode and arm it:
```bash

ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'MANUAL'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

```

## Test Motors Manually

Move the motors forward

```bash
ros2 topic pub -r 10 /mavros/rc/override mavros_msgs/msg/OverrideRCIn "{channels: [1600, 0, 1600, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"
```
Stop all motor movement

```bash
ros2 topic pub -r 10 /mavros/rc/override mavros_msgs/msg/OverrideRCIn "{channels: [1500, 0, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"
```

## Run the Controller Node
Make sure the blueboat_control package is installed on your system and sourced via:

```bash
source ~/gz_ws/install/setup.bash
```

Now launch your real-time controller node for the BlueBoat:
```bash
ros2 run blueboat_control asv_pid_rc   --ros-args -p arrival_radius_m:=1.5              -p invert_throttle:=false              -p invert_steer:=false

```

## Send Target Waypoint

You can now send the boat to a specific location:

```bash
ros2 topic pub -r 10 /asv/target geometry_msgs/msg/Point "{x: 48.28407529304395, y: 11.605825035798238, z: 0.0}"

```
## Stop the Mission

If something goes wrong or you want to stop movement immediately, publish a stop signal:

```bash

ros2 topic pub /asv/stop std_msgs/msg/Bool "data: true"

```
## Your controller is now live on the real BlueBoat hardware!

Continue refining your system and testing it safely in controlled environments before heading out to the lake.

```bash
```

``` python
```

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

```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://@192.168.2.2:14600 -p system_id:=255 -p component_id:=190 -p tgt_system:=1 -p tgt_component:=1

```

```bash

ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'MANUAL'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

```

```bash

ros2 topic pub -r 10 /mavros/rc/override mavros_msgs/msg/OverrideRCIn "{channels: [1600, 0, 1600, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"

```

```bash

ros2 topic pub -r 10 /asv/target geometry_msgs/msg/Point "{x: 48.28407529304395, y: 11.605825035798238, z: 0.0}"


```

```bash

ros2 topic pub -r 10 /mavros/rc/override mavros_msgs/msg/OverrideRCIn "{channels: [1500, 0, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"

```

```bash

ros2 run blueboat_control asv_pid_rc   --ros-args -p arrival_radius_m:=1.5              -p invert_throttle:=false              -p invert_steer:=false

```

```bash

ros2 topic pub /asv/stop std_msgs/msg/Bool "data: true"

```