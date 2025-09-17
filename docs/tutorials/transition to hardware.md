---
id: transition to hardware
title: Transition to Hardware
---

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

ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'MANUAL'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

```

```bash

ros2 topic pub -r 10 /mavros/rc/override mavros_msgs/msg/OverrideRCIn "{channels: [1600, 0, 1600, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"

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