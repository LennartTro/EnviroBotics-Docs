---
id: build ROS nodes
title: Build your own ROS nodes
---


# Writing custom ROS 2 nodes

To control the BlueBoat in the simulation (e.g., moving it to a certain position), you’ll typically write **custom ROS 2 nodes**. These nodes allow you to subscribe to sensor data, process it (e.g., with a control algorithm), and publish commands to actuators.

---

## 1 What is a ROS 2 node?

A **ROS 2 node** is a single, executable program in a distributed robotic system that communicates with other nodes using topics, services, or actions.

Each node can:
- Subscribe to topics (receive data),
- Publish to topics (send data),
- Provide services (blocking calls),
- Use parameters (for tuning),
- And more.

Nodes are usually written in **Python** or **C++**. For simplicity, we will use **Python** in our Summer School.

---
## 2 Build your own nodes
### 1 Structure of a Simple Node (Python)

Below is a minimal ROS 2 node in Python that logs a message once per second.

```python
# my_node.py
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Hello from my_node!')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

```

To run this, place the file inside a ROS 2 Python package.

---

### 2 Creating a ROS 2 Python Package

To create your own package for BlueBoat control, go to the src Directory:
```bash

cd ~/ros2_ws/src

```

and run:

```bash

ros2 pkg create --build-type ament_python blueboat_controller

```

Then add your Python node (e.g., pid_controller.py) to the blueboat_controller/blueboat_controller directory and update setup.py to install it.

---

### 3 Building and Running your node
Build:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```
and run:
```bash

ros2 run blueboat_controller pid_controller

```
This will start your custom node.

### 4 Customize your node:
``` python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from pymavlink import mavutil
import math
import time


class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def compute(self, error):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        return self.kp * error + self.ki * self.integral + self.kd * derivative


class SimplePIDNode(Node):
    def __init__(self):
        super().__init__('simple_blueboat_pid')

        self.goal = [3.0, 2.0]  # Feste Zielkoordinate

        # PID-Regler
        self.heading_pid = PID(3.0, 0.0, 0.3)
        self.speed_pid = PID(0.2, 0.0, 0.5)

        # MAVLink Verbindung aufbauen
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.master.wait_heartbeat()
        self.get_logger().info("Verbunden mit MAVLink")

        # ROS Subscriber
        self.create_subscription(Odometry, '/model/blueboat/odometry', self.odom_callback, 10)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        yaw = self.get_yaw(msg.pose.pose.orientation)

        dx = self.goal[0] - pos.x
        dy = self.goal[1] - pos.y
        distance = math.hypot(dx, dy)

        target_angle = math.atan2(dy, dx)
        heading_error = (target_angle - yaw + math.pi) % (2 * math.pi) - math.pi

        steer_cmd = self.heading_pid.compute(heading_error)
        speed_cmd = self.speed_pid.compute(distance)

        # Ziel erreicht?
        if distance < 0.3:
            speed_cmd = -0.2  # leicht rückwärts bremsen

        pwm_left, pwm_right = self.compute_pwm(speed_cmd, steer_cmd)
        self.send_pwm(pwm_left, pwm_right)

        self.get_logger().info(f"Distanz: {distance:.2f} m | Heading Error: {math.degrees(heading_error):.1f}°")

    def compute_pwm(self, speed_cmd, steer_cmd):
        base_pwm = 1500
        power = int(400 * max(min(speed_cmd, 1.0), -1.0))
        turn = int(400 * max(min(steer_cmd, 1.0), -1.0))

        pwm_left = max(min(base_pwm - power - turn, 1900), 1100)
        pwm_right = max(min(base_pwm - power + turn, 1900), 1100)
        return pwm_left, pwm_right

    def send_pwm(self, left, right):
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            left, 0, right, 0,
            0, 0, 0, 0
        )

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = SimplePIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

```

<details>
<summary>Explanation of the simple BlueBoat PID Node</summary>

This minimal ROS 2 node drives the simulated BlueBoat toward a **fixed target position** using a basic PID controller for both heading and speed.

---

### 🚦 What this Node Does

- Subscribes to the boat's **odometry** in simulation
- Computes the **distance** and **heading angle** to the fixed target
- Uses **PID control** to calculate motor signals
- Sends **PWM commands via MAVLink** to steer and drive the boat

---

### 🧠 Core Concepts

| Component | Purpose |
|----------|---------|
| `PID` class | Calculates control output using proportional, integral, and derivative parts |
| `odom_callback()` | Reacts to new position data from Gazebo |
| `compute_pwm()` | Converts speed and steering into left/right motor commands |
| `send_pwm()` | Sends motor signals directly via MAVLink |
| `get_yaw()` | Extracts boat orientation from quaternion |

---

### 🔁 Control Logic

1. Compute distance and heading error to the target  
2. Use two PID controllers:
   - One for **heading correction**
   - One for **speed adjustment**
3. Translate results into PWM values
4. Command motors via MAVLink

---

### 🎯 Fixed Goal

The goal is defined statically in the code:

```python
self.goal = [3.0, 2.0]
```
</details>

---

# Next Steps