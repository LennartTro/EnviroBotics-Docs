---
id: build ROS nodes
title: Build your own ROS nodes
---


# Writing custom ROS 2 nodes

To control the BlueBoat in the simulation (e.g., moving it to a certain position), you’ll typically write **custom ROS 2 nodes**. These nodes allow you to subscribe to sensor data, process it (e.g., with a control algorithm), and publish commands to actuators.

---
## What is what?
### What is a ROS 2 node?

A **ROS 2 node** is a single, executable program in a distributed robotic system that communicates with other nodes using topics, services, or actions.

Each node can:
- Subscribe to topics (receive data),
- Publish to topics (send data),
- Provide services (blocking calls),
- Use parameters (for tuning),
- And more.

Nodes are usually written in **Python** or **C++**. For simplicity, we will use **Python** in our Summer School.


### What is a ROS 2 Package?

A ROS 2 package is the basic unit of software in the ROS 2 ecosystem. It acts as a container for related code, such as:
- One or more nodes
- Message or service definitions
- Configuration files (e.g., launch, params)
- Metadata (setup.py, package.xml)

Think of it as a folder that bundles everything you need to build, run, and share a ROS-based component. A node needs to be **inside a package** to be usable with ROS tools like `ros2 run`.

### How does a package relate to nodes?

A node is an executable Python (or C++) script.

A package provides the environment to organize, build, and install that node.

When you run ros2 run my_package my_node, ROS looks inside the package to find and execute the node.

---

## 2 Step-by-Step: From Python File to Executable Node

### Step 1 – Create a Python Package

Open a terminal in your workspace's `src` folder - we are working inside the Docker container in `gz_ws/` folder:

```text {8}
home/
├── Summer_School
│   |── gazebosim_blueboat_ardupilot_sitl/
|   |       |── blueboat_sitl/
|   |       |       |── ....
|   |       |       └── ...
|   |       |── gz_ws/
|   |       |    |── src/
|   |       |    |    |── "NODES"
|   |       |    |    └── ...
|   |       |    |── (install/)
|   |       |    |── (build/)
|   |       |    └── ...
|   |       |──  SITL_Models/
|   |       └── README.md
|   └── ...
└── ...
```
You can use this command to go there:

```bash
cd ~/gz_ws/src
```
Here you can create a ne Package with the name `blueboat_controller`:

```bash
ros2 pkg create --build-type ament_python blueboat_controller
```



That will create the package structure inside you `src/`-folder:

```text
blueboat_controller/
├── blueboat_controller/
|       └── __init__.py 
├── package.xml
├── setup.py
└── resource/

```
### Step 2 – Add Your Node Script

Create your own Python file in the `blueboat_controller/` directory:
```bash
touch blueboat_controller/blueboat_controller/pid_controller.py
```
Paste your ROS 2 node code here (e.g., a simple PID controller).
Make sure the first line of the script is:
```python
#!/usr/bin/env python3
```
Also, don’t forget to make the script executable:
```bash
chmod +x blueboat_controller/blueboat_controller/pid_controller.py
```
Or do an right-click on the file pid_controller.py --> Properties....

### Step 3 – Update setup.py

Open the generated setup.py and modify it like this:
```python {20}
from setuptools import setup

package_name = 'blueboat_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Custom controller node for BlueBoat',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pid_controller = blueboat_controller.pid_controller:main',
        ],
    },
)

```
The entry in console_scripts allows ROS 2 to run the node with:

```bash
ros2 run blueboat_controller pid_controller
```

But first build...

### Step 4 – Building and Running your node
Build:
```bash
cd ~/gz_ws
colcon build --symlink-install --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON -DCMAKE_CXX_STANDARD=17"
```
After building, always source your workspace:
```bash
source install/setup.bash
```

and run:
```bash

ros2 run blueboat_controller pid_controller

```
This will start your custom node. In this case all it does is...

### Customize your node:
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
        self.last_time = None

    def update(self, error):
        now = time.time()
        dt = now - self.last_time if self.last_time else 0.1
        self.last_time = now

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        return self.kp * error + self.ki * self.integral + self.kd * derivative


class BlueboatPIDNode(Node):
    def __init__(self):
        super().__init__('blueboat_pid_regler')

        # Festes Ziel (XY)
        self.goal = [0.0, 0.0]

        # PID-Regler
        self.heading_pid = PID(kp=3.0, ki=0.0, kd=0.4)
        self.speed_pid = PID(kp=0.1, ki=0.0, kd=0.8)

        # MAVLink-Verbindung
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.master.wait_heartbeat()
        self.get_logger().info("Mit MAVLink verbunden - Modus: XY, Ziel=(0,0)")

        # Odometrie abonnieren
        self.create_subscription(Odometry, '/model/blueboat/odometry', self.odom_callback, 10)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        yaw = self.get_yaw(msg.pose.pose.orientation)

        distance, heading_error = self.compute_control_errors(pos.x, pos.y, yaw)

        speed_cmd = self.compute_speed_cmd(distance, heading_error)
        steer_cmd = self.compute_steer_cmd(heading_error)

        pwm_left, pwm_right = self.compute_motor_pwms(speed_cmd, steer_cmd)
        self.send_pwm(pwm_left, pwm_right)

        # Kompaktes Logging
        self.get_logger().info(
            f"XY: x={pos.x:.2f}, y={pos.y:.2f}, yaw={math.degrees(yaw):.1f}° | "
            f"Distanz: {distance:.2f} m | Heading-Error: {math.degrees(heading_error):.1f}° | "
            f"PWM L/R: {pwm_left}/{pwm_right}"
        )

    def compute_control_errors(self, x, y, yaw):
        dx = self.goal[0] - x
        dy = self.goal[1] - y
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx) if distance > 1e-6 else yaw
        heading_error = (target_angle - yaw + math.pi) % (2 * math.pi) - math.pi
        return distance, heading_error

    def compute_speed_cmd(self, distance, heading_error):
        if distance < 0.1:
            self.speed_pid.integral = 0.0
            return -0.2  # leicht rückwärts zum Bremsen
        if distance > 0.3 and abs(heading_error) > math.radians(90):
            return 0.0  # auf der Stelle drehen
        return max(min(-self.speed_pid.update(distance), 1.0), -0.5)

    def compute_steer_cmd(self, heading_error):
        steer = self.heading_pid.update(heading_error)
        return max(min(steer, 1.0), -1.0)

    def compute_motor_pwms(self, speed_cmd, steer_cmd):
        base_pwm = 1500
        power = int(400 * speed_cmd)
        turn = int(400 * steer_cmd)
        left = max(min(base_pwm - power - turn, 1900), 1100)
        right = max(min(base_pwm - power + turn, 1900), 1100)
        return left, right

    def send_pwm(self, pwm_left, pwm_right):
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            pwm_left, 0, pwm_right, 0,
            0, 0, 0, 0
        )

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = BlueboatPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


```
### And build again: 

Build:
```bash
cd ~/gz_ws
colcon build --symlink-install --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON -DCMAKE_CXX_STANDARD=17"
```
After building, always source your workspace:
```bash
source install/setup.bash
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

### See what your node does 

Before starting your ROS 2 node, make sure that the vehicle is armed in QGroundControl. Otherwise, your BlueBoat won’t respond to motor commands, even though your node is running correctly.

✅ Arming the Vehicle in QGroundControl

1 Open QGroundControl (QGC)

2 Ensure it is connected to the simulated vehicle via MAVLink (Ready to Fly in the top bar) - click on "Ready to Fly"

3 Click the "Arm" button (top left)

3 Slide or hold the spacebar to confirm arming

<div style={{ display: 'flex', gap: '1rem', justifyContent: 'space-between', flexWrap: 'wrap' }}> <div style={{ flex: '1 1 48%' }}> <img src="/EnviroBotics-Docs/img/Arming2.png" alt="QGroundControl arm slider" style={{ width: '100%', borderRadius: '8px' }} /> </div> <div style={{ flex: '1 1 48%' }}> <img src="/EnviroBotics-Docs/img/Arming1.png" alt="QGroundControl armed status" style={{ width: '100%', borderRadius: '8px' }} /> </div> </div>

Once the vehicle is armed, run your Terminal and controller node:

With your new customized node, your BlueBoat is able to navigate to the goal ([3.0 , 2.0]) that you defined.
To see what happens, watch your Gazebo simulation and run you node:

```bash
ros2 run blueboat_controller pid_controller

```
The Boat is approaching the destination and you get an output of values for distance and heading error in your terminal.

<div style={{ display: 'flex', gap: '1rem', justifyContent: 'space-between', flexWrap: 'wrap' }}>

  <div style={{ flex: '1 1 48%' }}>
    <h4>Gazebo simulation:</h4>
    <img src="/EnviroBotics-Docs/img/Gazebo.png" alt="gazebo" style={{ width: '100%', borderRadius: '8px' }} />
  </div>

  <div style={{ flex: '1 1 48%' }}>
    <h4>Terminal:</h4>
    <img src="/EnviroBotics-Docs/img/Terminal.png"  alt="terminal" style={{ width: '100%', borderRadius: '8px' }} />
  </div>
</div>

---


# Next Steps
Your BlueBoat can now navigate to a fixed destination — great progress!

But in a real-world scenario, we often want more:

**Multiple waypoints, automated actions at each stop, and full mission control.**

To make this possible, we’ll build a second node — a planner — that manages goal sequences and coordinates with your controller.