---
id: controller node
title: Adjust you controller
---

# Motivation for Updating the Controller

Up to this point, our controller was capable of guiding the BlueBoat to a single fixed target position. While this is a useful starting point, our actual mission requires more complex behavior:

 - The BlueBoat should visit multiple waypoints in sequence.
 - At each waypoint, a specific action should be performed (e.g., measuring water temperature).
 - The vehicle must hold its position reliably during these operations.

To implement this functionality, we introduce a separate planner node that is responsible for:

 - Sending a sequence of target positions to the controller.
 - Waiting until the target is reached.
 - Triggering any desired actions at each waypoint.

This separation of concerns allows the controller to focus purely on navigation and stabilization, while the planner handles mission logic.

``` python 
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from pymavlink import mavutil
from blueboat_interfaces.srv import SetTarget
from std_msgs.msg import Bool
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

        # Zielposition
        self.goal = [0.0, 0.0]  # initiales Ziel
        # PID-Regler initialisieren
        self.heading_pid = PID(kp=3.0, ki=0.0, kd=0.4)
        self.speed_pid = PID(kp=0.1, ki=0.0, kd=0.8)

        # MAVLink-Verbindung
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.master.wait_heartbeat()
        self.get_logger().info("Mit MAVLink verbunden")

        # Odometrie abonnieren
        self.create_subscription(Odometry, '/model/blueboat/odometry', self.odom_callback, 10)
        
        self.target_service = self.create_service(SetTarget, 'set_target', self.set_target_callback)
        # Publisher für Ziel-Erreichung
        self.target_reached_pub = self.create_publisher(Bool, '/target_reached', 10)
        self.goal_reached = False  # merken, ob schon gemeldet

    def set_target_callback(self, request, response):
        self.goal = [request.x, request.y]
        self.get_logger().info(f"Neues Ziel empfangen: x={request.x}, y={request.y}")
        self.goal_reached = False  # zurücksetzen!
        response.accepted = True
        time.sleep(5)
        self.get_logger().info("...abgeschlossen.")
        return response

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        yaw = self.get_yaw(msg.pose.pose.orientation)

        distance, heading_error = self.compute_control_errors(pos.x, pos.y, yaw)
        speed_cmd = self.compute_speed_cmd(distance, heading_error)
        steer_cmd = self.compute_steer_cmd(heading_error)

        pwm_left, pwm_right = self.compute_motor_pwms(speed_cmd, steer_cmd)

        self.send_pwm(pwm_left, pwm_right)
        #Zielerreichung publizieren (nur einaml)
        if distance < 0.3 and not self.goal_reached:
            self.goal_reached = True
            self.target_reached_pub.publish(Bool(data=True))
            self.get_logger().info("Ziel erreicht -> Nachricht gesendet")

        # Ausgabe der aktuellen Werte

        #self.get_logger().info(f"BootPos: x={pos.x:.2f}, y={pos.y:.2f}, yaw={math.degrees(yaw):.1f}°")
        self.get_logger().info(f"Distanz: {distance:.2f} m | Heading_Fehler: {math.degrees(heading_error):.1f}°")
        #self.get_logger().info(f"PWM Left: {pwm_left}, PWM Right: {pwm_right}")

    def compute_control_errors(self, x, y, yaw):
        dx = self.goal[0] - x
        dy = self.goal[1] - y
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
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
            0,# CH5
            0, # CH6
            0, 0
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

# PID Controller Node – Detailed Explanation

<details>
<summary>Click to expand the explanation of the PID-based BlueBoat Node</summary>

## Overall Node Structure

The script is a ROS 2 node written in Python that controls a simulated BlueBoat. It listens to the boat’s position and orientation and sends motor commands to steer it toward a target.

## PID Class

This class implements a standard Proportional-Integral-Derivative controller. It tries to minimize the difference between the current value and the target by calculating:

Proportional (P): Immediate error

Integral (I): Accumulated past error

Derivative (D): Rate of change of error

```python
class PID:
    def __init__(self, kp, ki, kd):
        ...
```
🚀 Node Initialization
```python
class BlueboatPIDNode(Node):
    def __init__(self):
        ...
```
In this section:

    Connects to MAVLink to control motors

    Subscribes to odometry data from Gazebo

    Provides a service for setting new target positions

    Publishes a message when the goal is reached

## Target Service
```python
self.target_service = self.create_service(SetTarget, 'set_target', self.set_target_callback)
```
This allows other nodes (e.g., a GUI or script) to set a target position (x, y) for the BlueBoat to navigate to.

🛰️ Odometry Subscriber
```python
self.create_subscription(Odometry, '/model/blueboat/odometry', self.odom_callback, 10)
```
Subscribes to the boat's real-time position and orientation in the simulation.
## Control Logic

The odom_callback() receives new odometry data and:

    Computes control errors between current and goal positions

    Uses PID controllers to calculate speed and steering

    Translates results to motor PWM signals

    Sends commands via MAVLink

## Motor Commands
```python
def send_pwm(self, pwm_left, pwm_right):
    self.master.mav.rc_channels_override_send(...)
```
This command overrides the throttle of left and right motors using MAVLink.

## Goal Reached Feedback
```python
self.target_reached_pub.publish(Bool(data=True))
```
Once the boat reaches the target within a 30 cm threshold, a message is published on /target_reached. Other systems can react to that (e.g., start logging, load next waypoint, etc.).
## Summary Table

| Section               | Purpose                                                                 |
|------------------------|-------------------------------------------------------------------------|
| `PID` class           | Controls speed & heading corrections                                     |
| `odom_callback()`     | Runs every time new pose info is received                                |
| `compute_*()` functions | Handle direction, distance, speed, and motor power calculations         |
| `send_pwm()`          | Sends left/right PWM signals to control movement                         |
| `set_target_callback()` | Service handler for receiving new target positions                      |
| `target_reached_pub`  | Publishes once the BlueBoat is near the target (within 0.3 m threshold)  |


</details>


<details>

<summary>What is a Position Service in ROS 2?</summary>

In this context, a **ROS 2 service** is used to **send target coordinates (x, y)** to the BlueBoat at runtime.

Unlike publishing to a topic (which is one-way), a service in ROS follows a **request/response** pattern:

Client ⇨ Service ⇨ Response


This means:  
- Another node (e.g. a GUI, script, or mission planner) sends a **target position request**
- Your PID node **receives** it and updates its internal goal coordinates
- It optionally sends back a **confirmation**

---

### Why Use a Service?

- **Dynamic control**: You can update the boat’s target anytime without restarting anything.
- **Predictable interaction**: The request/response pattern gives feedback (e.g., "Target accepted").
- **Encapsulation**: The PID logic doesn’t need to know where the request came from.

---

### Example Workflow

1. Some external node runs:
   ```python
   ros2 service call /set_target blueboat_interfaces/srv/SetTarget "{x: 5.0, y: 2.0}"
   ```

    Your node handles it with:
    ```python
    self.target_service = self.create_service(SetTarget, 'set_target', self.set_target_callback)
    ```
    Inside the callback:
    ```python
    def set_target_callback(self, request, response):
        self.goal = [request.x, request.y]
        response.accepted = True
        return response
    ```
### Summary


| Part             | Purpose                                                    |
|------------------|------------------------------------------------------------|
| `SetTarget.srv`  | Defines the message structure (`x`, `y`, `accepted`)       |
| `/set_target`    | The name of the service used for calling                   |
| `create_service` | Binds the request handler (`set_target_callback`)          |
| `goal` variable  | Internally stores the new coordinates                      |

This approach allows other ROS 2 components to navigate the BlueBoat on demand – ideal for interactive or autonomous missions.
</details> 