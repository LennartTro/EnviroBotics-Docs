---
id: planner node
title: Creating a BlueBoat Planner Node
---

# BlueBoat Planner Node


While the controller node ensures that the BlueBoat can navigate to a single target position, it does **not manage missions or sequences of goals**In real-world scenarios, however, we often want our robot to:

Visit multiple waypoints,

 - Perform specific actions at each stop (e.g., environmental measurements),
 - And continue the mission autonomously without manual intervention.

To achieve this, we introduce a **planner node** whose role is to orchestrate the mission:

 - It maintains a list of goal positions.
 - It sends each target to the controller via a **ROS 2 service call**.
 - It listens to a feedback topic (/target_reached) to know when the boat has arrived.
 - It performs an action (e.g., logging, waiting, spawning markers) at each waypoint before moving to the next.

This decoupled design ensures modularity:
The planner handles the **"what to do"**, and the controller takes care of **"how to get there"**.

---

## Create a new ROS 2 package

If you haven't already, create a new workspace and a package (e.g., `blueboat_planner`):

```bash
mkdir -p ~/blueboat_ws/src
cd ~/blueboat_ws/src
ros2 pkg create --build-type ament_python blueboat_planner
```

Make sure your environment is sourced and you’re using the same workspace you built your BlueBoat simulation in.

### Package Structure

After creating the package, your folder should look like this:
```text
blueboat_planner/
├── blueboat_planner
│   └── planner_node.py
├── package.xml
├── setup.cfg
├── setup.py
└── resource/
```

Add the following line to your setup.py under entry_points:
```python
'console_scripts': [
    'planner_node = blueboat_planner.planner_node:main',
],
```
Also make sure to include blueboat_interfaces as a dependency in your package.xml.

---

## Planner Node Function

This node:

Sends a sequence of target positions using a ROS 2 service.

Waits for the robot to confirm target arrival.

Waits 5 seconds to simulate a measurement or action.

Spawns a marker at each location.

Sends the next target.

### planner_node.py

Place this file inside blueboat_planner/blueboat_planner/:
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from blueboat_interfaces.srv import SetTarget
import time
import subprocess

class Planner(Node):
    def __init__(self):
        super().__init__('blueboat_planner')

        self.client = self.create_client(SetTarget, 'set_target')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Warte auf Regler-Service...')

        self.targets = [[5.0, 0.0], [5.0, 5.0], [0.0, 5.0], [0.0, 0.0]]
        self.current_target = 0
        self.waiting_for_target = False

        self.create_subscription(Bool, '/target_reached', self.target_callback, 10)

        self.send_next_target()

    def send_next_target(self):
        if self.current_target >= len(self.targets):
            self.get_logger().info("Alle Ziele erreicht.")
            return

        x, y = self.targets[self.current_target]
        req = SetTarget.Request()
        req.x = x
        req.y = y
        self.waiting_for_target = True
        self.get_logger().info("moechte Ziel senden...")
        marker_name = f"marker_{self.current_target}"
        self.spawn_marker(marker_name, x, y)
        future = self.client.call_async(req)
        future.add_done_callback(self.handle_target_response)

    def handle_target_response(self, future):
        try:
            result = future.result()
            if result.accepted:
                self.get_logger().info(f"Ziel {self.current_target + 1} gesendet.")
            else:
                self.get_logger().warn("Ziel wurde nicht akzeptiert.")
        except Exception as e:
            self.get_logger().error(f"Service-Aufruf fehlgeschlagen: {e}")

    def target_callback(self, msg):
        self.get_logger().info(f"Callback empfangen: data={msg.data}, waiting={self.waiting_for_target}")
        if msg.data and self.waiting_for_target:
            self.waiting_for_target = False
            self.perform_action_at_target()
            self.current_target += 1
            self.send_next_target()

    def perform_action_at_target(self):
        self.get_logger().info("Warte 5 Sekunden als Aktion...")
        time.sleep(5)
        self.get_logger().info("Aktion abgeschlossen.")

    def spawn_marker(self, name, x, y, z=0.0):
        sdf_path = "/home/blueboat_sitl/gz_ws/src/marker_model/model.sdf"
        cmd = [
            "gz", "service", "-s", "/world/waves/create",
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000",
            "--req", f'sdf_filename: "{sdf_path}", name: "{name}", pose: {{ position: {{ x: {x}, y: {y}, z: {z} }} }}'
        ]
        subprocess.run(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## Run the Node

Build and source the workspace:

```bash
cd ~/blueboat_ws
colcon build
source install/setup.bash
```
Then run the planner node (inside your Docker container):

```bash
ros2 run blueboat_planner blueboat_planner
```
---

## How it Works

1. The node sends a coordinate via the set_target service.

2. The PID node drives the boat to that position.

3. When the /target_reached message is received:

    - A 5-second pause simulates data collection.

    - The next target is sent.

4. For each point, a visual marker is spawned in Gazebo.



<img src="/EnviroBotics-Docs/img/Simulation.png" alt="palnner Node" style={{ width: '100%', marginBottom: '1rem' }} />


---


## Summary

This node allows fully autonomous waypoint missions in the BlueBoat simulation. It demonstrates how ROS 2 services and topics can be used to coordinate multi-step behaviors across nodes.