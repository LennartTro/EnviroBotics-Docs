---
id: workflow
title: Project Workflow
---

# Project Workflow

## 1. Introduction & Problem Definition

Start by understanding the real-world environmental challenge (e.g., lake monitoring, temperature profiling).
Define the mission goal, choose your focus, and discuss it with your team.

---

## 2. Setup & Tooling

Prepare your development environment:

Install Ubuntu (or use a Virtual Machine)

Set up Docker and required containers

Install your IDE (e.g., VS Code)

Test your installation using the BlueBoat simulation

This step ensures everyone starts with a reliable, working toolchain.

---

## 3. Simulation & Algorithm Development

Use the Gazebo-based simulation to:

Test vehicle dynamics and sensors

Write and debug your own ROS 2 nodes

Develop controllers and planners (e.g., for waypoint navigation)

Simulation enables fast iteration without hardware risk.

---

## 4. Transition to Hardware

Once your logic works in simulation, it’s time to:

Configure the real BlueBoat

Connect and calibrate hardware

Deploy your ROS 2 code to the real system

This is where your virtual work meets the physical world.

---

## 5. Measurement System Integration

Build and integrate your custom measurement device, e.g.,:

Temperature winch system

Additional sensors (e.g., GPS, IMU)

Make sure data is collected, processed, and logged reliably via ROS 2.

---

## 6. Real-World Testing at the Lake

In the final phase, you:

Launch your boat into the lake

Follow your mission plan autonomously

Collect real-world data and observe system performance

This is where your design is validated under real environmental conditions.

---

### Work Iteratively!

Agile principles apply:
Refine your code, adjust your hardware, improve your logic – all based on testing and feedback.

Build. Test. Improve. Repeat.