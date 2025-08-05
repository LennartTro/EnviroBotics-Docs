---
id: concept
title: Concept
---

# Concept of the EnviroBotics Summer School

The EnviroBotics Summer School introduces students to environmental robotics, combining hardware, software, and real-world data collection. Participants will work in teams, build their own robots, and program them to perform meaningful tasks related to environmental monitoring.

In this summer school, you will get in touch with the whole robotics pipeline. Starting from sensing, over planning to control and actuation, you will be developing suitable algorithms to tackle chosen challenges.

We will be using the ROS 2 framework. To evaluate the algorithms and solutions before deploying the real robot, we rely on gazebo as simulation environment, where you can interact with the vehicle just as you would with a real vehicle. This enables using the exact same code for both simulation and real world experiment.

This documentation serves as your central guide throughout the program.

---

**Here is a short intro:**
<img src="/EnviroBotics-Docs/img/Concept.jpg" alt="Concept Overview" style={{ width: '100%', marginBottom: '1rem' }} />

<div style={{ display: 'flex', justifyContent: 'space-between', gap: '150px', flexWrap: 'wrap' }}>
  <div style={{ flex: '1 1 40%', aspectRatio: '16 / 9' }}>
    <video controls poster="/EnviroBotics-Docs/img/preview.jpg" style={{ width: '100%', height: '100%', objectFit: 'cover' }}>
      <source src="/EnviroBotics-Docs/video/Sim_vid.mp4" type="video/mp4" />
    </video>
  </div>

  <div style={{ flex: '1 1 40%', aspectRatio: '16 / 9' }}>
    <video controls poster="/EnviroBotics-Docs/img/preview.jpg" style={{ width: '100%', height: '100%', objectFit: 'cover' }}>
      <source src="/EnviroBotics-Docs/video/Video.mp4" type="video/mp4" />
    </video>
  </div>
</div>

---

## Keywords
**Gazebo**

[Gazebo](https://gazebosim.org/docs/fortress/getstarted/) is the standalone simulator that computes the physical behaviour of the objects in the simulation. It also generates sensor data or renders camera images of simulated camera sensors. gazebo itself is independent of ROS. Maybe it is the easiest to view gazebo as a drop-in replacement for the real world robot, so we can work on our algorithms without beeing in the lab all day.

**ROS2**

[ROS2](https://www.ros.org/) is the framework we use in this course. So probably most of the time (if not all) is spent on developing/implementing algorithms and letting them interact via the communication capabilities ROS 2 provides. Imagine ROS as some advanced message passing library. So instead of writing a single rather complex program, that handles all of our problems, we can use ROS to write multiple small programs that can communicate with each other via messages and solve only particular tasks.

**Docker**

[Docker](https://www.docker.com/) is a platform that helps you package software into standardized units called *containers*. These containers bundle your application code along with everything it needs to run – libraries, dependencies, and system tools. In the context of robotics, Docker makes it easy to ensure that all participants have the exact same development environment, avoiding the infamous “but it works on my machine” problem. During the summer school, we’ll use Docker to provide you with pre-configured environments for ROS, simulation tools, and more – so you can focus on building cool things, not debugging setups.

---

## Hardware Architecture
You will develop your algorithms in ROS and make use of the publisher/subscriber system, which allows us to run different functionalities on different machines. In fact ROS abstracts the communication in a way that we as users do not even recognize whether or not our programs/nodes communicate across different machines (computers) in the same network.

---

## Localization Module