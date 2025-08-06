---
id: Install Docker
title: Install Docker
---

# Install Docker

## Install Docker (Ubuntu)

To install Docker on Ubuntu, run the following:

```bash
# Update your existing list of packages
sudo apt-get update

# Install packages to allow apt to use a repository over HTTPS
sudo apt-get install ca-certificates curl

# Add Docker’s official GPG key:
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the Docker repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo \"${UBUNTU_CODENAME:-$VERSION_CODENAME}\") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Update your package list again
sudo apt-get update

# Install Docker Engine
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Test the installation
sudo docker run hello-world
```

<details>
<summary><strong>What is Docker and why are we using it?</strong></summary>

<br/>

## Docker in Robotics: Why we use containers instead of full system installations

### What is Docker?

Docker is a platform that allows you to **package software environments into containers**. These containers include your application, all necessary dependencies, and even a minimal operating system — so they run **consistently on any computer**.

> Think of Docker as a “portable lab environment” that always works the same way, no matter where it runs.

Each container behaves like a lightweight virtual computer — but without the overhead of a full virtual machine.

In robotics, Docker is especially useful because it allows us to:

- Avoid the “it works on my machine” problem  
- Share **identical environments** across multiple students and developers  
- Quickly switch between ROS versions or setups  
- Ensure that everyone works with the exact same system configuration

Without Docker, each student would need to manually install a matching Linux OS, ROS version, simulator setup, and all dependencies — a tedious and error-prone process.

Docker containers are cross-platform and portable. You can run them on Windows, macOS, or Linux — as long as Docker is installed. This makes collaboration, version control, and system replication significantly easier.

<br/>

#### Visual overview:

<img src="/EnviroBotics-Docs/img/Docker_CHATGPT.png" alt="Docker vs Native Setup Comparison" style={{ width: '50%', border: '1px solid #ccc', borderRadius: '8px', marginTop: '1rem' }} />

---

### Why is Docker useful for ROS?

Installing ROS (Robot Operating System) manually can be **complex and error-prone**, especially when dealing with different operating systems, conflicting library versions, or specific dependencies.

Docker solves this by:

- Providing **identical environments** for all participants  
- Reducing setup time to just a few minutes  
- Preventing accidental damage to your own OS  
- Allowing multiple ROS versions side by side  
- Making your entire setup easily shareable and reproducible  

---

### How Docker works (simplified)

#### Without Docker (traditional installation)
```
| Laptop / PC
| ├─ OS (e.g. Ubuntu 22.04)
| ├─ ROS 2 Humble
| ├─ Python 3.10
| └─ Manually installed libraries and tools
```

**Problems:**  
– Everyone has a slightly different setup  
– Version conflicts are common  
– Setup takes time and can break things

#### With Docker
```
Laptop / PC
├─ Any OS (Windows, macOS, Linux)
├─ Docker Engine
└─ Container
   ├─ Mini Ubuntu
   ├─ ROS 2 Humble
   ├─ Python 3.10
   └─ All required tools and libraries
```



**Benefits:**  
– Same environment for everyone  
– Fast and reliable setup  
– Works on all platforms

---

### Container vs. Virtual Machine

> **Containers are not VMs**  
> Containers share the host OS kernel, making them **lightweight** and **fast** to start, unlike full virtual machines which simulate a complete OS.

---

### Summary

Using Docker ensures:

- You always have a **working ROS environment**
- No more setup headaches
- Easy sharing and portability of projects

Especially in a robotics summer school with many participants and diverse computers, Docker is the **fastest, safest, and most efficient** way to get everyone up and running.

</details>
