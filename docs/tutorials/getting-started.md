---
id: getting-started
title: Getting started
---

# Getting started
Open a Terminal and run the following command to create a new folder with the name **Summer_School** (you could also do a right click in the home file system under and select "New Folder"):
```bash
mkdir Summer_School
```
Change into the folder:
```bash
cd Summer_School
```

## Clone the repository 

Please check, that you are inside the Summer_School folder and clone the repository:
```bash
git clone https://github.com/tum-erl/gazebosim_blueboat_ardupilot_sitl
```
Now you should have a file structure like that:

```text
home/
├── Summer_School/
│   └── gazebosim_blueboat_ardupilot_sitl/
|          |── blueboat_sitl/
|          |       |── docker/
|          |       |     |──  run.sh
|          |       |     |──  build.sh
|          |       |     └──  Dockerfile
|          |       |──  ...
|          |       └── ...
|          |── gz_ws/
|          |    |── src/
|          |    |    |── "NODES"
|          |    |    └── ...
|          |    |── (install/)
|          |    |── (build/)
|          |    └── ...
|          |       
|          |──  SITL_Models/
|          └── README.md
|   
└── ...
```


---

## Build a Docker Image
Navigate to the docker folder:
```bash
cd /gazebosim_blueboat_ardupilot_sitl/blueboat_sitl/docker
```
and build the docker image:
```bash
sudo ./build.sh
```

That may take some time. If it fails, please try building it again.

---

## Install QGroundControl

1. Update your system and install dependencies: 
```bash
# Update package index
sudo apt update
#Enable serial-port access Add your user to the dialout group so you can talk to USB devices without root:
sudo usermod -aG dialout "$(id -un)"
```
```bash
# Install required dependencies
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```
2. Download QGroundControl-x86_64.AppImage - [by clicking here](https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage)
3. Create a new folder "QGroundControl" inside the Summer_School folder and put the QGroundControl-x86_64.AppImage from your download folder to you that folder (drag and drop or copy and past)

Your file system should look like this:
```text
home/
├── Summer_School
│   |── gazebosim_blueboat_ardupilot_sitl/
|   |       |── blueboat_sitl/
|   |       |       |── docker
|   |       |       |     |──  run.sh
|   |       |       |     |──  build.sh
|   |       |       |     └──  Dockerfile
|   |       |       |──  ...
|   |       |       └── ...
|   |       |── gz_ws/
|   |       |    |── src/
|   |       |    |    |── "NODES"
|   |       |    |    └── ...
|   |       |    |── (install/)
|   |       |    |── (build/)
|   |       |    └── ...
|   |       |       
|   |       |──  SITL_Models/
|   |       └── README.md
|   └── QGroundControl/
|        └── QGroundControl-x86_64.Appl...
└── ...
```

3. Make the AppImage executable:
Either run this command inside the QGRoundControl folder
```bash 
chmod +x QGroundControl-<arch>.AppImage
```
or do a right-click an the icon --> select properties and change the slider "Executable as Program"

4. Run QGroundControl Either double-click the AppImage in your file manager or launch it from a terminal:
```bash
./QGroundControl-<arch>.AppImage
```
and here you are (or somewhere else on the map):

<img src="/EnviroBotics-Docs/img/QGroundControl.png" alt="getting started" style={{ width: '100%', marginBottom: '1rem' }} />


---

## Start a Docker Container

That is part of the following chapter

---