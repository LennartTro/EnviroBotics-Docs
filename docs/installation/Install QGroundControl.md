---
id: Install QGroundControl
title: Install QGroundControl
---

# Install QGroundControl

## Install QGroundControl on Ubuntu

```bash
# Update package index
sudo apt update
#Enable serial-port access Add your user to the dialout group so you can talk to USB devices without root:
sudo usermod -aG dialout "$(id -un)"

# Install required dependencies
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y

# Download QGroundControl AppImage
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage

# Make it executable
chmod +x QGroundControl.AppImage

# Run it
./QGroundControl.AppImage

```

## Install QGroundControl on Windows
1. Download QGroundControl-x86_64.AppImage.

2. Make the AppImage executable:
```bash 
chmod +x QGroundControl-<arch>.AppImage
```
Run QGroundControl Either double-click the AppImage in your file manager or launch it from a terminal:
```bash
./QGroundControl-<arch>.AppImage
```