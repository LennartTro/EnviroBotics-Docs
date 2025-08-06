---
id: Install QGroundControl
title: Install QGroundControl
---

# Install QGroundControl

## Install QGroundControl on Ubuntu

```bash
# Update package index
sudo apt update

# Install required dependencies
sudo apt install gstreamer1.0-plugins-base \
                 gstreamer1.0-plugins-good \
                 gstreamer1.0-plugins-bad \
                 gstreamer1.0-plugins-ugly \
                 gstreamer1.0-libav \
                 gstreamer1.0-tools \
                 gstreamer1.0-x \
                 gstreamer1.0-pulseaudio \
                 libqt5serialport5

# Download QGroundControl AppImage
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage

# Make it executable
chmod +x QGroundControl.AppImage

# Run it
./QGroundControl.AppImage

```

## Install QGroundControl on Windows

1. Go to the official download page:
     👉 [QGroundControl](https://www.qgroundcontrol.com/)
2. Download the Windows Installer (.msi)
3. Run the installer and follow the setup instructions.
4. Launch QGroundControl from the Start Menu.
> Ensure your vehicle is connected via USB or telemetry (UDP, TCP, or serial) so it can be detected by QGroundControl.