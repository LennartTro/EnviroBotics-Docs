---
id: setup
title: Installation & Setup
---

## Clone & Run BlueBoat ArduPilot Simulation in Docker

```bash
# Clone the repository
git clone https://github.com/markusbuchholz/gazebosim_blueboat_ardupilot_sitl.git
cd gazebosim_blueboat_ardupilot_sitl

# Optional but recommended: Check the Dockerfile and build context
ls
cat Dockerfile

# Build the Docker container
sudo docker build -t blueboat-sitl .

# Run the container (with GUI support if needed)
sudo docker run -it --rm \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    blueboat-sitl
```

Follow the step-by-step setup instructions provided during the first session.
