# magdock
Autonomous Drone with a magnet, so it can attach to metal.

[Videos of it working](https://drive.google.com/drive/folders/1dibxdaSVwbIMsM3NwXmNaIzQBTG8TQRz?usp=sharing)

# Contents
- [Install](#install)
- [Build](#build)
- [Run](#run)
- [Initiate Attaching and Detaching](#initiate-docking--detaching)


# Install
Before using this work's code, the CameraProcessor Node needs to be "installed", as its code uses a custom python environment.

Go to the CameraProcessor package:
```bash
  cd src/py_camera_processor
```

```bash
  ./installCamProcessor.sh
```

# Build
Edit the fcu_url variable at the top of the launch file, eg with nano:
```bash
  nano src/bringup/launch/launch.py
```

After editing the file, use colcon to build in the root of this repository.
```bash
  colcon build
```


# Run
This is to be executed on the real drone, so clone the repo to the companion computer of the drone.

Don't forget to follow the [installation instructions](#install)  and [build instructions](#build) on the drone.

After installing the dependencies and building the code, source the created bash file.
```bash
source install/setup.bash
```

Now the bringup launch-file can be used:
```bash
ros2 launch bringup launch.py
```


# Initiate Docking / Detaching
Make sure the magnet node is running.

Before attaching or detaching make sure the code was build and sourced!
```bash
  colcon build &&
  source install/setup.bash 
```
## Docking:

For docking to the marker with the id 42 use:
```bash
  ros2 service call magdrone/commander/land custom_services/srv/LandTarget "{tf2_target: marker_42}"
```
For other markers change the number in the service call.

## Detaching:
```bash
  ros2 service call magdrone/commander/takedown custom_services/srv/Takedown "{}"
```