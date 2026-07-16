![Tested on: Humble](https://img.shields.io/badge/tested%20on-Humble-blue)
![Humble: passing](https://img.shields.io/badge/Humble-passing-brightgreen?logo=ros)
![Jazzy: passing](https://img.shields.io/badge/Jazzy-passing-brightgreen?logo=ros)

# Gravity 8x8 ToF Matrix Ros2 Package

 A Ros2 package for running the [DFRobot Gravity: 8x8 Matrix ToF 3D Distance Sensor](https://www.dfrobot.com/product-2999.html) and publishing it's data as a PointCloud2 data. The data is published as a 8x8 matrix at 15hz.

## Parameters:
This package exposes two parameters:
- `port`: Default: `/dev/ttyUSB0` (Serial device path) 
- `topic`: Default: `/points` (PointCloud2 topic name)

## Install:

Install Dependencies:
```shell
sudo apt install python3-pyserial ros-<YOUR_ROS_DISTRO>-sensor-msgs
```

Clone repo:
```shell
git clone git@github.com:Moth-Balls/gravity_tof_matrix.git
```


## Running Node:

Basic node start:
```shell
ros2 run gravity_tof_matrix gravity_tof_matrix
```
This will start the node using the default serial device `/dev/ttyUSB0`.


Custom serial device launch:
```shell
ros2 run gravity_tof_matrix gravity_tof_matrix --ros-args -p port:=/your/serial/device
```
This will start the node using whatever serial device you want.



## Future Implementations:
This package is fairly directed at just my purposes but I'd like to add the rest of the functionality:

- 4x4 60hz Mode
- Add support for larger 64x8 module


## Note:
This package was made with the intention of using the official [DFRobot repo](https://github.com/DFRobot/DFRobot_MatrixLidar) for this module but I could not get the .py file to work with the module. The module seems to default to outputting the lidar data automatically over serial in 8x8 15hz mode. This resulted in just reading the raw bytes and converting to ascii. 







