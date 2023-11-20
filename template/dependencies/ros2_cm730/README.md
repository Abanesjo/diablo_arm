# CM730

This repository contains a suite of ROS 2 packages to work with the
[CM730 sub
controller](http://support.robotis.com/en/product/darwin-op/references/reference/hardware_specifications/electronics/sub_controller_(cm-730).htm). More
specifically, it is set up to work with a robot that has a combination
of a CM-730 and Dynamixel motors (specifically MX-28s), such as the
[DARwIn-OP](http://support.robotis.com/en/product/darwin-op.htm) or
derivatives. It should also work with (or be easy to adapt for)
similar components/robots, that use the same Dynamixel protocol, such
as the [ROBOTIS OP2 with a
CM-740](http://www.robotis.us/robotis-op2-us/), but this has not been
tested.

The following diagram shows the different components of the suite and
their interactions:

![CM-730 ROS 2 diagram](https://gitlab.com/boldhearts/ros2_cm730/raw/master/cm730ros2diagram.svg)

## Bring Up

The individual nodes, described below, can be started separately, but you can start all nodes in the following ways:

1. Using `launch`: the `cm730_bringup` package supplies a launch file
   that brings up all the nodes:
   
        ros2 launch cm730_bringup cm730_bringup.launch.py

    There is also a launch file that launches all the nodes except for
    the driver node. This is useful when you want to run the driver on
    the robot and all other nodes externally:

        ros2 launch cm730_bringup cm730_bringup_nodriver.launch.py

2. Using a composed executable: the `cm730_bringup` package supplies
    an executable that runs all nodes in a single process. Because the
    nodes share the same memory space, this may be more efficient:
    
        ros2 run cm730_bringup cm730_composed

## Components

### `cm730driver`

Performs the actual communication with the CM-730 over USB, exposes
ROS 2 services that map to the Dynamixel serial protocol.

### `cm730controller`

Runs a 125Hz loop that reads and writes to the cm730driver. It
transforms the raw byte messages from the driver to structured
`CM730Info` and `MX28Info` messages and publishes these (topics:
`/cm730/cm730info` and `/cm730/mx28info`). It also subscribes to the
`/cm730/mx28command` topic, to receive `MX28Command` messages that it
transforms into driver write requests.

### `mx_joint_controller`

Provides a higher level interface to servo motors than
`cm730controller`: rather than using raw bytes, it subscribes to
`JointCommand` messages that contain data in standard units (topic:
`/cm730/joint_commands`), and it transforms motor information messages
into standard ROS 2 `sensor_msgs/JointState` (topic: `/joint_states`)
messages, to be used for instance by ROS 2's `robot_state_publisher`
node.

### `imu_publisher`

Reads the values of the accelerometer and the gyrometer of the CM-730 and publishes them as an IMU standard message as topic `/imu/data_raw`.
