# CM730 Driver

The CM730 driver node is responsible for direct communication with the
[CM730 sub
controller](http://support.robotis.com/en/product/darwin-op/references/reference/hardware_specifications/electronics/sub_controller_(cm-730).htm). It
forms a thin service that maps the Dynamixel serial protocol directly
to ROS 2 services.

## Services

The supported services are defined in the `cm730driver_msgs` package.

### Ping

Simple service to ping devices.

    # Ping the CM730 controller (which has device ID 200)
    ros2 service call ping cm730driver_msgs/Ping "{device_id: 200}"

    # Ping all motors of a Darwin/Boldbot
    for i in {1..20}; do ros2 service call ping cm730driver_msgs/Ping "{device_id: $i}"; done

### Read

Read a number of consecutive bytes from the control table of a device.

    # Read 2 bytes from the CM730 control table, starting at address 24
    # (Dynamixel power and LED pannel power status)
    ros2 service call read cm730driver_msgs/Read "{device_id: 200, address: 24, length: 2}"

### Write
    
Write a number of consecutive bytes to the control table of a device.

    # Write `1` to the CM730 control table at address 24
    # (Turn on Dynamixel power)
    ros2 service call write cm730driver_msgs/Write "{device_id: 200, address: 24, data: [1]}"
    
### BulkRead

Read ranges of data from the control tables of multiple devices.

    # Read 2 bytes from the CM730 control table, starting at address 0 (the model number),
    # and 1 byte from the table of device 3 at address 24 (torque enabled status)
    ros2 service call bulkread cm730driver_msgs/BulkRead "{read_requests: [2, 200, 0, 1, 3, 24]}"

### SyncWrite

Write data to the same range to the control tables of multiple devices.

    # Write 1 byte, starting at address 24 (torque enabled status) to the control tables
    # of devices 19 and 20; 1 (on) for the first, 0 (off) for the second
    ros2 service call syncwrite cm730driver_msgs/SyncWrite "{address: 24, length: 1, data: [19, 1, 20, 0]}"

