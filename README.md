# corner_radar_driver

This project provides ROS drivers for Bosch Corner Radar LGU sensors.

These packages are developed for ROS 2 Humble on Ubuntu 22.04.

## CAN Based Drivers

- [**corner_radar_driver**](corner_radar_driver/README.md): Receiver
  node for the Bosch Corner Radar LGU sensor
- [**corner_radar_driver_msgs**](corner_radar_driver_msgs/README.md):
  The custom message interface for the corner_radar_driver package

The CAN communication based sensors were tested in a 500 kBd CAN configuration.

This driver uses the
[off_highway_can](https://index.ros.org/p/off_highway_can/github-bosch-engineering-off_highway_sensor_drivers/)
package for en- and decoding of CAN FD frames.

For further information, have a look at the linked package readmes.

### Architecture

The most relevant packages for an application of the CAN communication based sensors are the
[**corner_radar_driver**](corner_radar_driver/README.md) package, which provides a `receiver` node
to convert CAN FD frames received from the sensor into ROS messages.

The sensor package does **not** contain a CAN to ROS driver. Instead, the interface towards the
sensor side is encoded as
[`ros2_socketcan_msgs/msg/FdFrame`](https://github.com/autowarefoundation/ros2_socketcan/blob/main/ros2_socketcan_msgs/msg/FdFrame.msg)
ROS messages. Such messages can be handled by e.g., the
[ros2_socketcan](https://github.com/autowarefoundation/ros2_socketcan) receiver, which converts
between such ROS messages and physical CAN FD frames through the SocketCAN driver.

## Intended Use

See [intended use](doc/intended_use.md).

## License

Please see [LICENSE](LICENSE).

## Build

### Prerequisites

Install:

- Ubuntu jammy 22.04
- ROS humble

### Install Dependencies

Clone this repository into your workspace and execute in it:

```bash
rosdep update && rosdep install --from-paths src --ignore-src -r -y
```

### Compile

Execute in your workspace

```bash
colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Release'
```

for using colcon.
