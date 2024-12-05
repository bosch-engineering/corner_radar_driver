# corner_radar_driver

The corner_radar_driver package provides a receiver node to receive and decode CAN frames of the
Bosch Corner Radar LGU Sensor into ROS messages - it implements an `off_highway_can::Receiver`.

Further information on the Bosch Corner Radar LGU Sensor, it's inputs, outputs and how they can be
interpreted can be found in the documentation that has been provided together with the Hardware.

## Supported devices

| **Device name** | **Part Number** | **Description** |
| -| - | - |
| CR5CP LGU | 0 203 3BB 172 - 02 | - Radar sensor with up to 170 locations <br> |

Contact: In case of issues with the driver create an issue in the corner_radar_driver GitHub
repository.

## Nodes

### Receiver

The radar receiver decodes CAN FD frames into a location list, manages the current list and
publishes it cyclically.

All received messages are checked for their age (message not older than parameter `allowed_age`). If
this check does not succeed the received message is not further processed and skipped.

The relevant radar CAN FD frame IDs to process are specified by the `location_base_id`. The base ID
should correspond to the first location frame ID of the radar CAN node and needs to be adapted for
the specific bus setup.

The location list is published as a list of radar readings or as a point cloud and contains up to
170 locations.

#### Subscribed Topics

* **from_can_bus_fd
  ([`ros2_socketcan_msgs/msg/FdFrame`](https://github.com/autowarefoundation/ros2_socketcan/blob/main/ros2_socketcan_msgs/msg/FdFrame.msg))**
  * Radar CAN FD frames to decode

#### Published Topics

* **locations
  ([corner_radar_msgs/msg/LocationArray.msg])**
  * Contains location readings.
* **locations_pcl
  ([`sensor_msgs/PointCloud2`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html))**
  * Contains current locations list of radar as point cloud.

#### Parameters

See [receiver_params.yaml](config/receiver_params.yaml).

## Launch files

* **[receiver_launch](launch/receiver_launch.py)**: Starts the receiver with the given parameters.
  * Arguments:
    * **params**: Path to ROS YAML parameter file to load for receiver. If not provided, default
      parameters from this package are loaded.
