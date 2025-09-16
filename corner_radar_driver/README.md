# corner_radar_driver

The corner_radar_driver package provides a receiver node to receive and decode CAN frames of the
Bosch Corner Radar LGU Sensor into ROS messages - it implements an `off_highway_can::Receiver`.

Further information on the Bosch Corner Radar LGU Sensor, its inputs, outputs and how they can be
interpreted can be found in the documentation provided with the hardware.

## Supported devices

| **Device name** | **Part Number** | **Description** | **Supported by commit** |
| - | - | - | - |
| Bosch Corner LGU Sensor | 02033BB172–02 | - Sample version with up to 170 locations | [6c382d8](https://github.com/bosch-engineering/corner_radar_driver/commit/6c382d8a55e7705e121312974cae584b00249a27) <br> |
| Bosch Corner LGU Sensor | 02033BB0EH-01 | - Sample version with up to 255 locations | [humble-devel](https://github.com/bosch-engineering/corner_radar_driver/tree/humble-devel) <br> |
| Bosch Corner LGU Sensor | 0203306797 | - Series version with up to 255 locations and up to 4 sensors on the same CAN bus | [humble-devel](https://github.com/bosch-engineering/corner_radar_driver/tree/humble-devel) <br> |

Contact: In case of issues with the driver, create an issue in the corner_radar_driver GitHub
repository.

## Nodes

### Receiver

The radar receiver decodes CAN FD frames into a location list, manages the current list and
publishes it cyclically.

All received messages are checked to be valid (no messages with zero values on all fields) and
for their age (message not older than parameter `allowed_age`). If these checks do not succeed
the received message is not further processed and is skipped.

The relevant radar CAN FD frame IDs to process are specified by the `location_base_id`. The base ID
should correspond to the first location frame ID of the radar CAN node and needs to be adapted for
the specific bus setup.

#### Multi-Sensor Configuration

The receiver supports simultaneous data processing from up to 4 corner radar sensors, each individually
configurable through the sensor configuration parameters. Each sensor can be independently activated
or deactivated and configured with specific mounting positions and operational parameters.

**Sensor Configuration Features:**

- **Individual activation**: Each of the 4 sensors (sensor1-sensor4) can be independently enabled/disabled
- **Location capacity**: Configurable maximum number of location messages per sensor:
  - `0x01`: 85 CAN FD location messages (255 locations) - default
  - `0x02`: 56 CAN FD location messages (168 locations)
  - `0x03`: 37 CAN FD location messages (111 locations)
  - `0x04`: 28 CAN FD location messages (84 locations)
- **CAN FD source address**: Each sensor uses a distinct source address (0x18FF04B0-0x18FF04B3)
- **Radar mounting position**: Individual mounting position configuration with 6 degrees of freedom:
  - Translation: xt, yt, zt (position in meters)
  - Rotation: roll, pitch, yaw (orientation in radians)

The driver configuration regarding the maximum number of locations and the CAN FD source address 
must match the configuration of the radar sensors.

The aggregated location data from all active sensors is published as both a structured list of radar
readings and as a unified point cloud.

**Recommended Configuration (CAN Bus Load Optimization):**

Due to CAN Bus load considerations, the following configuration is recommended:

- **1 active sensor**: 85 location messages (0x01)
- **2 active sensors**: 56 location messages each (0x02)
- **3 active sensors**: 37 location messages each (0x03)
- **4 active sensors**: 28 location messages each (0x04)

#### Subscribed Topics

* **from_can_bus_fd
  ([`ros2_socketcan_msgs/msg/FdFrame`](https://github.com/autowarefoundation/ros2_socketcan/blob/main/ros2_socketcan_msgs/msg/FdFrame.msg))**
  * Radar CAN FD frames to decode

#### Published Topics

* **locations
  ([corner_radar_msgs/msg/LocationArray.msg](../corner_radar_driver_msgs/msg/LocationArray.msg))**
  * Contains location readings aggregated from all active sensors.
* **locations_pcl
  ([`sensor_msgs/PointCloud2`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html))**
  * Contains the aggregated locations from all active sensors as a unified point cloud.

#### Parameters

See [receiver_params.yaml](config/receiver_params.yaml) and [sensors_configuration.yaml](config/sensors_configuration.yaml).

## Launch files

* **[receiver_launch](launch/receiver_launch.py)**: Starts the receiver with the given parameters.
  * Arguments:
    * **params**: Path to ROS YAML parameter file to load for receiver. If not provided, default
      parameters from this package are loaded.
