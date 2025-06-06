// Copyright 2023 Robert Bosch GmbH and its subsidiaries
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <numbers>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <stdexcept>
#include <string>

#include "pcl_conversions/pcl_conversions.h"

#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "off_highway_can/receiver.hpp"
#include "corner_radar_driver_msgs/msg/location.hpp"
#include "corner_radar_driver_msgs/msg/location_array.hpp"
#include "corner_radar_driver_msgs/msg/location_values.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace corner_radar_driver
{

/**
 * \brief Radar receiver class to decode CAN FD frames into location list to
 * publish.
 *
 * Location list is published as simple list or as point cloud.
 */
class Receiver : public off_highway_can::Receiver
{
public:
  using Message = off_highway_can::Message;
  using LocationValues = corner_radar_driver_msgs::msg::LocationValues;
  using Location = corner_radar_driver_msgs::msg::Location;
  using Locations = corner_radar_driver_msgs::msg::LocationArray;

  /**
   * \brief Construct a new Receiver object.
   */
  explicit Receiver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * \brief Destroy the Receiver object.
   */
  ~Receiver() = default;

private:
  // API to fill
  /**
   * \brief Fill message definitions to decode frames of CAN node. Only stored
   * definitions are processed.
   *
   * \return Messages of CAN node to decode and process
   */
  Messages fillMessageDefinitions() override;

  /**
   * \brief Process CAN message (e.g. convert into own data type).
   *
   * \param header Header of corresponding ROS message
   * \param id Id of respective CAN frame
   * \param message Decoded message (values) of frame to use for processing
   */
  void process(
    std_msgs::msg::Header header, const FrameId & id,
    Message & message) override;

  /**
   * \brief Check if location is invalid.
   *
   * \param location Location to check
   * \return True if location is invalid and should be filtered, false otherwise
   */
  bool filter(const Location & location);

  /**
   * \brief Manage location list and publish it.
   */
  void manage_and_publish();

  /**
   * \brief Filter locations and remove too old locations or too old B message
   * information from locations.
   */
  void manage_locations();

  /**
   * \brief Publish locations as list.
   */
  void publish_locations();

  /**
   * \brief Publish locations as point cloud.
   */
  void publish_pcl();

  /**
   * \brief Update diagnostics status by checking last sensor information.
   *
   * Uses sensor blind, SW / HW / CAN / config fail, a set DTC, sensor not safe
   * of sensor information message to indicate sensor error.
   *
   * \param stat Status wrapper of diagnostics.
   */
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) const;

  /**
   * \brief Declare and get node parameters
   */
  void declare_and_get_parameters();

  /**
   * @brief Transforms a point location from a given frame to the "base_link"
   * frame and adds it to a point cloud.
   *
   * This function looks up the transform between the specified frame and
   * "base_link" using the provided tf2 buffer. It applies the transformation
   * (translation and rotation) to the input point location, updates its
   * coordinates, and appends the transformed point to the provided point cloud.
   * If the transform is not available, an error is logged.
   *
   * @param location         The point location to be transformed (input).
   * @param frame_id         The frame ID in which the input location is
   * defined.
   * @param locations_pcl    The point cloud to which the transformed location
   * will be added.
   * @param tf_buffer        Shared pointer to the tf2_ros::Buffer used for
   * looking up transforms.
   */
  void process_location(
    PclPointLocation & location, std::string & frame_id,
    pcl::PointCloud<PclPointLocation> & locations_pcl,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  struct SensorInfo
  {
    bool active;
    uint16_t max_number_locations;
    uint32_t can_fd_source_address;
  };

  std::map<std::string, SensorInfo> sensors_;
  std::map<uint16_t, std::string> id_to_sensor_;

  const std::map<uint16_t, uint16_t> did_to_loc_number_ = {
    {1, 85}, {2, 56}, {3, 37}, {4, 28}};

  rclcpp::Publisher<Locations>::SharedPtr pub_locations_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pub_locations_pcl_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  /// Maximum number of locations that can be stored assuming 85 location messages
  /// (maximum) per sensor
  static constexpr uint32_t kMaxLocations = 340;

  uint32_t location_base_id_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  /// Allowed age of locations and B message information
  double allowed_age_;
  double publish_frequency_;

  /// Locations stored as optionals in fixed-size array to encode validity while
  /// ensuring order
  std::array<std::optional<Location>, kMaxLocations> locations_;
  std::array<uint16_t, 4> total_number_of_locations_{0, 0, 0, 0};
};
}  // namespace corner_radar_driver
