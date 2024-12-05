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

#include <memory>
#include <optional>
#include <string>

#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "off_highway_can/receiver.hpp"
#include "corner_radar_driver_msgs/msg/location.hpp"
#include "corner_radar_driver_msgs/msg/location_array.hpp"
#include "corner_radar_driver_msgs/msg/location_values.hpp"

namespace corner_radar_driver
{

/**
 * \brief Radar receiver class to decode CAN FD frames into location list to publish.
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
   * \brief Fill message definitions to decode frames of CAN node. Only stored definitions are
   * processed.
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
  void process(std_msgs::msg::Header header, const FrameId & id, Message & message) override;

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
   * \brief Filter locations and remove too old locations or too old B message information from locations.
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

  static constexpr uint8_t kCountLocations = 85;

  rclcpp::Publisher<Locations>::SharedPtr pub_locations_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_locations_pcl_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  /// CAN id of first location message
  uint32_t location_base_id_;

  /// Allowed age of locations and B message information
  double allowed_age_;
  double publish_frequency_;

  /// Locations stored as optionals in fixed-size array to encode validity while ensuring order
  std::array<std::optional<Location>, kCountLocations> locations_;
};
}  // namespace corner_radar_driver
