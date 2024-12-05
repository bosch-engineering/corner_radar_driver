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

#include "corner_radar_driver/receiver.hpp"

#include <numbers>
#include <regex>
#include <stdexcept>

#include "pcl_conversions/pcl_conversions.h"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "off_highway_can/helper.hpp"

#include "corner_radar_driver/pcl_point_location.hpp"

namespace corner_radar_driver
{

static constexpr double kDegToRad = std::numbers::pi / 180.0;

Receiver::Receiver(const rclcpp::NodeOptions & options)
: off_highway_can::Receiver("receiver", options, true)
{
  declare_and_get_parameters();

  pub_locations_ = create_publisher<Locations>("locations", 10);
  pub_locations_pcl_ = create_publisher<sensor_msgs::msg::PointCloud2>("locations_pcl", 10);

  Receiver::start();

  publish_timer_ = rclcpp::create_timer(
    this,
    get_clock(),
    std::chrono::duration<double>(1.0 / publish_frequency_),
    std::bind(&Receiver::manage_and_publish, this)
  );
}

Receiver::Messages Receiver::fillMessageDefinitions()
{
  Messages m;

  Message l;
  l.name = "Location_XXX";
  l.crc_index = std::nullopt;
  l.length = 64;
  // Start bit, length, big endian, signed, factor, offset
  l.signals["crc_index"] = {0, 16, false, false, 1, 0};
  l.signals["message_counter"] = {16, 8, false, false, 1, 0};
  l.signals["block_counter"] = {24, 8, false, false, 1, 0};

  l.signals["l1_radial_distance"] = {32, 16, false, false, 0.0078125, 0};
  l.signals["l1_radial_velocity"] = {48, 16, false, false, 0.00390625, -128};
  l.signals["l1_azimuth_angle"] = {64, 16, false, false, 0.01, -327.68};
  l.signals["l1_elevation_angle"] = {80, 16, false, false, 0.01, -327.68};
  l.signals["l1_radial_distance_variance"] = {96, 16, false, false, 0.0000152587890625, 0};
  l.signals["l1_radial_velocity_variance"] = {112, 16, false, false, 0.0000152587890625, 0};
  l.signals["l1_azimuth_angle_variance"] = {128, 16, false, false, 0.00390625, 0};
  l.signals["l1_elevation_angle_variance"] = {144, 16, false, false, 0.00390625, 0};
  l.signals["l1_radial_distance_velocity_covariance"] = {
    160, 16, false, false, 0.000003814697265625, -0.125};
  l.signals["l1_rcs"] = {176, 16, false, false, 0.00390625, -128};
  l.signals["l1_rssi"] = {192, 16, false, false, 0.001953125, 0};
  l.signals["l1_radial_distance_velocity_quality"] = {208, 8, false, false, 1, 0};
  l.signals["l1_azimuth_angle_quality"] = {216, 8, false, false, 1, 0};
  l.signals["l1_elevation_angle_quality"] = {224, 8, false, false, 1, 0};
  l.signals["l1_azimuthal_partner_id"] = {232, 8, false, false, 1, 0};

  l.signals["l2_radial_distance"] = {256, 16, false, false, 0.0078125, 0};
  l.signals["l2_radial_velocity"] = {272, 16, false, false, 0.00390625, -128};
  l.signals["l2_azimuth_angle"] = {288, 16, false, false, 0.01, -327.68};
  l.signals["l2_elevation_angle"] = {304, 16, false, false, 0.01, -327.68};
  l.signals["l2_radial_distance_variance"] = {320, 16, false, false, 0.0000152587890625, 0};
  l.signals["l2_radial_velocity_variance"] = {336, 16, false, false, 0.0000152587890625, 0};
  l.signals["l2_azimuth_angle_variance"] = {352, 16, false, false, 0.00390625, 0};
  l.signals["l2_elevation_angle_variance"] = {368, 16, false, false, 0.00390625, 0};
  l.signals["l2_radial_distance_velocity_covariance"] = {
    384, 16, false, false, 0.000003814697265625, -0.125};
  l.signals["l2_rcs"] = {400, 16, false, false, 0.00390625, -128};
  l.signals["l2_rssi"] = {416, 16, false, false, 0.001953125, 0};
  l.signals["l2_radial_distance_velocity_quality"] = {432, 8, false, false, 1, 0};
  l.signals["l2_azimuth_angle_quality"] = {440, 8, false, false, 1, 0};
  l.signals["l2_elevation_angle_quality"] = {448, 8, false, false, 1, 0};
  l.signals["l2_azimuthal_partner_id"] = {456, 8, false, false, 1, 0};

  // Fill message definitions
  for (uint8_t i = 0; i < kCountLocations; ++i) {
    m[location_base_id_ + i] = l;
    // Replace XX with index in message name
    auto & name = m[location_base_id_ + i].name;
    name = std::regex_replace(name, std::regex("XX"), std::to_string(i));
  }

  return m;
}

void Receiver::process(std_msgs::msg::Header header, const FrameId & id, Message & message)
{
  using off_highway_can::auto_static_cast;

  int32_t location_frame_id = id - location_base_id_;
  if (location_frame_id >= 0 && location_frame_id <= kCountLocations) {
    Location l;
    l.id = location_frame_id;
    l.header = header;

    // Extract and cast signal values from the message to the location structure
    auto_static_cast(l.crc, message.signals["crc_index"].value);
    auto_static_cast(l.alive_ctr, message.signals["message_counter"].value);
    auto_static_cast(l.prot_block_ctr, message.signals["block_counter"].value);

    auto_static_cast(
      l.location1.radial_distance, message.signals["l1_radial_distance"].value);
    auto_static_cast(
      l.location1.radial_velocity, message.signals["l1_radial_velocity"].value);
    auto_static_cast(
      l.location1.azimuth_angle,
      message.signals["l1_azimuth_angle"].value * kDegToRad);
    auto_static_cast(
      l.location1.elevation_angle, message.signals["l1_elevation_angle"].value * kDegToRad);
    auto_static_cast(
      l.location1.radial_distance_variance,
      message.signals["l1_radial_distance_variance"].value);
    auto_static_cast(
      l.location1.radial_velocity_variance,
      message.signals["l1_radial_velocity_variance"].value);
    auto_static_cast(
      l.location1.azimuth_angle_variance,
      message.signals["l1_azimuth_angle_variance"].value * kDegToRad * kDegToRad);
    auto_static_cast(
      l.location1.elevation_angle_variance,
      message.signals["l1_elevation_angle_variance"].value * kDegToRad * kDegToRad);
    auto_static_cast(
      l.location1.radial_distance_velocity_covariance,
      message.signals["l1_radial_distance_velocity_covariance"].value);
    auto_static_cast(l.location1.rcs, message.signals["l1_rcs"].value);
    auto_static_cast(l.location1.rssi, message.signals["l1_rssi"].value);
    auto_static_cast(
      l.location1.radial_distance_velocity_quality,
      message.signals["l1_radial_distance_velocity_quality"].value);
    auto_static_cast(
      l.location1.azimuth_angle_quality, message.signals["l1_azimuth_angle_quality"].value);
    auto_static_cast(
      l.location1.elevation_angle_quality,
      message.signals["l1_elevation_angle_quality"].value);
    auto_static_cast(
      l.location1.azimuthal_partner_id, message.signals["l1_azimuthal_partner_id"].value);

    auto_static_cast(
      l.location2.radial_distance, message.signals["l2_radial_distance"].value);
    auto_static_cast(
      l.location2.radial_velocity, message.signals["l2_radial_velocity"].value);
    auto_static_cast(
      l.location2.azimuth_angle,
      message.signals["l2_azimuth_angle"].value * kDegToRad);
    auto_static_cast(
      l.location2.elevation_angle, message.signals["l2_elevation_angle"].value * kDegToRad);
    auto_static_cast(
      l.location2.radial_distance_variance,
      message.signals["l2_radial_distance_variance"].value);
    auto_static_cast(
      l.location2.radial_velocity_variance,
      message.signals["l2_radial_velocity_variance"].value);
    auto_static_cast(
      l.location2.azimuth_angle_variance,
      message.signals["l2_azimuth_angle_variance"].value * kDegToRad * kDegToRad);
    auto_static_cast(
      l.location2.elevation_angle_variance,
      message.signals["l2_elevation_angle_variance"].value * kDegToRad * kDegToRad);
    auto_static_cast(
      l.location2.radial_distance_velocity_covariance,
      message.signals["l2_radial_distance_velocity_covariance"].value);
    auto_static_cast(l.location2.rcs, message.signals["l2_rcs"].value);
    auto_static_cast(l.location2.rssi, message.signals["l2_rssi"].value);
    auto_static_cast(
      l.location2.radial_distance_velocity_quality,
      message.signals["l2_radial_distance_velocity_quality"].value);
    auto_static_cast(
      l.location2.azimuth_angle_quality, message.signals["l2_azimuth_angle_quality"].value);
    auto_static_cast(
      l.location2.elevation_angle_quality,
      message.signals["l2_elevation_angle_quality"].value);
    auto_static_cast(
      l.location2.azimuthal_partner_id, message.signals["l2_azimuthal_partner_id"].value);
    locations_[l.id] = l;
  }
}

bool Receiver::filter(const Location & /*location*/)
{
  return false;
}

void Receiver::manage_and_publish()
{
  manage_locations();
  publish_locations();
  publish_pcl();
}

void Receiver::manage_locations()
{
  auto & list = locations_;

  for (auto & loc : list) {
    if (loc && (filter(*loc) || abs((now() - loc->header.stamp).seconds()) > allowed_age_)) {
      loc = {};
    }
  }
}

void Receiver::publish_locations()
{
  if (pub_locations_->get_subscription_count() == 0) {
    return;
  }

  Locations msg;
  msg.header.stamp = now();
  msg.header.frame_id = node_frame_id_;

  for (const auto & location : locations_) {
    if (location) {
      msg.locations.push_back(*location);
    }
  }

  pub_locations_->publish(msg);
}

void Receiver::publish_pcl()
{
  if (pub_locations_pcl_->get_subscription_count() == 0) {
    return;
  }

  pcl::PointCloud<PclPointLocation> locations_pcl;
  locations_pcl.is_dense = true;
  locations_pcl.header.frame_id = node_frame_id_;
  pcl_conversions::toPCL(now(), locations_pcl.header.stamp);

  for (const auto & location : locations_) {
    if (location) {
      locations_pcl.emplace_back(location->location1);
      locations_pcl.emplace_back(location->location2);
    }
  }

  sensor_msgs::msg::PointCloud2 pointcloud2;
  pcl::toROSMsg(locations_pcl, pointcloud2);
  pub_locations_pcl_->publish(pointcloud2);
}

void Receiver::declare_and_get_parameters()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;

  param_desc.description = "CAN frame id of first location message";
  declare_parameter<int>("location_base_id", 0x208, param_desc);
  location_base_id_ = get_parameter("location_base_id").as_int();

  param_desc.description =
    "Allowed age corresponding to output cycle time of sensor plus safety margin";
  declare_parameter<double>("allowed_age", 0.1);
  allowed_age_ = get_parameter("allowed_age").as_double();

  param_desc.description =
    "Frequency at which current location list (point cloud) is published. Corresponds to ~100 ms "
    "radar sending cycle time.";
  declare_parameter<double>("publish_frequency", 10.0);
  publish_frequency_ = get_parameter("publish_frequency").as_double();
}

}  // namespace corner_radar_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(corner_radar_driver::Receiver)
