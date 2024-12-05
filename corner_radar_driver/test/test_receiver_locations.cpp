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

#include <random>
#include <stdexcept>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "corner_radar_driver/receiver.hpp"
#include "off_highway_can/helper.hpp"

#include "ros2_socketcan_msgs/msg/fd_frame.hpp"

using off_highway_can::auto_static_cast;
using namespace std::chrono_literals;

static constexpr double kDegToRad = std::numbers::pi / 180.0;

inline double sgn(double x)
{
  return (x > 0) - (x < 0);
}

inline void apply_half_increment_offset(double & value, double increment)
{
  value += sgn(value) * increment * 0.5;
}

class LocationsPublisher : public rclcpp::Node
{
public:
  LocationsPublisher(
    corner_radar_driver_msgs::msg::LocationArray test_locations,
    off_highway_can::Receiver::Messages & msg_def)
  : Node("corner_radar_driver_locations_pub")
  {
    // Initialize can msg
    ros2_socketcan_msgs::msg::FdFrame can_msg_location;

    // Initialize publisher
    publisher_ = this->create_publisher<ros2_socketcan_msgs::msg::FdFrame>("from_can_bus_fd", 1);

    // Publish locations to from_can_bus
    for (corner_radar_driver_msgs::msg::Location test_location : test_locations.locations) {
      auto_static_cast(can_msg_location.id, 0x208 + test_location.id);
      auto_static_cast(can_msg_location.header.stamp, now());
      off_highway_can::Message & location_msg = msg_def[can_msg_location.id];
      auto_static_cast(location_msg.signals["crc_index"].value, test_location.crc);
      auto_static_cast(location_msg.signals["message_counter"].value, test_location.alive_ctr);
      auto_static_cast(location_msg.signals["block_counter"].value, test_location.prot_block_ctr);

      auto_static_cast(
        location_msg.signals["l1_radial_distance"].value,
        test_location.location1.radial_distance);
      auto_static_cast(
        location_msg.signals["l1_radial_velocity"].value,
        test_location.location1.radial_velocity);
      auto_static_cast(
        location_msg.signals["l1_azimuth_angle"].value,
        test_location.location1.azimuth_angle / kDegToRad);
      auto_static_cast(
        location_msg.signals["l1_elevation_angle"].value,
        test_location.location1.elevation_angle / kDegToRad);
      auto_static_cast(
        location_msg.signals["l1_radial_distance_variance"].value,
        test_location.location1.radial_distance_variance);
      auto_static_cast(
        location_msg.signals["l1_radial_velocity_variance"].value,
        test_location.location1.radial_velocity_variance);
      auto_static_cast(
        location_msg.signals["l1_azimuth_angle_variance"].value,
        test_location.location1.azimuth_angle_variance / kDegToRad / kDegToRad);
      auto_static_cast(
        location_msg.signals["l1_elevation_angle_variance"].value,
        test_location.location1.elevation_angle_variance / kDegToRad / kDegToRad);
      auto_static_cast(
        location_msg.signals["l1_radial_distance_velocity_covariance"].value,
        test_location.location1.radial_distance_velocity_covariance);
      auto_static_cast(location_msg.signals["l1_rcs"].value, test_location.location1.rcs);
      auto_static_cast(location_msg.signals["l1_rssi"].value, test_location.location1.rssi);
      auto_static_cast(
        location_msg.signals["l1_radial_distance_velocity_quality"].value,
        test_location.location1.radial_distance_velocity_quality);
      auto_static_cast(
        location_msg.signals["l1_azimuth_angle_quality"].value,
        test_location.location1.azimuth_angle_quality);
      auto_static_cast(
        location_msg.signals["l1_elevation_angle_quality"].value,
        test_location.location1.elevation_angle_quality);
      auto_static_cast(
        location_msg.signals["l1_azimuthal_partner_id"].value,
        test_location.location1.azimuthal_partner_id);

      auto_static_cast(
        location_msg.signals["l2_radial_distance"].value,
        test_location.location2.radial_distance);
      auto_static_cast(
        location_msg.signals["l2_radial_velocity"].value,
        test_location.location2.radial_velocity);
      auto_static_cast(
        location_msg.signals["l2_azimuth_angle"].value,
        test_location.location2.azimuth_angle / kDegToRad);
      auto_static_cast(
        location_msg.signals["l2_elevation_angle"].value,
        test_location.location2.elevation_angle / kDegToRad);
      auto_static_cast(
        location_msg.signals["l2_radial_distance_variance"].value,
        test_location.location2.radial_distance_variance);
      auto_static_cast(
        location_msg.signals["l2_radial_velocity_variance"].value,
        test_location.location2.radial_velocity_variance);
      auto_static_cast(
        location_msg.signals["l2_azimuth_angle_variance"].value,
        test_location.location2.azimuth_angle_variance / kDegToRad / kDegToRad);
      auto_static_cast(
        location_msg.signals["l2_elevation_angle_variance"].value,
        test_location.location2.elevation_angle_variance / kDegToRad / kDegToRad);
      auto_static_cast(
        location_msg.signals["l2_radial_distance_velocity_covariance"].value,
        test_location.location2.radial_distance_velocity_covariance);
      auto_static_cast(location_msg.signals["l2_rcs"].value, test_location.location2.rcs);
      auto_static_cast(location_msg.signals["l2_rssi"].value, test_location.location2.rssi);
      auto_static_cast(
        location_msg.signals["l2_radial_distance_velocity_quality"].value,
        test_location.location2.radial_distance_velocity_quality);
      auto_static_cast(
        location_msg.signals["l2_azimuth_angle_quality"].value,
        test_location.location2.azimuth_angle_quality);
      auto_static_cast(
        location_msg.signals["l2_elevation_angle_quality"].value,
        test_location.location2.elevation_angle_quality);
      auto_static_cast(
        location_msg.signals["l2_azimuthal_partner_id"].value,
        test_location.location2.azimuthal_partner_id);
      // Encode message
      location_msg.encode(can_msg_location.data);

      // Publish
      publisher_->publish(can_msg_location);

      defined_location_ids++;
    }
  }

  uint8_t get_defined_location_ids()
  {
    return defined_location_ids;
  }

protected:
  uint8_t defined_location_ids = 0;

private:
  rclcpp::Publisher<ros2_socketcan_msgs::msg::FdFrame>::SharedPtr publisher_;
};  // LocationsPublisher

class LocationsSubscriber : public rclcpp::Node
{
public:
  LocationsSubscriber()
  : Node("corner_radar_driver_locations_sub"), locations_updated_(false)
  {
    subscriber_ = this->create_subscription<corner_radar_driver_msgs::msg::LocationArray>(
      "locations", 1,
      std::bind(&LocationsSubscriber::locationsCallback, this, std::placeholders::_1));
  }

  corner_radar_driver_msgs::msg::LocationArray get_locations()
  {
    return received_locations_;
  }

  inline bool locationsUpdated()
  {
    return locations_updated_;
  }

  inline void resetLocationsIndicator()
  {
    locations_updated_ = false;
  }

private:
  void locationsCallback(const corner_radar_driver_msgs::msg::LocationArray msg)
  {
    received_locations_ = msg;
    locations_updated_ = true;
  }
  rclcpp::Subscription<corner_radar_driver_msgs::msg::LocationArray>::SharedPtr subscriber_;
  corner_radar_driver_msgs::msg::LocationArray received_locations_;
  bool locations_updated_;
};  // LocationsSubscriber

class RandomQuantizedGenerator
{
public:
  RandomQuantizedGenerator(double resolution, double min, double max)
  : resolution{resolution}
  {
    int64_t min_as_int = min / resolution;
    int64_t max_as_int = max / resolution;
    uniform_distribution = std::uniform_int_distribution<int64_t>{min_as_int, max_as_int};
  }

  template<class T>
  double operator()(T & rng)
  {
    return uniform_distribution(rng) * resolution;
  }

private:
  std::uniform_int_distribution<int64_t> uniform_distribution;
  double resolution;
};

class TestRadarReceiver : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Overwrite allowed age to avoid timing issues in unit tests
    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("allowed_age", 1.0)
    };
    auto node_options = rclcpp::NodeOptions();
    node_options.parameter_overrides(params);
    node_ = std::make_shared<corner_radar_driver::Receiver>(node_options);

    ASSERT_EQ(node_->get_parameter("allowed_age").as_double(), 1.0);

    locations_subscriber_ = std::make_shared<LocationsSubscriber>();
  }

  void publish_locations(corner_radar_driver_msgs::msg::LocationArray locations);
  corner_radar_driver_msgs::msg::LocationArray get_locations();
  void verify_locations(
    corner_radar_driver_msgs::msg::LocationArray test_locations,
    corner_radar_driver_msgs::msg::LocationArray received_locations);

private:
  void spin_receiver(const std::chrono::nanoseconds & duration);
  void spin_subscriber(const std::chrono::nanoseconds & duration);

  std::shared_ptr<corner_radar_driver::Receiver> node_;

  std::shared_ptr<LocationsPublisher> locations_publisher_;
  std::shared_ptr<LocationsSubscriber> locations_subscriber_;
};

void TestRadarReceiver::publish_locations(corner_radar_driver_msgs::msg::LocationArray locations)
{
  auto msg_def = node_->get_messages();
  locations_publisher_ = std::make_shared<LocationsPublisher>(locations, msg_def);
  spin_receiver(100ms);
}

corner_radar_driver_msgs::msg::LocationArray TestRadarReceiver::get_locations()
{
  spin_subscriber(500ms);
  corner_radar_driver_msgs::msg::LocationArray subscribed_locations_ =
    locations_subscriber_->get_locations();
  return subscribed_locations_;
}

void TestRadarReceiver::spin_receiver(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  while (rclcpp::ok() && node_->now() - start_time <= duration) {
    rclcpp::spin_some(node_);
  }
}

void TestRadarReceiver::spin_subscriber(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  while (rclcpp::ok() && node_->now() - start_time <= duration) {
    rclcpp::spin_some(locations_subscriber_);
    rclcpp::sleep_for(100ms);
  }
}

void TestRadarReceiver::verify_locations(
  corner_radar_driver_msgs::msg::LocationArray test_locations,
  corner_radar_driver_msgs::msg::LocationArray received_locations)
{
  EXPECT_EQ(node_->count_publishers("from_can_bus_fd"), 1U);
  EXPECT_EQ(node_->count_subscribers("from_can_bus_fd"), 1U);
  EXPECT_EQ(node_->count_publishers("locations"), 1U);
  EXPECT_EQ(node_->count_subscribers("locations"), 1U);

  // Check locations
  uint8_t found_location_ids = 0;
  for (corner_radar_driver_msgs::msg::Location received_location :
    received_locations.locations)
  {
    // Search received location in test locations
    corner_radar_driver_msgs::msg::Location current_test_location;
    for (corner_radar_driver_msgs::msg::Location test_location : test_locations.locations) {
      if (received_location.id == test_location.id) {
        current_test_location = test_location;
        break;
      }
    }

    EXPECT_EQ(received_location.id, current_test_location.id);
    EXPECT_EQ(received_location.crc, current_test_location.crc);
    EXPECT_EQ(received_location.alive_ctr, current_test_location.alive_ctr);
    EXPECT_EQ(received_location.prot_block_ctr, current_test_location.prot_block_ctr);

    EXPECT_EQ(
      received_location.location1.radial_distance,
      current_test_location.location1.radial_distance);
    EXPECT_EQ(
      received_location.location1.radial_velocity,
      current_test_location.location1.radial_velocity);
    EXPECT_NEAR(
      received_location.location1.azimuth_angle,
      current_test_location.location1.azimuth_angle, 0.09);
    EXPECT_NEAR(
      received_location.location1.elevation_angle,
      current_test_location.location1.elevation_angle, 0.09);
    EXPECT_NEAR(
      received_location.location1.radial_distance_variance,
      current_test_location.location1.radial_distance_variance, 0.0009);
    EXPECT_NEAR(
      received_location.location1.radial_velocity_variance,
      current_test_location.location1.radial_velocity_variance, 0.0009);
    EXPECT_NEAR(
      received_location.location1.azimuth_angle_variance,
      current_test_location.location1.azimuth_angle_variance, 0.0009);
    EXPECT_NEAR(
      received_location.location1.elevation_angle_variance,
      current_test_location.location1.elevation_angle_variance, 0.0009);
    EXPECT_NEAR(
      received_location.location1.radial_distance_velocity_covariance,
      current_test_location.location1.radial_distance_velocity_covariance, 0.0009);
    EXPECT_EQ(received_location.location1.rcs, current_test_location.location1.rcs);
    EXPECT_EQ(received_location.location1.rssi, current_test_location.location1.rssi);
    EXPECT_EQ(
      received_location.location1.radial_distance_velocity_quality,
      current_test_location.location1.radial_distance_velocity_quality);
    EXPECT_EQ(
      received_location.location1.azimuth_angle_quality,
      current_test_location.location1.azimuth_angle_quality);
    EXPECT_EQ(
      received_location.location1.elevation_angle_quality,
      current_test_location.location1.elevation_angle_quality);
    EXPECT_EQ(
      received_location.location1.azimuthal_partner_id,
      current_test_location.location1.azimuthal_partner_id);

    EXPECT_EQ(
      received_location.location2.radial_distance,
      current_test_location.location2.radial_distance);
    EXPECT_EQ(
      received_location.location2.radial_velocity,
      current_test_location.location2.radial_velocity);
    EXPECT_NEAR(
      received_location.location2.azimuth_angle,
      current_test_location.location2.azimuth_angle, 0.09);
    EXPECT_NEAR(
      received_location.location2.elevation_angle,
      current_test_location.location2.elevation_angle, 0.09);
    EXPECT_NEAR(
      received_location.location2.radial_distance_variance,
      current_test_location.location2.radial_distance_variance, 0.0009);
    EXPECT_NEAR(
      received_location.location2.radial_velocity_variance,
      current_test_location.location2.radial_velocity_variance, 0.0009);
    EXPECT_NEAR(
      received_location.location2.azimuth_angle_variance,
      current_test_location.location2.azimuth_angle_variance, 0.09);
    EXPECT_NEAR(
      received_location.location2.elevation_angle_variance,
      current_test_location.location2.elevation_angle_variance, 0.09);
    EXPECT_NEAR(
      received_location.location2.radial_distance_velocity_covariance,
      current_test_location.location2.radial_distance_velocity_covariance, 0.0009);
    EXPECT_EQ(received_location.location2.rcs, current_test_location.location2.rcs);
    EXPECT_EQ(received_location.location2.rssi, current_test_location.location2.rssi);
    EXPECT_EQ(
      received_location.location2.radial_distance_velocity_quality,
      current_test_location.location2.radial_distance_velocity_quality);
    EXPECT_EQ(
      received_location.location2.azimuth_angle_quality,
      current_test_location.location2.azimuth_angle_quality);
    EXPECT_EQ(
      received_location.location2.elevation_angle_quality,
      current_test_location.location2.elevation_angle_quality);
    EXPECT_EQ(
      received_location.location2.azimuthal_partner_id,
      current_test_location.location2.azimuthal_partner_id);

    found_location_ids++;
  }
  EXPECT_EQ(locations_publisher_->get_defined_location_ids(), found_location_ids);
}

TEST_F(TestRadarReceiver, testLocationZero) {
  corner_radar_driver_msgs::msg::LocationArray test_locations;
  corner_radar_driver_msgs::msg::Location test_location;
  test_location.id = 0;
  test_location.location1.radial_distance = 0.0;
  test_location.location1.radial_distance_variance = 0.0;
  test_location.location1.radial_velocity = 0.0;
  test_location.location1.radial_velocity_variance = 0.0;
  test_location.location1.radial_distance_velocity_covariance = 0.0;
  test_location.location1.radial_distance_velocity_quality = 0.0;
  test_location.location1.elevation_angle = 0.0;
  test_location.location1.elevation_angle_quality = 0.0;
  test_location.location1.elevation_angle_variance = 0.0;
  test_location.location1.azimuth_angle = 0.0;
  test_location.location1.azimuth_angle_quality = 0.0;
  test_location.location1.azimuth_angle_variance = 0.0;
  test_location.location1.azimuthal_partner_id = 0;
  test_location.location1.rcs = 0.0;
  test_location.location1.rssi = 0.0;

  test_location.location2.radial_distance = 0.0;
  test_location.location2.radial_distance_variance = 0.0;
  test_location.location2.radial_velocity = 0.0;
  test_location.location2.radial_velocity_variance = 0.0;
  test_location.location2.radial_distance_velocity_covariance = 0.0;
  test_location.location2.radial_distance_velocity_quality = 0.0;
  test_location.location2.elevation_angle = 0.0;
  test_location.location2.elevation_angle_quality = 0.0;
  test_location.location2.elevation_angle_variance = 0.0;
  test_location.location2.azimuth_angle = 0.0;
  test_location.location2.azimuth_angle_quality = 0.0;
  test_location.location2.azimuth_angle_variance = 0.0;
  test_location.location2.azimuthal_partner_id = 0;
  test_location.location2.rcs = 0.0;
  test_location.location2.rssi = 0.0;
  test_locations.locations.push_back(test_location);

  publish_locations(test_locations);
  verify_locations(test_locations, get_locations());
}

TEST_F(TestRadarReceiver, testLocationsValidValues) {
  corner_radar_driver_msgs::msg::LocationArray test_locations;
  corner_radar_driver_msgs::msg::Location test_location;
  test_location.id = 0;
  test_location.location1.radial_distance = 510.0;
  test_location.location1.radial_distance_variance = 0.5;
  test_location.location1.radial_velocity = -50.0;
  test_location.location1.radial_velocity_variance = 0.01;
  test_location.location1.radial_distance_velocity_covariance = 0.03;
  test_location.location1.radial_distance_velocity_quality = 120.0;
  test_location.location1.elevation_angle = 25.0 * kDegToRad;
  test_location.location1.elevation_angle_quality = 50.0;
  test_location.location1.elevation_angle_variance = 20.0 * kDegToRad * kDegToRad;
  test_location.location1.azimuth_angle = 45.0 * kDegToRad;
  test_location.location1.azimuth_angle_quality = 100.0;
  test_location.location1.azimuth_angle_variance = 60.0 * kDegToRad * kDegToRad;
  test_location.location1.azimuthal_partner_id = 24.0;
  test_location.location1.rcs = 32.0;
  test_location.location1.rssi = 12.5;

  test_location.location2.radial_distance = 405.0;
  test_location.location2.radial_distance_variance = 0.7;
  test_location.location2.radial_velocity = 12.0;
  test_location.location2.radial_velocity_variance = 0.8;
  test_location.location2.radial_distance_velocity_covariance = 0.1;
  test_location.location2.radial_distance_velocity_quality = 98.0;
  test_location.location2.elevation_angle = -20.0 * kDegToRad;
  test_location.location2.elevation_angle_quality = 19.0;
  test_location.location2.elevation_angle_variance = 220.0 * kDegToRad * kDegToRad;
  test_location.location2.azimuth_angle = 80.0 * kDegToRad;
  test_location.location2.azimuth_angle_quality = 62.0;
  test_location.location2.azimuth_angle_variance = 48.0 * kDegToRad * kDegToRad;
  test_location.location2.azimuthal_partner_id = 65.0;
  test_location.location2.rcs = 126.0;
  test_location.location2.rssi = 102.0;
  test_locations.locations.push_back(test_location);

  publish_locations(test_locations);
  verify_locations(test_locations, get_locations());
}

TEST_F(TestRadarReceiver, testLocationsMinValues) {
  corner_radar_driver_msgs::msg::LocationArray test_locations;
  corner_radar_driver_msgs::msg::Location test_location;
  test_location.id = 0;
  test_location.location1.radial_distance = 0.0;
  test_location.location1.radial_distance_variance = 0.0;
  test_location.location1.radial_velocity = -128.0;
  test_location.location1.radial_velocity_variance = 0.0;
  test_location.location1.radial_distance_velocity_covariance = -0.125;
  test_location.location1.radial_distance_velocity_quality = 0.0;
  test_location.location1.elevation_angle = -30.0 * kDegToRad;
  test_location.location1.elevation_angle_quality = 0.0;
  test_location.location1.elevation_angle_variance = 0.0;
  test_location.location1.azimuth_angle = -90.0 * kDegToRad;
  test_location.location1.azimuth_angle_quality = 0.0;
  test_location.location1.azimuth_angle_variance = 0.0;
  test_location.location1.azimuthal_partner_id = 0.0;
  test_location.location1.rcs = -128.0;
  test_location.location1.rssi = 0.0;

  test_location.location2.radial_distance = 0.0;
  test_location.location2.radial_distance_variance = 0.0;
  test_location.location2.radial_velocity = -128.0;
  test_location.location2.radial_velocity_variance = 0.0;
  test_location.location2.radial_distance_velocity_covariance = -0.125;
  test_location.location2.radial_distance_velocity_quality = 0.0;
  test_location.location2.elevation_angle = -30.0 * kDegToRad;
  test_location.location2.elevation_angle_quality = 0.0;
  test_location.location2.elevation_angle_variance = 0.0;
  test_location.location2.azimuth_angle = -90.0 * kDegToRad;
  test_location.location2.azimuth_angle_quality = 0.0;
  test_location.location2.azimuth_angle_variance = 0.0;
  test_location.location2.azimuthal_partner_id = 0.0;
  test_location.location2.rcs = -128.0;
  test_location.location2.rssi = 0.0;
  test_locations.locations.push_back(test_location);

  publish_locations(test_locations);
  verify_locations(test_locations, get_locations());
}

TEST_F(TestRadarReceiver, testLocationsMaxValues) {
  corner_radar_driver_msgs::msg::LocationArray test_locations;
  corner_radar_driver_msgs::msg::Location test_location;
  test_location.id = 0;
  test_location.location1.radial_distance = 511.9921875;
  test_location.location1.radial_distance_variance = 0.999984741210938;
  test_location.location1.radial_velocity = 127.99609375;
  test_location.location1.radial_velocity_variance = 0.999984741210938;
  test_location.location1.radial_distance_velocity_covariance = 0.124996185302734;
  test_location.location1.radial_distance_velocity_quality = 255.0;
  test_location.location1.elevation_angle = 30.0 * kDegToRad;
  test_location.location1.elevation_angle_quality = 255.0;
  test_location.location1.elevation_angle_variance = 255.99609375 * kDegToRad * kDegToRad;
  test_location.location1.azimuth_angle = 90.0 * kDegToRad;
  test_location.location1.azimuth_angle_quality = 255.0;
  test_location.location1.azimuth_angle_variance = 255.99609375 * kDegToRad * kDegToRad;
  test_location.location1.azimuthal_partner_id = 255.0;
  test_location.location1.rcs = 127.99609375;
  test_location.location1.rssi = 127.998046875;

  test_location.location2.radial_distance = 511.9921875;
  test_location.location2.radial_distance_variance = 0.999984741210938;
  test_location.location2.radial_velocity = 127.99609375;
  test_location.location2.radial_velocity_variance = 0.999984741210938;
  test_location.location2.radial_distance_velocity_covariance = 0.124996185302734;
  test_location.location2.radial_distance_velocity_quality = 255.0;
  test_location.location2.elevation_angle = 30.0 * kDegToRad;
  test_location.location2.elevation_angle_quality = 255.0;
  test_location.location2.elevation_angle_variance = 255.99609375 * kDegToRad * kDegToRad;
  test_location.location2.azimuth_angle = 90.0 * kDegToRad;
  test_location.location2.azimuth_angle_quality = 255.0;
  test_location.location2.azimuth_angle_variance = 255.99609375 * kDegToRad * kDegToRad;
  test_location.location2.azimuthal_partner_id = 255.0;
  test_location.location2.rcs = 127.99609375;
  test_location.location2.rssi = 127.998046875;
  test_locations.locations.push_back(test_location);

  publish_locations(test_locations);
  verify_locations(test_locations, get_locations());
}

TEST_F(TestRadarReceiver, test85RandomValidLocations) {
  corner_radar_driver_msgs::msg::LocationArray test_locations;
  // Create vector with unique IDs
  std::vector<uint8_t> ids(85);
  std::iota(std::begin(ids), std::end(ids), 0);
  std::default_random_engine rng;
  std::shuffle(std::begin(ids), std::end(ids), rng);

  for (auto id : ids) {
    corner_radar_driver_msgs::msg::Location test_location;

    test_location.id = id;
    test_location.location1.radial_distance =
      RandomQuantizedGenerator{0.0078125, 0.0, 511.9921875}(rng);
    test_location.location1.radial_distance_variance =
      RandomQuantizedGenerator{1.52587890625e-05, 0.0, 0.999984741210938}(rng);
    test_location.location1.radial_velocity =
      RandomQuantizedGenerator{0.00390625, -128.0, 127.99609375}(rng);
    test_location.location1.radial_velocity_variance =
      RandomQuantizedGenerator{1.52587890625e-05, 0.0, 0.999984741210938}(rng);
    test_location.location1.radial_distance_velocity_covariance =
      RandomQuantizedGenerator{3.814697265625e-06, -0.125, 0.124996185302734}(rng);
    test_location.location1.radial_distance_velocity_quality =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}(rng);
    test_location.location1.elevation_angle =
      RandomQuantizedGenerator{0.01 * kDegToRad, -30.0 * kDegToRad, 30.0 * kDegToRad}(rng);
    test_location.location1.elevation_angle_quality =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}(rng);
    test_location.location1.elevation_angle_variance =
      RandomQuantizedGenerator{0.00390625 * kDegToRad * kDegToRad, 0.0,
      255.99609375 * kDegToRad * kDegToRad}(rng);
    test_location.location1.azimuth_angle =
      RandomQuantizedGenerator{0.01 * kDegToRad, -90.0 * kDegToRad, 90.0 * kDegToRad}(rng);
    test_location.location1.azimuth_angle_quality =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}(rng);
    test_location.location1.azimuth_angle_variance =
      RandomQuantizedGenerator{0.00390625 * kDegToRad * kDegToRad, 0.0,
      255.99609375 * kDegToRad * kDegToRad}(rng);
    test_location.location1.azimuthal_partner_id =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}(rng);
    test_location.location1.rcs =
      RandomQuantizedGenerator{0.00390625, -128.0, 127.99609375}(rng);
    test_location.location1.rssi =
      RandomQuantizedGenerator{0.001953125, 0.0, 127.99804687}(rng);

    test_location.location2.radial_distance =
      RandomQuantizedGenerator{0.0078125, 0.0, 511.9921875}(rng);
    test_location.location2.radial_distance_variance =
      RandomQuantizedGenerator{1.52587890625e-05, 0.0, 0.999984741210938}(rng);
    test_location.location2.radial_velocity =
      RandomQuantizedGenerator{0.00390625, -128.0, 127.99609375}(rng);
    test_location.location2.radial_velocity_variance =
      RandomQuantizedGenerator{1.52587890625e-05, 0.0, 0.999984741210938}(rng);
    test_location.location2.radial_distance_velocity_covariance =
      RandomQuantizedGenerator{3.814697265625e-06, -0.125, 0.124996185302734}(rng);
    test_location.location2.radial_distance_velocity_quality =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}(rng);
    test_location.location2.elevation_angle =
      RandomQuantizedGenerator{0.00390625 * kDegToRad, -30.0 * kDegToRad, 30.0 * kDegToRad}(rng);
    test_location.location2.elevation_angle_quality =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}(rng);
    test_location.location2.elevation_angle_variance =
      RandomQuantizedGenerator{0.00390625 * kDegToRad * kDegToRad, 0.0,
      255.99609375 * kDegToRad * kDegToRad}(rng);
    test_location.location2.azimuth_angle =
      RandomQuantizedGenerator{0.01 * kDegToRad, -90.0 * kDegToRad, 90.0 * kDegToRad}(rng);
    test_location.location2.azimuth_angle_quality =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}(rng);
    test_location.location2.azimuth_angle_variance =
      RandomQuantizedGenerator{0.00390625 * kDegToRad * kDegToRad, 0.0,
      255.99609375 * kDegToRad * kDegToRad}(rng);
    test_location.location2.azimuthal_partner_id =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}(rng);
    test_location.location2.rcs =
      RandomQuantizedGenerator{0.00390625, -128.0, 127.99609375}(rng);
    test_location.location2.rssi =
      RandomQuantizedGenerator{0.001953125, 0.0, 127.99804687}(rng);
    test_locations.locations.push_back(test_location);
  }

  publish_locations(test_locations);
  verify_locations(test_locations, get_locations());
}

TEST_F(TestRadarReceiver, testInvalidLocationId) {
  corner_radar_driver_msgs::msg::LocationArray test_locations;
  corner_radar_driver_msgs::msg::Location test_location;
  // Set invalid location ID
  test_location.id = 245;

  test_locations.locations.push_back(test_location);

  EXPECT_THROW(publish_locations(test_locations), std::runtime_error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
