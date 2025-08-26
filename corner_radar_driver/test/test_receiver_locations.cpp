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
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <numeric>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "corner_radar_driver/pcl_point_location.hpp"
#include "corner_radar_driver/receiver.hpp"
#include "off_highway_can/helper.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros2_socketcan_msgs/msg/fd_frame.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using off_highway_can::auto_static_cast;
using namespace std::chrono_literals;

static constexpr double kDegToRad = std::numbers::pi / 180.0;
// Publish frequency for locations
static constexpr double kPublishFrequency = 10.0;
// CAN ID offset between two locations
static constexpr size_t kLocationIdOffset = 256;
// Maximum number of locations that need to be published
static constexpr size_t kMaxLocations = 340;


inline double sgn(double x)
{
  return (x > 0) - (x < 0);
}

inline void apply_half_increment_offset(double & value, double increment)
{
  value += sgn(value) * increment * 0.5;
}

struct SensorInfo
{
  bool active;
  uint16_t max_number_locations;
  uint32_t can_fd_source_address;
  struct MountingPosition
  {
    double xt = 0.0;
    double yt = 0.0;
    double zt = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
  } mounting_position;
};

std::map<uint16_t, std::string> id_to_sensor_;

const std::map<uint16_t, uint16_t> did_to_loc_number_ = {
  {1, 85},
  {2, 56},
  {3, 37},
  {4, 28}
};

using SensorConfig = std::map<std::string, SensorInfo>;

class LocationsPublisher : public rclcpp::Node
{
public:
  LocationsPublisher()
  : Node("corner_radar_driver_receiver") {}

  uint16_t get_defined_location_ids()
  {
    return defined_location_ids;
  }

  void override_location_ids(uint16_t loc_ids)
  {
    defined_location_ids = loc_ids;
  }

protected:
  uint16_t defined_location_ids = 0;
  rclcpp::Publisher<ros2_socketcan_msgs::msg::FdFrame>::SharedPtr publisher_;

// private:
};  // LocationsPublisher

class LocationsSubscriber : public rclcpp::Node
{
public:
  LocationsSubscriber()
  : Node("test_locations_subscriber")
  {
    subscription_ = this->create_subscription<corner_radar_driver_msgs::msg::LocationArray>(
      "locations", 10,
      [this](const corner_radar_driver_msgs::msg::LocationArray::SharedPtr msg) {
        received_msgs_.push_back(*msg);
      });
  }
  corner_radar_driver_msgs::msg::LocationArray get_received_messages() const
  {
    if (received_msgs_.empty()) {throw std::runtime_error("No messages received");}
    return received_msgs_.back();
  }
  void clear_messages() {received_msgs_.clear();}

private:
  rclcpp::Subscription<corner_radar_driver_msgs::msg::LocationArray>::SharedPtr subscription_;
  std::vector<corner_radar_driver_msgs::msg::LocationArray> received_msgs_;
};

class PclSubscriber : public rclcpp::Node
{
public:
  PclSubscriber()
  : Node("corner_radar_driver_pcl_sub"), pcl_updated_(false)
  {
    subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "locations_pcl", 1,
      std::bind(&PclSubscriber::pclCallback, this, std::placeholders::_1));
  }
  sensor_msgs::msg::PointCloud2 get_pcl() {return received_pcl_;}
  bool locationsUpdated() {return pcl_updated_;}
  void resetLocationsIndicator() {pcl_updated_ = false;}

private:
  void pclCallback(const sensor_msgs::msg::PointCloud2 msg)
  {
    received_pcl_ = msg;
    pcl_updated_ = true;
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
  sensor_msgs::msg::PointCloud2 received_pcl_;
  bool pcl_updated_;
};

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

class TestRadarReceiver : public testing::TestWithParam<SensorConfig>
{
protected:
  SensorConfig sensors_config_;
  void SetUp()
  {
    sensors_config_ = GetParam();

    std::map<std::string, SensorInfo> sensors = sensors_config_.empty() ? SensorConfig{
      {"sensor1", {true, 0x01, 0x18FF04B0, {}}},
      {"sensor2", {false, 0x01, 0x18FF04B1, {}}},
      {"sensor3", {false, 0x01, 0x18FF04B2, {}}},
      {"sensor4", {false, 0x01, 0x18FF04B3, {}}}
    } : sensors_config_;

    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("name", "corner_radar_driver_receiver"),
      rclcpp::Parameter("allowed_age", kMaxLocations / kPublishFrequency),
      rclcpp::Parameter("node_frame_id", "base_link"),
      rclcpp::Parameter("sensors.sensor1.active", static_cast<bool>(sensors["sensor1"].active)),
      rclcpp::Parameter(
        "sensors.sensor1.max_number_locations",
        static_cast<int64_t>(sensors["sensor1"].max_number_locations)),
      rclcpp::Parameter(
        "sensors.sensor1.can_fd_source_address",
        static_cast<int64_t>(sensors["sensor1"].can_fd_source_address)),
      rclcpp::Parameter("sensors.sensor2.active", static_cast<bool>(sensors["sensor2"].active)),
      rclcpp::Parameter(
        "sensors.sensor2.max_number_locations",
        static_cast<int64_t>(sensors["sensor2"].max_number_locations)),
      rclcpp::Parameter(
        "sensors.sensor2.can_fd_source_address",
        static_cast<int64_t>(sensors["sensor2"].can_fd_source_address)),
      rclcpp::Parameter("sensors.sensor3.active", static_cast<bool>(sensors["sensor3"].active)),
      rclcpp::Parameter(
        "sensors.sensor3.max_number_locations",
        static_cast<int64_t>(sensors["sensor3"].max_number_locations)),
      rclcpp::Parameter(
        "sensors.sensor3.can_fd_source_address",
        static_cast<int64_t>(sensors["sensor3"].can_fd_source_address)),
      rclcpp::Parameter("sensors.sensor4.active", static_cast<bool>(sensors["sensor4"].active)),
      rclcpp::Parameter(
        "sensors.sensor4.max_number_locations",
        static_cast<int64_t>(sensors["sensor4"].max_number_locations)),
      rclcpp::Parameter(
        "sensors.sensor4.can_fd_source_address",
        static_cast<int64_t>(sensors["sensor4"].can_fd_source_address))
    };

    auto node_options = rclcpp::NodeOptions();
    node_options.parameter_overrides(params);
    std::this_thread::sleep_for(100ms);

    node_ = std::make_shared<corner_radar_driver::Receiver>(node_options);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rclcpp::Clock::make_shared());
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    publish_static_transforms(sensors);

    ASSERT_EQ(node_->get_parameter("allowed_age").as_double(), kMaxLocations / kPublishFrequency);
    ASSERT_EQ(node_->get_parameter("node_frame_id").as_string(), "base_link");

    pcl_subscriber_ = std::make_shared<PclSubscriber>();
    locations_subscriber_ = std::make_shared<LocationsSubscriber>();
  }
  void TearDown() {stop_parallel_publishing();}
  corner_radar_driver_msgs::msg::LocationArray get_locations();
  sensor_msgs::msg::PointCloud2 get_pcl();
  void spin_subscriber_pcl(const std::chrono::nanoseconds & duration);
  void verify_pcl(sensor_msgs::msg::PointCloud2 received_pcl);
  void verify_locations(
    corner_radar_driver_msgs::msg::LocationArray test_locations,
    corner_radar_driver_msgs::msg::LocationArray received_locations);
  void publish_static_transforms(const std::map<std::string, SensorInfo> & sensors);
  void start_parallel_publishing(
    const corner_radar_driver_msgs::msg::LocationArray & locations,
    double rate_hz = 10.0, std::map<std::string, SensorInfo> sensors = {});
  void stop_parallel_publishing();
  std::shared_ptr<corner_radar_driver::Receiver> node_;
  std::shared_ptr<LocationsSubscriber> locations_subscriber_;
  std::atomic<bool> pub_running_;

private:
  std::shared_ptr<PclSubscriber> pcl_subscriber_;
  std::thread publisher_thread_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

corner_radar_driver_msgs::msg::LocationArray TestRadarReceiver::get_locations()
{
  auto received_locations = locations_subscriber_->get_received_messages();
  return received_locations;
}

void TestRadarReceiver::publish_static_transforms(const std::map<std::string, SensorInfo> & sensors)
{
  std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
  for (const auto & sensor_pair : sensors) {
    const SensorInfo & sensor_info = sensor_pair.second;
    if (sensor_info.active) {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.stamp = rclcpp::Clock().now();
      transform_stamped.header.frame_id = "base_link";
      transform_stamped.child_frame_id = sensor_pair.first;
      transform_stamped.transform.translation.x = sensor_info.mounting_position.xt;
      transform_stamped.transform.translation.y = sensor_info.mounting_position.yt;
      transform_stamped.transform.translation.z = sensor_info.mounting_position.zt;
      tf2::Quaternion quaternion;
      quaternion.setRPY(
        sensor_info.mounting_position.roll, sensor_info.mounting_position.pitch,
        sensor_info.mounting_position.yaw);
      transform_stamped.transform.rotation.x = quaternion.x();
      transform_stamped.transform.rotation.y = quaternion.y();
      transform_stamped.transform.rotation.z = quaternion.z();
      transform_stamped.transform.rotation.w = quaternion.w();
      static_transforms.push_back(transform_stamped);
    }
  }
  if (!static_transforms.empty()) {tf_broadcaster_->sendTransform(static_transforms);}
}

sensor_msgs::msg::PointCloud2 TestRadarReceiver::get_pcl()
{
  spin_subscriber_pcl(500ms);
  sensor_msgs::msg::PointCloud2 subscribed_pcl_ =
    pcl_subscriber_->get_pcl();
  return subscribed_pcl_;
}

void TestRadarReceiver::spin_subscriber_pcl(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  while (rclcpp::ok() && node_->now() - start_time <= duration) {
    rclcpp::spin_some(pcl_subscriber_);
    rclcpp::sleep_for(100ms);
  }
}

void TestRadarReceiver::start_parallel_publishing(
  const corner_radar_driver_msgs::msg::LocationArray & locations,
  double rate_hz,
  std::map<std::string, SensorInfo> sensors)
{
  stop_parallel_publishing();

  pub_running_ = true;
  publisher_thread_ = std::thread(
    [this, locations, rate_hz, sensors, test_locations = locations]() {
      auto msg_def = node_->get_messages();
      auto publisher_node = std::make_shared<LocationsPublisher>();
      auto publisher =
      publisher_node->create_publisher<ros2_socketcan_msgs::msg::FdFrame>("from_can_bus_fd", 1);

      ros2_socketcan_msgs::msg::FdFrame can_msg_location;

      rclcpp::WallRate rate(rate_hz);
      size_t defined_location_ids = 0;
      size_t location_index = 0;
      int k = 0;
      for (const auto & sensor_pair : sensors) {
        const SensorInfo & info = sensor_pair.second;

        if (!info.active) {
          continue;
        }

        uint32_t base_location_id = info.can_fd_source_address;
        uint16_t num_locations = did_to_loc_number_.at(info.max_number_locations);
        for (uint16_t i = 0; i < num_locations; ++i, ++location_index) {
          auto_static_cast(can_msg_location.id, base_location_id + i * kLocationIdOffset);
          // auto_static_cast(can_msg_location.header.stamp, rclcpp::Clock().now());
          auto_static_cast(can_msg_location.header.stamp, publisher_node->now());
          off_highway_can::Message & location_msg = msg_def[can_msg_location.id];
          auto_static_cast(
            location_msg.signals["crc_index"].value,
            test_locations.locations[location_index].crc);
          auto_static_cast(
            location_msg.signals["message_counter"].value,
            test_locations.locations[location_index].alive_ctr);
          auto_static_cast(
            location_msg.signals["block_counter"].value,
            test_locations.locations[location_index].prot_block_ctr);

          auto_static_cast(
            location_msg.signals["l1_radial_distance"].value,
            test_locations.locations[location_index].location1.radial_distance);
          auto_static_cast(
            location_msg.signals["l1_radial_velocity"].value,
            test_locations.locations[location_index].location1.radial_velocity);
          auto_static_cast(
            location_msg.signals["l1_azimuth_angle"].value,
            test_locations.locations[location_index].location1.azimuth_angle / kDegToRad);
          auto_static_cast(
            location_msg.signals["l1_elevation_angle"].value,
            test_locations.locations[location_index].location1.elevation_angle / kDegToRad);
          auto_static_cast(
            location_msg.signals["l1_radial_distance_variance"].value,
            test_locations.locations[location_index].location1.radial_distance_variance);
          auto_static_cast(
            location_msg.signals["l1_radial_velocity_variance"].value,
            test_locations.locations[location_index].location1.radial_velocity_variance);
          auto_static_cast(
            location_msg.signals["l1_azimuth_angle_variance"].value,
            test_locations.locations[location_index].location1.azimuth_angle_variance / kDegToRad /
            kDegToRad);
          auto_static_cast(
            location_msg.signals["l1_elevation_angle_variance"].value,
            test_locations.locations[location_index].location1.elevation_angle_variance /
            kDegToRad / kDegToRad);
          auto_static_cast(
            location_msg.signals["l1_radial_distance_velocity_covariance"].value,
            test_locations.locations[location_index].location1.radial_distance_velocity_covariance);
          auto_static_cast(
            location_msg.signals["l1_rcs"].value,
            test_locations.locations[location_index].location1.rcs);
          auto_static_cast(
            location_msg.signals["l1_rssi"].value,
            test_locations.locations[location_index].location1.rssi);
          auto_static_cast(
            location_msg.signals["l1_radial_distance_velocity_quality"].value,
            test_locations.locations[location_index].location1.radial_distance_velocity_quality);
          auto_static_cast(
            location_msg.signals["l1_azimuth_angle_quality"].value,
            test_locations.locations[location_index].location1.azimuth_angle_quality);
          auto_static_cast(
            location_msg.signals["l1_elevation_angle_quality"].value,
            test_locations.locations[location_index].location1.elevation_angle_quality);
          auto_static_cast(
            location_msg.signals["l1_azimuthal_partner_id"].value,
            test_locations.locations[location_index].location1.azimuthal_partner_id);
          auto_static_cast(
            location_msg.signals["l1_measurement_status"].value,
            test_locations.locations[location_index].location1.measurement_status);

          auto_static_cast(
            location_msg.signals["l2_radial_distance"].value,
            test_locations.locations[location_index].location2.radial_distance);
          auto_static_cast(
            location_msg.signals["l2_radial_velocity"].value,
            test_locations.locations[location_index].location2.radial_velocity);
          auto_static_cast(
            location_msg.signals["l2_azimuth_angle"].value,
            test_locations.locations[location_index].location2.azimuth_angle / kDegToRad);
          auto_static_cast(
            location_msg.signals["l2_elevation_angle"].value,
            test_locations.locations[location_index].location2.elevation_angle / kDegToRad);
          auto_static_cast(
            location_msg.signals["l2_radial_distance_variance"].value,
            test_locations.locations[location_index].location2.radial_distance_variance);
          auto_static_cast(
            location_msg.signals["l2_radial_velocity_variance"].value,
            test_locations.locations[location_index].location2.radial_velocity_variance);
          auto_static_cast(
            location_msg.signals["l2_azimuth_angle_variance"].value,
            test_locations.locations[location_index].location2.azimuth_angle_variance / kDegToRad /
            kDegToRad);
          auto_static_cast(
            location_msg.signals["l2_elevation_angle_variance"].value,
            test_locations.locations[location_index].location2.elevation_angle_variance /
            kDegToRad / kDegToRad);
          auto_static_cast(
            location_msg.signals["l2_radial_distance_velocity_covariance"].value,
            test_locations.locations[location_index].location2.radial_distance_velocity_covariance);
          auto_static_cast(
            location_msg.signals["l2_rcs"].value,
            test_locations.locations[location_index].location2.rcs);
          auto_static_cast(
            location_msg.signals["l2_rssi"].value,
            test_locations.locations[location_index].location2.rssi);
          auto_static_cast(
            location_msg.signals["l2_radial_distance_velocity_quality"].value,
            test_locations.locations[location_index].location2.radial_distance_velocity_quality);
          auto_static_cast(
            location_msg.signals["l2_azimuth_angle_quality"].value,
            test_locations.locations[location_index].location2.azimuth_angle_quality);
          auto_static_cast(
            location_msg.signals["l2_elevation_angle_quality"].value,
            test_locations.locations[location_index].location2.elevation_angle_quality);
          auto_static_cast(
            location_msg.signals["l2_azimuthal_partner_id"].value,
            test_locations.locations[location_index].location2.azimuthal_partner_id);
          auto_static_cast(
            location_msg.signals["l2_measurement_status"].value,
            test_locations.locations[location_index].location2.measurement_status);

          auto_static_cast(
            location_msg.signals["l3_radial_distance"].value,
            test_locations.locations[location_index].location3.radial_distance);
          auto_static_cast(
            location_msg.signals["l3_radial_velocity"].value,
            test_locations.locations[location_index].location3.radial_velocity);
          auto_static_cast(
            location_msg.signals["l3_azimuth_angle"].value,
            test_locations.locations[location_index].location3.azimuth_angle / kDegToRad);
          auto_static_cast(
            location_msg.signals["l3_elevation_angle"].value,
            test_locations.locations[location_index].location3.elevation_angle / kDegToRad);
          auto_static_cast(
            location_msg.signals["l3_radial_distance_variance"].value,
            test_locations.locations[location_index].location3.radial_distance_variance);
          auto_static_cast(
            location_msg.signals["l3_radial_velocity_variance"].value,
            test_locations.locations[location_index].location3.radial_velocity_variance);
          auto_static_cast(
            location_msg.signals["l3_azimuth_angle_variance"].value,
            test_locations.locations[location_index].location3.azimuth_angle_variance / kDegToRad /
            kDegToRad);
          auto_static_cast(
            location_msg.signals["l3_elevation_angle_variance"].value,
            test_locations.locations[location_index].location3.elevation_angle_variance /
            kDegToRad / kDegToRad);
          auto_static_cast(
            location_msg.signals["l3_radial_distance_velocity_covariance"].value,
            test_locations.locations[location_index].location3.radial_distance_velocity_covariance);
          auto_static_cast(
            location_msg.signals["l3_rcs"].value,
            test_locations.locations[location_index].location3.rcs);
          auto_static_cast(
            location_msg.signals["l3_rssi"].value,
            test_locations.locations[location_index].location3.rssi);
          auto_static_cast(
            location_msg.signals["l3_radial_distance_velocity_quality"].value,
            test_locations.locations[location_index].location3.radial_distance_velocity_quality);
          auto_static_cast(
            location_msg.signals["l3_azimuth_angle_quality"].value,
            test_locations.locations[location_index].location3.azimuth_angle_quality);
          auto_static_cast(
            location_msg.signals["l3_elevation_angle_quality"].value,
            test_locations.locations[location_index].location3.elevation_angle_quality);
          auto_static_cast(
            location_msg.signals["l3_azimuthal_partner_id"].value,
            test_locations.locations[location_index].location3.azimuthal_partner_id);
          auto_static_cast(
            location_msg.signals["l3_measurement_status"].value,
            test_locations.locations[location_index].location3.measurement_status);
          // Encode message
          location_msg.encode(can_msg_location.data);

          // Publish
          can_msg_location.is_extended = true;
          publisher->publish(can_msg_location);

          defined_location_ids++;

          rclcpp::spin_some(publisher_node);
          rate.sleep();
        }
        k++;
      }
      pub_running_ = false;
    });
}

void TestRadarReceiver::stop_parallel_publishing()
{
  pub_running_ = false;
  if (publisher_thread_.joinable()) {
    publisher_thread_.join();
  }
}

void TestRadarReceiver::verify_pcl(sensor_msgs::msg::PointCloud2 received_pcl)
{
  // Convert the PointCloud2 message to a PCL point cloud
  pcl::PointCloud<corner_radar_driver::PclPointLocation> pcl_cloud;
  pcl::fromROSMsg(received_pcl, pcl_cloud);

  // Check if each point from the pcl has value different than zero
  for (const auto & point : pcl_cloud.points) {
    bool check = false;
    if (point.x == 0 && point.y == 0 && point.z == 0) {
      check = true;
    }
    EXPECT_FALSE(check);
  }
}

void TestRadarReceiver::verify_locations(
  corner_radar_driver_msgs::msg::LocationArray test_locations,
  corner_radar_driver_msgs::msg::LocationArray received_locations)
{
  EXPECT_EQ(node_->count_publishers("from_can_bus_fd"), 0U);
  EXPECT_EQ(node_->count_subscribers("from_can_bus_fd"), 1U);
  EXPECT_EQ(node_->count_publishers("locations"), 1U);
  EXPECT_EQ(node_->count_subscribers("locations"), 1U);
  EXPECT_EQ(node_->count_publishers("locations_pcl"), 1U);
  EXPECT_EQ(node_->count_subscribers("locations_pcl"), 1U);

  // Check locations
  uint16_t found_location_ids = 0;
  for (corner_radar_driver_msgs::msg::Location received_location :
    received_locations.locations)
  {
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

    EXPECT_NEAR(
      received_location.location1.radial_distance,
      current_test_location.location1.radial_distance, 0.009);
    EXPECT_NEAR(
      received_location.location1.radial_velocity,
      current_test_location.location1.radial_velocity, 0.009);
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
    EXPECT_NEAR(received_location.location1.rcs, current_test_location.location1.rcs, 0.00009);
    EXPECT_NEAR(received_location.location1.rssi, current_test_location.location1.rssi, 0.009);
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
      received_location.location1.measurement_status,
      current_test_location.location1.measurement_status);

    EXPECT_NEAR(
      received_location.location2.radial_distance,
      current_test_location.location2.radial_distance, 0.009);
    EXPECT_NEAR(
      received_location.location2.radial_velocity,
      current_test_location.location2.radial_velocity, 0.009);
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
    EXPECT_NEAR(received_location.location2.rcs, current_test_location.location2.rcs, 0.00009);
    EXPECT_NEAR(received_location.location2.rssi, current_test_location.location2.rssi, 0.009);
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
    EXPECT_EQ(
      received_location.location2.measurement_status,
      current_test_location.location2.measurement_status);

    EXPECT_NEAR(
      received_location.location3.radial_distance,
      current_test_location.location3.radial_distance, 0.009);
    EXPECT_NEAR(
      received_location.location3.radial_velocity,
      current_test_location.location3.radial_velocity, 0.009);
    EXPECT_NEAR(
      received_location.location3.azimuth_angle,
      current_test_location.location3.azimuth_angle, 0.09);
    EXPECT_NEAR(
      received_location.location3.elevation_angle,
      current_test_location.location3.elevation_angle, 0.09);
    EXPECT_NEAR(
      received_location.location3.radial_distance_variance,
      current_test_location.location3.radial_distance_variance, 0.0009);
    EXPECT_NEAR(
      received_location.location3.radial_velocity_variance,
      current_test_location.location3.radial_velocity_variance, 0.0009);
    EXPECT_NEAR(
      received_location.location3.azimuth_angle_variance,
      current_test_location.location3.azimuth_angle_variance, 0.09);
    EXPECT_NEAR(
      received_location.location3.elevation_angle_variance,
      current_test_location.location3.elevation_angle_variance, 0.09);
    EXPECT_NEAR(
      received_location.location3.radial_distance_velocity_covariance,
      current_test_location.location3.radial_distance_velocity_covariance, 0.0009);
    EXPECT_NEAR(received_location.location3.rcs, current_test_location.location3.rcs, 0.00009);
    EXPECT_NEAR(received_location.location3.rssi, current_test_location.location3.rssi, 0.009);
    EXPECT_EQ(
      received_location.location3.radial_distance_velocity_quality,
      current_test_location.location3.radial_distance_velocity_quality);
    EXPECT_EQ(
      received_location.location3.azimuth_angle_quality,
      current_test_location.location3.azimuth_angle_quality);
    EXPECT_EQ(
      received_location.location3.elevation_angle_quality,
      current_test_location.location3.elevation_angle_quality);
    EXPECT_EQ(
      received_location.location3.azimuthal_partner_id,
      current_test_location.location3.azimuthal_partner_id);
    EXPECT_EQ(
      received_location.location3.measurement_status,
      current_test_location.location3.measurement_status);

    found_location_ids++;
  }
  EXPECT_EQ(test_locations.locations.size(), found_location_ids);
}

TEST_P(TestRadarReceiver, testRandomZeroAllLocations) {
  corner_radar_driver_msgs::msg::LocationArray test_locations;
  corner_radar_driver_msgs::msg::Location test_location;

  corner_radar_driver_msgs::msg::LocationArray filtered_test_locations;

  const size_t send_locations = 10;

  // Use time as seed to generate random values
  time_t current_time;
  std::time(&current_time);
  std::srand(current_time);

  // Randomize the number of locations that will have zero values
  // (std::rand() % (max - min + 1) + min)
  size_t zero_values = (std::rand() % ((send_locations / 2) + 1));

  uint16_t num_locations = 0;
  for (const auto & sensor_pair : sensors_config_) {
    const SensorInfo & sensor_info = sensor_pair.second;

    if (sensor_info.active) {
      num_locations += did_to_loc_number_.at(sensor_info.max_number_locations);
    }
  }

  for (uint16_t i = 0; i < num_locations; i++) {
    test_location.id = i;
    test_location.location1.radial_distance = 300.0;
    test_location.location1.radial_distance_variance = 0.01;
    test_location.location1.radial_velocity = -50.0;
    test_location.location1.radial_velocity_variance = 0.01;
    test_location.location1.radial_distance_velocity_covariance = 0.03;
    test_location.location1.radial_distance_velocity_quality = 120.0;
    test_location.location1.elevation_angle = 25.0 * kDegToRad;
    test_location.location1.elevation_angle_quality = 50.0;
    test_location.location1.elevation_angle_variance = 0.01 * kDegToRad * kDegToRad;
    test_location.location1.azimuth_angle = 45.0 * kDegToRad;
    test_location.location1.azimuth_angle_quality = 100.0;
    test_location.location1.azimuth_angle_variance = 0.05 * kDegToRad * kDegToRad;
    test_location.location1.azimuthal_partner_id = 24.0;
    test_location.location1.rcs = 70;
    test_location.location1.rssi = 12.5;
    test_location.location1.measurement_status = 4;

    test_location.location2.radial_distance = 250.0;
    test_location.location2.radial_distance_variance = 0.04;
    test_location.location2.radial_velocity = 40.0;
    test_location.location2.radial_velocity_variance = 0.001;
    test_location.location2.radial_distance_velocity_covariance = -0.03;
    test_location.location2.radial_distance_velocity_quality = 20.0;
    test_location.location2.elevation_angle = 37.7 * kDegToRad;
    test_location.location2.elevation_angle_quality = 10.0;
    test_location.location2.elevation_angle_variance = 0.01 * kDegToRad * kDegToRad;
    test_location.location2.azimuth_angle = -45.0 * kDegToRad;
    test_location.location2.azimuth_angle_quality = 250.0;
    test_location.location2.azimuth_angle_variance = 0.9 * kDegToRad * kDegToRad;
    test_location.location2.azimuthal_partner_id = 1020.0;
    test_location.location2.rcs = 25.8;
    test_location.location2.rssi = 59.0;
    test_location.location2.measurement_status = 10;

    test_location.location3.radial_distance = 5.8;
    test_location.location3.radial_distance_variance = 0.01;
    test_location.location3.radial_velocity = -50.0;
    test_location.location3.radial_velocity_variance = 0.01;
    test_location.location3.radial_distance_velocity_covariance = 0.03;
    test_location.location3.radial_distance_velocity_quality = 120.0;
    test_location.location3.elevation_angle = 25.0 * kDegToRad;
    test_location.location3.elevation_angle_quality = 50.0;
    test_location.location3.elevation_angle_variance = 0.01 * kDegToRad * kDegToRad;
    test_location.location3.azimuth_angle = 45.0 * kDegToRad;
    test_location.location3.azimuth_angle_quality = 100.0;
    test_location.location3.azimuth_angle_variance = 0.05 * kDegToRad * kDegToRad;
    test_location.location3.azimuthal_partner_id = 24.0;
    test_location.location3.rcs = 67.6;
    test_location.location3.rssi = 12.5;
    test_location.location3.measurement_status = 4;
    test_locations.locations.push_back(test_location);
  }

  for (size_t i = 0; i < zero_values; i++) {
    // (std::rand() % (max - min + 1) + min)
    size_t rand_pos = (std::rand() % (send_locations + 1));

    // pick random number again if the location is already zero assigned
    while (test_locations.locations[rand_pos].location1.radial_distance == 0 &&
      test_locations.locations[rand_pos].location2.radial_distance == 0 &&
      test_locations.locations[rand_pos].location3.radial_distance == 0)
    {
      rand_pos = (std::rand() % (send_locations + 1));
    }

    test_locations.locations[rand_pos].location1.radial_distance = 0;
    test_locations.locations[rand_pos].location2.radial_distance = 0;
    test_locations.locations[rand_pos].location3.radial_distance = 0;
  }

  locations_subscriber_->clear_messages();

  start_parallel_publishing(test_locations, kPublishFrequency, sensors_config_);

  while (pub_running_) {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(locations_subscriber_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  auto result = get_locations();

  // Filter test_locations to remove zero value ones for testing
  for (size_t i = 0; i < test_locations.locations.size(); i++) {
    if (test_locations.locations[i].location1.radial_distance != 0 ||
      test_locations.locations[i].location2.radial_distance != 0 ||
      test_locations.locations[i].location3.radial_distance != 0)
    {
      filtered_test_locations.locations.push_back(test_locations.locations[i]);
    }
  }

  stop_parallel_publishing();

  verify_locations(filtered_test_locations, get_locations());
  verify_pcl(get_pcl());
}

TEST_P(TestRadarReceiver, testRandomZeroLocation1) {
  corner_radar_driver_msgs::msg::LocationArray test_locations;
  corner_radar_driver_msgs::msg::Location test_location;

  corner_radar_driver_msgs::msg::LocationArray filtered_test_locations;

  const uint16_t send_locations = 10;

  // Use time as seed to generate random values
  time_t current_time;
  std::time(&current_time);
  std::srand(current_time);

  // Randomize the number of locations that will have zero values
  // (std::rand() % (max - min + 1) + min)
  uint16_t zero_values = (std::rand() % ((send_locations / 2) + 1));

  uint16_t num_locations = 0;
  for (const auto & sensor_pair : sensors_config_) {
    const SensorInfo & sensor_info = sensor_pair.second;

    if (sensor_info.active) {
      num_locations += did_to_loc_number_.at(sensor_info.max_number_locations);
    }
  }

  for (uint16_t i = 0; i < num_locations; i++) {
    test_location.id = i;
    test_location.location1.radial_distance = 300.0;
    test_location.location1.radial_distance_variance = 0.01;
    test_location.location1.radial_velocity = -50.0;
    test_location.location1.radial_velocity_variance = 0.01;
    test_location.location1.radial_distance_velocity_covariance = 0.03;
    test_location.location1.radial_distance_velocity_quality = 120.0;
    test_location.location1.elevation_angle = 25.0 * kDegToRad;
    test_location.location1.elevation_angle_quality = 50.0;
    test_location.location1.elevation_angle_variance = 0.01 * kDegToRad * kDegToRad;
    test_location.location1.azimuth_angle = 45.0 * kDegToRad;
    test_location.location1.azimuth_angle_quality = 100.0;
    test_location.location1.azimuth_angle_variance = 0.05 * kDegToRad * kDegToRad;
    test_location.location1.azimuthal_partner_id = 24.0;
    test_location.location1.rcs = 70;
    test_location.location1.rssi = 12.5;
    test_location.location1.measurement_status = 4;

    test_location.location2.radial_distance = 250.0;
    test_location.location2.radial_distance_variance = 0.04;
    test_location.location2.radial_velocity = 40.0;
    test_location.location2.radial_velocity_variance = 0.001;
    test_location.location2.radial_distance_velocity_covariance = -0.03;
    test_location.location2.radial_distance_velocity_quality = 20.0;
    test_location.location2.elevation_angle = 37.7 * kDegToRad;
    test_location.location2.elevation_angle_quality = 10.0;
    test_location.location2.elevation_angle_variance = 0.01 * kDegToRad * kDegToRad;
    test_location.location2.azimuth_angle = -45.0 * kDegToRad;
    test_location.location2.azimuth_angle_quality = 250.0;
    test_location.location2.azimuth_angle_variance = 0.9 * kDegToRad * kDegToRad;
    test_location.location2.azimuthal_partner_id = 1020.0;
    test_location.location2.rcs = 25.8;
    test_location.location2.rssi = 59.0;
    test_location.location2.measurement_status = 10;

    test_location.location3.radial_distance = 5.8;
    test_location.location3.radial_distance_variance = 0.01;
    test_location.location3.radial_velocity = -50.0;
    test_location.location3.radial_velocity_variance = 0.01;
    test_location.location3.radial_distance_velocity_covariance = 0.03;
    test_location.location3.radial_distance_velocity_quality = 120.0;
    test_location.location3.elevation_angle = 25.0 * kDegToRad;
    test_location.location3.elevation_angle_quality = 50.0;
    test_location.location3.elevation_angle_variance = 0.01 * kDegToRad * kDegToRad;
    test_location.location3.azimuth_angle = 45.0 * kDegToRad;
    test_location.location3.azimuth_angle_quality = 100.0;
    test_location.location3.azimuth_angle_variance = 0.05 * kDegToRad * kDegToRad;
    test_location.location3.azimuthal_partner_id = 24.0;
    test_location.location3.rcs = 67.6;
    test_location.location3.rssi = 12.5;
    test_location.location3.measurement_status = 4;
    test_locations.locations.push_back(test_location);
  }

  for (uint16_t i = 0; i < zero_values; i++) {
    // (std::rand() % (max - min + 1) + min)
    uint16_t rand_pos = (std::rand() % (send_locations + 1));

    // pick random number again if the location is already zero assigned
    while (test_locations.locations[rand_pos].location1.radial_distance == 0) {
      rand_pos = (std::rand() % (send_locations + 1));
    }

    test_locations.locations[rand_pos].location1.radial_distance = 0;
  }

  locations_subscriber_->clear_messages();

  start_parallel_publishing(test_locations, kPublishFrequency, sensors_config_);

  while (pub_running_) {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(locations_subscriber_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  auto result = get_locations();

  // Filter test_locations to remove zero value ones for testing
  for (uint16_t i = 0; i < test_locations.locations.size(); i++) {
    if (test_locations.locations[i].location1.radial_distance != 0 ||
      test_locations.locations[i].location2.radial_distance != 0 ||
      test_locations.locations[i].location3.radial_distance != 0)
    {
      filtered_test_locations.locations.push_back(test_locations.locations[i]);
    }
  }

  stop_parallel_publishing();

  verify_locations(filtered_test_locations, result);
  verify_pcl(get_pcl());
}

TEST_P(TestRadarReceiver, testRandomZeroLocation2) {
  corner_radar_driver_msgs::msg::LocationArray test_locations;
  corner_radar_driver_msgs::msg::Location test_location;

  corner_radar_driver_msgs::msg::LocationArray filtered_test_locations;

  const uint16_t send_locations = 10;

  // Use time as seed to generate random values
  time_t current_time;
  std::time(&current_time);
  std::srand(current_time);

  // Randomize the number of locations that will have zero values
  // (std::rand() % (max - min + 1) + min)
  uint16_t zero_values = (std::rand() % ((send_locations / 2) + 1));

  uint16_t num_locations = 0;
  for (const auto & sensor_pair : sensors_config_) {
    const SensorInfo & sensor_info = sensor_pair.second;

    if (sensor_info.active) {
      num_locations += did_to_loc_number_.at(sensor_info.max_number_locations);
    }
  }

  for (uint16_t i = 0; i < num_locations; i++) {
    test_location.id = i;
    test_location.location1.radial_distance = 300.0;
    test_location.location1.radial_distance_variance = 0.01;
    test_location.location1.radial_velocity = -50.0;
    test_location.location1.radial_velocity_variance = 0.01;
    test_location.location1.radial_distance_velocity_covariance = 0.03;
    test_location.location1.radial_distance_velocity_quality = 120.0;
    test_location.location1.elevation_angle = 25.0 * kDegToRad;
    test_location.location1.elevation_angle_quality = 50.0;
    test_location.location1.elevation_angle_variance = 0.01 * kDegToRad * kDegToRad;
    test_location.location1.azimuth_angle = 45.0 * kDegToRad;
    test_location.location1.azimuth_angle_quality = 100.0;
    test_location.location1.azimuth_angle_variance = 0.05 * kDegToRad * kDegToRad;
    test_location.location1.azimuthal_partner_id = 24.0;
    test_location.location1.rcs = 70;
    test_location.location1.rssi = 12.5;
    test_location.location1.measurement_status = 4;

    test_location.location2.radial_distance = 250.0;
    test_location.location2.radial_distance_variance = 0.04;
    test_location.location2.radial_velocity = 40.0;
    test_location.location2.radial_velocity_variance = 0.001;
    test_location.location2.radial_distance_velocity_covariance = -0.03;
    test_location.location2.radial_distance_velocity_quality = 20.0;
    test_location.location2.elevation_angle = 37.7 * kDegToRad;
    test_location.location2.elevation_angle_quality = 10.0;
    test_location.location2.elevation_angle_variance = 0.01 * kDegToRad * kDegToRad;
    test_location.location2.azimuth_angle = -45.0 * kDegToRad;
    test_location.location2.azimuth_angle_quality = 250.0;
    test_location.location2.azimuth_angle_variance = 0.9 * kDegToRad * kDegToRad;
    test_location.location2.azimuthal_partner_id = 1020.0;
    test_location.location2.rcs = 25.8;
    test_location.location2.rssi = 59.0;
    test_location.location2.measurement_status = 10;

    test_location.location3.radial_distance = 5.8;
    test_location.location3.radial_distance_variance = 0.01;
    test_location.location3.radial_velocity = -50.0;
    test_location.location3.radial_velocity_variance = 0.01;
    test_location.location3.radial_distance_velocity_covariance = 0.03;
    test_location.location3.radial_distance_velocity_quality = 120.0;
    test_location.location3.elevation_angle = 25.0 * kDegToRad;
    test_location.location3.elevation_angle_quality = 50.0;
    test_location.location3.elevation_angle_variance = 0.01 * kDegToRad * kDegToRad;
    test_location.location3.azimuth_angle = 45.0 * kDegToRad;
    test_location.location3.azimuth_angle_quality = 100.0;
    test_location.location3.azimuth_angle_variance = 0.05 * kDegToRad * kDegToRad;
    test_location.location3.azimuthal_partner_id = 24.0;
    test_location.location3.rcs = 67.6;
    test_location.location3.rssi = 12.5;
    test_location.location3.measurement_status = 4;
    test_locations.locations.push_back(test_location);
  }

  for (uint16_t i = 0; i < zero_values; i++) {
    // (std::rand() % (max - min + 1) + min)
    uint16_t rand_pos = (std::rand() % (send_locations + 1));

    // pick random number again if the location is already zero assigned
    while (test_locations.locations[rand_pos].location2.radial_distance == 0) {
      rand_pos = (std::rand() % (send_locations + 1));
    }

    test_locations.locations[rand_pos].location2.radial_distance = 0;
  }

  locations_subscriber_->clear_messages();

  start_parallel_publishing(test_locations, kPublishFrequency, sensors_config_);

  while (pub_running_) {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(locations_subscriber_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  auto result = get_locations();

  // Filter test_locations to remove zero value ones for testing
  for (uint16_t i = 0; i < test_locations.locations.size(); i++) {
    if (test_locations.locations[i].location1.radial_distance != 0 ||
      test_locations.locations[i].location2.radial_distance != 0 ||
      test_locations.locations[i].location3.radial_distance != 0)
    {
      filtered_test_locations.locations.push_back(test_locations.locations[i]);
    }
  }

  stop_parallel_publishing();

  verify_locations(filtered_test_locations, result);
  verify_pcl(get_pcl());
}

TEST_P(TestRadarReceiver, testRandomZeroLocation3) {
  corner_radar_driver_msgs::msg::LocationArray test_locations;
  corner_radar_driver_msgs::msg::Location test_location;

  corner_radar_driver_msgs::msg::LocationArray filtered_test_locations;

  const uint16_t send_locations = 10;

  // Use time as seed to generate random values
  time_t current_time;
  std::time(&current_time);
  std::srand(current_time);

  // Randomize the number of locations that will have zero values
  // (std::rand() % (max - min + 1) + min)
  uint16_t zero_values = (std::rand() % ((send_locations / 2) + 1));

  uint16_t num_locations = 0;
  for (const auto & sensor_pair : sensors_config_) {
    const SensorInfo & sensor_info = sensor_pair.second;

    if (sensor_info.active) {
      num_locations += did_to_loc_number_.at(sensor_info.max_number_locations);
    }
  }

  for (uint16_t i = 0; i < num_locations; i++) {
    test_location.id = i;
    test_location.location1.radial_distance = 300.0;
    test_location.location1.radial_distance_variance = 0.01;
    test_location.location1.radial_velocity = -50.0;
    test_location.location1.radial_velocity_variance = 0.01;
    test_location.location1.radial_distance_velocity_covariance = 0.03;
    test_location.location1.radial_distance_velocity_quality = 120.0;
    test_location.location1.elevation_angle = 25.0 * kDegToRad;
    test_location.location1.elevation_angle_quality = 50.0;
    test_location.location1.elevation_angle_variance = 0.01 * kDegToRad * kDegToRad;
    test_location.location1.azimuth_angle = 45.0 * kDegToRad;
    test_location.location1.azimuth_angle_quality = 100.0;
    test_location.location1.azimuth_angle_variance = 0.05 * kDegToRad * kDegToRad;
    test_location.location1.azimuthal_partner_id = 24.0;
    test_location.location1.rcs = 70;
    test_location.location1.rssi = 12.5;
    test_location.location1.measurement_status = 4;

    test_location.location2.radial_distance = 250.0;
    test_location.location2.radial_distance_variance = 0.04;
    test_location.location2.radial_velocity = 40.0;
    test_location.location2.radial_velocity_variance = 0.001;
    test_location.location2.radial_distance_velocity_covariance = -0.03;
    test_location.location2.radial_distance_velocity_quality = 20.0;
    test_location.location2.elevation_angle = 37.7 * kDegToRad;
    test_location.location2.elevation_angle_quality = 10.0;
    test_location.location2.elevation_angle_variance = 0.01 * kDegToRad * kDegToRad;
    test_location.location2.azimuth_angle = -45.0 * kDegToRad;
    test_location.location2.azimuth_angle_quality = 250.0;
    test_location.location2.azimuth_angle_variance = 0.9 * kDegToRad * kDegToRad;
    test_location.location2.azimuthal_partner_id = 1020.0;
    test_location.location2.rcs = 25.8;
    test_location.location2.rssi = 59.0;
    test_location.location2.measurement_status = 10;

    test_location.location3.radial_distance = 5.8;
    test_location.location3.radial_distance_variance = 0.01;
    test_location.location3.radial_velocity = -50.0;
    test_location.location3.radial_velocity_variance = 0.01;
    test_location.location3.radial_distance_velocity_covariance = 0.03;
    test_location.location3.radial_distance_velocity_quality = 120.0;
    test_location.location3.elevation_angle = 25.0 * kDegToRad;
    test_location.location3.elevation_angle_quality = 50.0;
    test_location.location3.elevation_angle_variance = 0.01 * kDegToRad * kDegToRad;
    test_location.location3.azimuth_angle = 45.0 * kDegToRad;
    test_location.location3.azimuth_angle_quality = 100.0;
    test_location.location3.azimuth_angle_variance = 0.05 * kDegToRad * kDegToRad;
    test_location.location3.azimuthal_partner_id = 24.0;
    test_location.location3.rcs = 67.6;
    test_location.location3.rssi = 12.5;
    test_location.location3.measurement_status = 4;
    test_locations.locations.push_back(test_location);
  }

  for (uint16_t i = 0; i < zero_values; i++) {
    // (std::rand() % (max - min + 1) + min)
    uint16_t rand_pos = (std::rand() % (send_locations + 1));

    // pick random number again if the location is already zero assigned
    while (test_locations.locations[rand_pos].location3.radial_distance == 0) {
      rand_pos = (std::rand() % (send_locations + 1));
    }

    test_locations.locations[rand_pos].location3.radial_distance = 0;
  }

  locations_subscriber_->clear_messages();

  start_parallel_publishing(test_locations, kPublishFrequency, sensors_config_);

  while (pub_running_) {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(locations_subscriber_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  auto result = get_locations();

  // Filter test_locations to remove zero value ones for testing
  for (uint16_t i = 0; i < test_locations.locations.size(); i++) {
    if (test_locations.locations[i].location1.radial_distance != 0 ||
      test_locations.locations[i].location2.radial_distance != 0 ||
      test_locations.locations[i].location3.radial_distance != 0)
    {
      filtered_test_locations.locations.push_back(test_locations.locations[i]);
    }
  }

  stop_parallel_publishing();

  verify_locations(filtered_test_locations, result);
  verify_pcl(get_pcl());
}

TEST_P(TestRadarReceiver, testLocationsValidValues) {
  corner_radar_driver_msgs::msg::LocationArray test_locations;
  corner_radar_driver_msgs::msg::Location test_location;
  corner_radar_driver_msgs::msg::LocationArray expected_locations_test;

  uint16_t num_locations = 0;
  for (const auto & sensor_pair : sensors_config_) {
    const SensorInfo & sensor_info = sensor_pair.second;

    if (sensor_info.active) {
      num_locations += did_to_loc_number_.at(sensor_info.max_number_locations);
    }
  }

  for (uint16_t i = 0; i < num_locations; i++) {
    test_location.id = i;
    test_location.location1.radial_distance = 300.0;
    test_location.location1.radial_distance_variance = 0.01;
    test_location.location1.radial_velocity = -50.0;
    test_location.location1.radial_velocity_variance = 0.01;
    test_location.location1.radial_distance_velocity_covariance = 0.03;
    test_location.location1.radial_distance_velocity_quality = 120.0;
    test_location.location1.elevation_angle = 25.0 * kDegToRad;
    test_location.location1.elevation_angle_quality = 50.0;
    test_location.location1.elevation_angle_variance = 0.01 * kDegToRad * kDegToRad;
    test_location.location1.azimuth_angle = 45.0 * kDegToRad;
    test_location.location1.azimuth_angle_quality = 100.0;
    test_location.location1.azimuth_angle_variance = 0.05 * kDegToRad * kDegToRad;
    test_location.location1.azimuthal_partner_id = 24.0;
    test_location.location1.rcs = 70;
    test_location.location1.rssi = 12.5;
    test_location.location1.measurement_status = 4;

    test_location.location2.radial_distance = 250.0;
    test_location.location2.radial_distance_variance = 0.04;
    test_location.location2.radial_velocity = 40.0;
    test_location.location2.radial_velocity_variance = 0.001;
    test_location.location2.radial_distance_velocity_covariance = -0.03;
    test_location.location2.radial_distance_velocity_quality = 20.0;
    test_location.location2.elevation_angle = 37.7 * kDegToRad;
    test_location.location2.elevation_angle_quality = 10.0;
    test_location.location2.elevation_angle_variance = 0.01 * kDegToRad * kDegToRad;
    test_location.location2.azimuth_angle = -45.0 * kDegToRad;
    test_location.location2.azimuth_angle_quality = 250.0;
    test_location.location2.azimuth_angle_variance = 0.9 * kDegToRad * kDegToRad;
    test_location.location2.azimuthal_partner_id = 1020.0;
    test_location.location2.rcs = 25.8;
    test_location.location2.rssi = 59.0;
    test_location.location2.measurement_status = 10;

    test_location.location3.radial_distance = 5.8;
    test_location.location3.radial_distance_variance = 0.01;
    test_location.location3.radial_velocity = -50.0;
    test_location.location3.radial_velocity_variance = 0.01;
    test_location.location3.radial_distance_velocity_covariance = 0.03;
    test_location.location3.radial_distance_velocity_quality = 120.0;
    test_location.location3.elevation_angle = 25.0 * kDegToRad;
    test_location.location3.elevation_angle_quality = 50.0;
    test_location.location3.elevation_angle_variance = 0.01 * kDegToRad * kDegToRad;
    test_location.location3.azimuth_angle = 45.0 * kDegToRad;
    test_location.location3.azimuth_angle_quality = 100.0;
    test_location.location3.azimuth_angle_variance = 0.05 * kDegToRad * kDegToRad;
    test_location.location3.azimuthal_partner_id = 24.0;
    test_location.location3.rcs = 67.6;
    test_location.location3.rssi = 12.5;
    test_location.location3.measurement_status = 4;

    test_locations.locations.push_back(test_location);
  }

  locations_subscriber_->clear_messages();

  start_parallel_publishing(test_locations, kPublishFrequency, sensors_config_);

  while (pub_running_) {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(locations_subscriber_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  auto result = get_locations();

  stop_parallel_publishing();

  verify_locations(test_locations, result);
}

TEST_P(TestRadarReceiver, testLocationsMinValues) {
  corner_radar_driver_msgs::msg::LocationArray test_locations;
  corner_radar_driver_msgs::msg::Location test_location;

  uint16_t num_locations = 0;
  for (const auto & sensor_pair : sensors_config_) {
    const SensorInfo & sensor_info = sensor_pair.second;

    if (sensor_info.active) {
      num_locations += did_to_loc_number_.at(sensor_info.max_number_locations);
    }
  }

  for (uint16_t i = 0; i < num_locations; i++) {
    test_location.id = i;
    test_location.location1.radial_distance = 0.1;
    test_location.location1.radial_distance_variance = 0.0;
    test_location.location1.radial_velocity = -110.0;
    test_location.location1.radial_velocity_variance = 0.0;
    test_location.location1.radial_distance_velocity_covariance = -0.1024;
    test_location.location1.radial_distance_velocity_quality = 0.0;
    test_location.location1.elevation_angle = -45.0 * kDegToRad;
    test_location.location1.elevation_angle_quality = 0.0;
    test_location.location1.elevation_angle_variance = 0.0;
    test_location.location1.azimuth_angle = -90.0 * kDegToRad;
    test_location.location1.azimuth_angle_quality = 0.0;
    test_location.location1.azimuth_angle_variance = 0.0;
    test_location.location1.azimuthal_partner_id = 0.0;
    test_location.location1.rcs = -50.0;
    test_location.location1.rssi = 0.0;
    test_location.location1.measurement_status = 0.0;

    test_location.location2.radial_distance = 0.1;
    test_location.location2.radial_distance_variance = 0.0;
    test_location.location2.radial_velocity = -110.0;
    test_location.location2.radial_velocity_variance = 0.0;
    test_location.location2.radial_distance_velocity_covariance = -0.1024;
    test_location.location2.radial_distance_velocity_quality = 0.0;
    test_location.location2.elevation_angle = -45.0 * kDegToRad;
    test_location.location2.elevation_angle_quality = 0.0;
    test_location.location2.elevation_angle_variance = 0.0;
    test_location.location2.azimuth_angle = -90.0 * kDegToRad;
    test_location.location2.azimuth_angle_quality = 0.0;
    test_location.location2.azimuth_angle_variance = 0.0;
    test_location.location2.azimuthal_partner_id = 0.0;
    test_location.location2.rcs = -50.0;
    test_location.location2.rssi = 0.0;
    test_location.location2.measurement_status = 0.0;

    test_location.location3.radial_distance = 0.1;
    test_location.location3.radial_distance_variance = 0.0;
    test_location.location3.radial_velocity = -110.0;
    test_location.location3.radial_velocity_variance = 0.0;
    test_location.location3.radial_distance_velocity_covariance = -0.1024;
    test_location.location3.radial_distance_velocity_quality = 0.0;
    test_location.location3.elevation_angle = -45.0 * kDegToRad;
    test_location.location3.elevation_angle_quality = 0.0;
    test_location.location3.elevation_angle_variance = 0.0;
    test_location.location3.azimuth_angle = -90.0 * kDegToRad;
    test_location.location3.azimuth_angle_quality = 0.0;
    test_location.location3.azimuth_angle_variance = 0.0;
    test_location.location3.azimuthal_partner_id = 0.0;
    test_location.location3.rcs = -50.0;
    test_location.location3.rssi = 0.0;
    test_location.location3.measurement_status = 0.0;

    test_locations.locations.push_back(test_location);
  }

  locations_subscriber_->clear_messages();

  start_parallel_publishing(test_locations, kPublishFrequency, sensors_config_);

  while (pub_running_) {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(locations_subscriber_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  auto result = get_locations();

  stop_parallel_publishing();

  verify_locations(test_locations, result);
}

TEST_P(TestRadarReceiver, testLocationsMaxValues) {
  corner_radar_driver_msgs::msg::LocationArray test_locations;
  corner_radar_driver_msgs::msg::Location test_location;

  uint16_t num_locations = 0;
  for (const auto & sensor_pair : sensors_config_) {
    const SensorInfo & sensor_info = sensor_pair.second;

    if (sensor_info.active) {
      num_locations += did_to_loc_number_.at(sensor_info.max_number_locations);
    }
  }

  for (uint16_t i = 0; i < num_locations; i++) {
    test_location.id = i;
    test_location.location1.radial_distance = 327.67;
    test_location.location1.radial_distance_variance = 0.05115;
    test_location.location1.radial_velocity = 55.0;
    test_location.location1.radial_velocity_variance = 0.1023;
    test_location.location1.radial_distance_velocity_covariance = 0.1023;
    test_location.location1.radial_distance_velocity_quality = 255.0;
    test_location.location1.elevation_angle = 45.0 * kDegToRad;
    test_location.location1.elevation_angle_quality = 255.0;
    test_location.location1.elevation_angle_variance = 1.023 * kDegToRad * kDegToRad;
    test_location.location1.azimuth_angle = 90.0 * kDegToRad;
    test_location.location1.azimuth_angle_quality = 255.0;
    test_location.location1.azimuth_angle_variance = 1.023 * kDegToRad * kDegToRad;
    test_location.location1.azimuthal_partner_id = 1023.0;
    test_location.location1.rcs = 70.0;
    test_location.location1.rssi = 100.0;
    test_location.location1.measurement_status = 15;

    test_location.location2.radial_distance = 327.67;
    test_location.location2.radial_distance_variance = 0.05115;
    test_location.location2.radial_velocity = 55.0;
    test_location.location2.radial_velocity_variance = 0.1023;
    test_location.location2.radial_distance_velocity_covariance = 0.1023;
    test_location.location2.radial_distance_velocity_quality = 255.0;
    test_location.location2.elevation_angle = 45.0 * kDegToRad;
    test_location.location2.elevation_angle_quality = 255.0;
    test_location.location2.elevation_angle_variance = 1.023 * kDegToRad * kDegToRad;
    test_location.location2.azimuth_angle = 90.0 * kDegToRad;
    test_location.location2.azimuth_angle_quality = 255.0;
    test_location.location2.azimuth_angle_variance = 1.023 * kDegToRad * kDegToRad;
    test_location.location2.azimuthal_partner_id = 1023.0;
    test_location.location2.rcs = 70.0;
    test_location.location2.rssi = 100.0;
    test_location.location2.measurement_status = 15;

    test_location.location3.radial_distance = 327.67;
    test_location.location3.radial_distance_variance = 0.05115;
    test_location.location3.radial_velocity = 55.0;
    test_location.location3.radial_velocity_variance = 0.1023;
    test_location.location3.radial_distance_velocity_covariance = 0.1023;
    test_location.location3.radial_distance_velocity_quality = 255.0;
    test_location.location3.elevation_angle = 45.0 * kDegToRad;
    test_location.location3.elevation_angle_quality = 255.0;
    test_location.location3.elevation_angle_variance = 1.023 * kDegToRad * kDegToRad;
    test_location.location3.azimuth_angle = 90.0 * kDegToRad;
    test_location.location3.azimuth_angle_quality = 255.0;
    test_location.location3.azimuth_angle_variance = 1.023 * kDegToRad * kDegToRad;
    test_location.location3.azimuthal_partner_id = 1023.0;
    test_location.location3.rcs = 70.0;
    test_location.location3.rssi = 100.0;
    test_location.location3.measurement_status = 15;

    test_locations.locations.push_back(test_location);
  }

  locations_subscriber_->clear_messages();

  start_parallel_publishing(test_locations, kPublishFrequency, sensors_config_);

  while (pub_running_) {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(locations_subscriber_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  auto result = get_locations();

  stop_parallel_publishing();

  verify_locations(test_locations, result);
}

TEST_P(TestRadarReceiver, testRandomValidLocations) {
  corner_radar_driver_msgs::msg::LocationArray test_locations;
  corner_radar_driver_msgs::msg::Location test_location;

  uint16_t num_locations = 0;
  for (const auto & sensor_pair : sensors_config_) {
    const SensorInfo & sensor_info = sensor_pair.second;

    if (sensor_info.active) {
      num_locations += did_to_loc_number_.at(sensor_info.max_number_locations);
    }
  }

  // Create vector with unique IDs
  std::vector<uint16_t> ids(num_locations);
  std::iota(std::begin(ids), std::end(ids), 0);
  std::default_random_engine rng;
  std::shuffle(std::begin(ids), std::end(ids), rng);

  for (uint16_t i = 0; i < num_locations; i++) {
    test_location.id = i;
    test_location.location1.radial_distance =
      RandomQuantizedGenerator{0.01, 0.1, 327.67}(rng);
    test_location.location1.radial_distance_variance =
      RandomQuantizedGenerator{5e-005, 0.0, 0.05115}(rng);
    test_location.location1.radial_velocity = RandomQuantizedGenerator{0.01, -110.0, 55.0}(rng);
    test_location.location1.radial_velocity_variance =
      RandomQuantizedGenerator{0.0001, 0.0, 0.1023}(rng);
    test_location.location1.radial_distance_velocity_covariance =
      RandomQuantizedGenerator{0.0001, -0.1024, 0.1023}(rng);
    test_location.location1.radial_distance_velocity_quality =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}(rng);
    test_location.location1.elevation_angle =
      RandomQuantizedGenerator{0.1 * kDegToRad, -45.0 * kDegToRad, 45.0 * kDegToRad}(rng);
    test_location.location1.elevation_angle_quality =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}(rng);
    test_location.location1.elevation_angle_variance =
      RandomQuantizedGenerator{0.001 * kDegToRad * kDegToRad, 0.0,
      1.023 * kDegToRad * kDegToRad}(rng);
    test_location.location1.azimuth_angle =
      RandomQuantizedGenerator{0.1 * kDegToRad, -90.0 * kDegToRad, 90.0 * kDegToRad}(rng);
    test_location.location1.azimuth_angle_quality = RandomQuantizedGenerator{1.0, 0.0, 255.0}(rng);
    test_location.location1.azimuth_angle_variance =
      RandomQuantizedGenerator{0.001 * kDegToRad * kDegToRad, 0.0,
      1.023 * kDegToRad * kDegToRad}(rng);
    test_location.location1.azimuthal_partner_id = RandomQuantizedGenerator{1.0, 0.0, 1023.0}(rng);
    test_location.location1.rcs = RandomQuantizedGenerator{0.2, -50.0, 70.0}(rng);
    test_location.location1.rssi = RandomQuantizedGenerator{0.1, 0.0, 100.0}(rng);
    test_location.location1.measurement_status = RandomQuantizedGenerator{1.0, 0.0, 15.0}(rng);

    test_location.location2.radial_distance =
      RandomQuantizedGenerator{0.01, 0.1, 327.67}(rng);
    test_location.location2.radial_distance_variance =
      RandomQuantizedGenerator{5e-005, 0.0, 0.05115}(rng);
    test_location.location2.radial_velocity = RandomQuantizedGenerator{0.01, -110.0, 55.0}(rng);
    test_location.location2.radial_velocity_variance =
      RandomQuantizedGenerator{0.0001, 0.0, 0.1023}(rng);
    test_location.location2.radial_distance_velocity_covariance =
      RandomQuantizedGenerator{0.0001, -0.1024, 0.1023}(rng);
    test_location.location2.radial_distance_velocity_quality =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}(rng);
    test_location.location2.elevation_angle =
      RandomQuantizedGenerator{0.1 * kDegToRad, -45.0 * kDegToRad, 45.0 * kDegToRad}(rng);
    test_location.location2.elevation_angle_quality =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}(rng);
    test_location.location2.elevation_angle_variance =
      RandomQuantizedGenerator{0.001 * kDegToRad * kDegToRad, 0.0,
      1.023 * kDegToRad * kDegToRad}(rng);
    test_location.location2.azimuth_angle =
      RandomQuantizedGenerator{0.1 * kDegToRad, -90.0 * kDegToRad, 90.0 * kDegToRad}(rng);
    test_location.location2.azimuth_angle_quality = RandomQuantizedGenerator{1.0, 0.0, 255.0}(rng);
    test_location.location2.azimuth_angle_variance =
      RandomQuantizedGenerator{0.001 * kDegToRad * kDegToRad, 0.0,
      1.023 * kDegToRad * kDegToRad}(rng);
    test_location.location2.azimuthal_partner_id = RandomQuantizedGenerator{1.0, 0.0, 1023.0}(rng);
    test_location.location2.rcs = RandomQuantizedGenerator{0.2, -50.0, 70.0}(rng);
    test_location.location2.rssi = RandomQuantizedGenerator{0.1, 0.0, 100.0}(rng);
    test_location.location2.measurement_status = RandomQuantizedGenerator{1.0, 0.0, 15.0}(rng);

    test_location.location3.radial_distance =
      RandomQuantizedGenerator{0.01, 0.1, 327.67}(rng);
    test_location.location3.radial_distance_variance =
      RandomQuantizedGenerator{5e-005, 0.0, 0.05115}(rng);
    test_location.location3.radial_velocity = RandomQuantizedGenerator{0.01, -110.0, 55.0}(rng);
    test_location.location3.radial_velocity_variance =
      RandomQuantizedGenerator{0.0001, 0.0, 0.1023}(rng);
    test_location.location3.radial_distance_velocity_covariance =
      RandomQuantizedGenerator{0.0001, -0.1024, 0.1023}(rng);
    test_location.location3.radial_distance_velocity_quality =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}(rng);
    test_location.location3.elevation_angle =
      RandomQuantizedGenerator{0.1 * kDegToRad, -45.0 * kDegToRad, 45.0 * kDegToRad}(rng);
    test_location.location3.elevation_angle_quality =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}(rng);
    test_location.location3.elevation_angle_variance =
      RandomQuantizedGenerator{0.001 * kDegToRad * kDegToRad, 0.0,
      1.023 * kDegToRad * kDegToRad}(rng);
    test_location.location3.azimuth_angle =
      RandomQuantizedGenerator{0.1 * kDegToRad, -90.0 * kDegToRad, 90.0 * kDegToRad}(rng);
    test_location.location3.azimuth_angle_quality = RandomQuantizedGenerator{1.0, 0.0, 255.0}(rng);
    test_location.location3.azimuth_angle_variance =
      RandomQuantizedGenerator{0.001 * kDegToRad * kDegToRad, 0.0,
      1.023 * kDegToRad * kDegToRad}(rng);
    test_location.location3.azimuthal_partner_id = RandomQuantizedGenerator{1.0, 0.0, 1023.0}(rng);
    test_location.location3.rcs = RandomQuantizedGenerator{0.2, -50.0, 70.0}(rng);
    test_location.location3.rssi = RandomQuantizedGenerator{0.1, 0.0, 100.0}(rng);
    test_location.location3.measurement_status = RandomQuantizedGenerator{1.0, 0.0, 15.0}(rng);

    test_locations.locations.push_back(test_location);
  }

  locations_subscriber_->clear_messages();

  start_parallel_publishing(test_locations, kPublishFrequency, sensors_config_);

  while (pub_running_) {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(locations_subscriber_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  auto result = get_locations();

  stop_parallel_publishing();

  verify_locations(test_locations, result);
}

INSTANTIATE_TEST_SUITE_P(
  SensorConfigs,
  TestRadarReceiver,
  ::testing::Values(
    SensorConfig{
  {"sensor1", {
      true, 0x01, 0x18FF04B0,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor2", {
      false, 0x01, 0x18FF04B1,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor3", {
      false, 0x01, 0x18FF04B2,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor4", {
      false, 0x01, 0x18FF04B3,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }}
},
    SensorConfig{
  {"sensor1", {
      true, 0x02, 0x18FF04B0,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor2", {
      true, 0x02, 0x18FF04B1,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor3", {
      false, 0x01, 0x18FF04B2,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor4", {
      false, 0x01, 0x18FF04B3,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }}
},
    SensorConfig{
  {"sensor1", {
      true, 0x03, 0x18FF04B0,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor2", {
      true, 0x03, 0x18FF04B1,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor3", {
      true, 0x03, 0x18FF04B2,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor4", {
      false, 0x01, 0x18FF04B3,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }}
},
    SensorConfig{
  {"sensor1", {
      true, 0x04, 0x18FF04B0,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor2", {
      true, 0x04, 0x18FF04B1,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor3", {
      true, 0x04, 0x18FF04B2,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor4", {
      true, 0x04, 0x18FF04B3,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }}
},
    SensorConfig{
  {"sensor1", {
      true, 0x01, 0x18FF04B0,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor2", {
      true, 0x01, 0x18FF04B1,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor3", {
      false, 0x01, 0x18FF04B2,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor4", {
      false, 0x01, 0x18FF04B3,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }}
},
    SensorConfig{
  {"sensor1", {
      true, 0x01, 0x18FF04B0,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor2", {
      true, 0x01, 0x18FF04B1,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor3", {
      true, 0x01, 0x18FF04B2,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor4", {
      false, 0x01, 0x18FF04B3,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }}
},
    SensorConfig{
  {"sensor1", {
      true, 0x01, 0x18FF04B0,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor2", {
      true, 0x01, 0x18FF04B1,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor3", {
      true, 0x01, 0x18FF04B2,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor4", {
      true, 0x01, 0x18FF04B3,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }}
},
    SensorConfig{
  {"sensor1", {
      true, 0x01, 0x18FF04B0,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor2", {
      true, 0x02, 0x18FF04B1,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor3", {
      true, 0x03, 0x18FF04B2,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor4", {
      true, 0x04, 0x18FF04B3,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }}
},
    SensorConfig{
  {"sensor1", {
      false, 0x01, 0x18FF04B0,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor2", {
      true, 0x01, 0x18FF04B1,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor3", {
      false, 0x01, 0x18FF04B2,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor4", {
      false, 0x01, 0x18FF04B3,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }}
},
    SensorConfig{
  {"sensor1", {
      false, 0x01, 0x18FF04B0,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor2", {
      false, 0x01, 0x18FF04B1,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor3", {
      true, 0x01, 0x18FF04B2,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor4", {
      false, 0x01, 0x18FF04B3,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }}
},
    SensorConfig{
  {"sensor1", {
      false, 0x01, 0x18FF04B0,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor2", {
      false, 0x01, 0x18FF04B1,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor3", {
      false, 0x01, 0x18FF04B2,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }},
  {"sensor4", {
      true, 0x01, 0x18FF04B3,
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    }}
}
  )
);

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
