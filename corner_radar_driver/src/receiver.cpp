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


#include "corner_radar_driver/pcl_point_location.hpp"
#include "corner_radar_driver/receiver.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "off_highway_can/helper.hpp"


namespace corner_radar_driver
{

static constexpr double kDegToRad = std::numbers::pi / 180.0;

Receiver::Receiver(const rclcpp::NodeOptions & options)
: off_highway_can::Receiver("receiver", options, true),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  transform_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
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
  l.signals["block_counter"] = {24, 4, false, false, 1, 0};

  l.signals["l1_radial_distance"] = {32, 15, false, false, 0.01, 0};
  l.signals["l1_radial_velocity"] = {47, 15, false, false, 0.01, -163.84};
  l.signals["l1_elevation_angle"] = {62, 10, false, false, 0.1, -51.2};
  l.signals["l1_azimuth_angle"] = {72, 11, false, false, 0.1, -102.4};
  l.signals["l1_radial_distance_velocity_covariance"] = {83, 11, false, false, 0.0001, -0.1024};
  l.signals["l1_radial_distance_variance"] = {94, 10, false, false, 5e-005, 0};
  l.signals["l1_radial_velocity_variance"] = {104, 10, false, false, 0.0001, 0};
  l.signals["l1_elevation_angle_variance"] = {114, 10, false, false, 0.001, 0};
  l.signals["l1_azimuth_angle_variance"] = {124, 10, false, false, 0.001, 0};
  l.signals["l1_rcs"] = {134, 10, false, false, 0.2, -102.4};
  l.signals["l1_radial_distance_velocity_quality"] = {144, 8, false, false, 1, 0};
  l.signals["l1_elevation_angle_quality"] = {152, 8, false, false, 1, 0};
  l.signals["l1_azimuth_angle_quality"] = {160, 8, false, false, 1, 0};
  l.signals["l1_rssi"] = {168, 10, false, false, 0.1, 0};
  l.signals["l1_azimuthal_partner_id"] = {178, 10, false, false, 1, 0};
  l.signals["l1_measurement_status"] = {188, 4, false, false, 1, 0};

  l.signals["l2_radial_distance"] = {192, 15, false, false, 0.01, 0};
  l.signals["l2_radial_velocity"] = {207, 15, false, false, 0.01, -163.84};
  l.signals["l2_elevation_angle"] = {222, 10, false, false, 0.1, -51.2};
  l.signals["l2_azimuth_angle"] = {232, 11, false, false, 0.1, -102.4};
  l.signals["l2_radial_distance_velocity_covariance"] = {243, 11, false, false, 0.0001, -0.1024};
  l.signals["l2_radial_distance_variance"] = {254, 10, false, false, 5e-005, 0};
  l.signals["l2_radial_velocity_variance"] = {264, 10, false, false, 0.0001, 0};
  l.signals["l2_elevation_angle_variance"] = {274, 10, false, false, 0.001, 0};
  l.signals["l2_azimuth_angle_variance"] = {284, 10, false, false, 0.001, 0};
  l.signals["l2_rcs"] = {294, 10, false, false, 0.2, -102.4};
  l.signals["l2_radial_distance_velocity_quality"] = {304, 8, false, false, 1, 0};
  l.signals["l2_elevation_angle_quality"] = {312, 8, false, false, 1, 0};
  l.signals["l2_azimuth_angle_quality"] = {320, 8, false, false, 1, 0};
  l.signals["l2_rssi"] = {328, 10, false, false, 0.1, 0};
  l.signals["l2_azimuthal_partner_id"] = {338, 10, false, false, 1, 0};
  l.signals["l2_measurement_status"] = {348, 4, false, false, 1, 0};

  l.signals["l3_radial_distance"] = {352, 15, false, false, 0.01, 0};
  l.signals["l3_radial_velocity"] = {367, 15, false, false, 0.01, -163.84};
  l.signals["l3_elevation_angle"] = {382, 10, false, false, 0.1, -51.2};
  l.signals["l3_azimuth_angle"] = {392, 11, false, false, 0.1, -102.4};
  l.signals["l3_radial_distance_velocity_covariance"] = {403, 11, false, false, 0.0001, -0.1024};
  l.signals["l3_radial_distance_variance"] = {414, 10, false, false, 5e-005, 0};
  l.signals["l3_radial_velocity_variance"] = {424, 10, false, false, 0.0001, 0};
  l.signals["l3_elevation_angle_variance"] = {434, 10, false, false, 0.001, 0};
  l.signals["l3_azimuth_angle_variance"] = {444, 10, false, false, 0.001, 0};
  l.signals["l3_rcs"] = {454, 10, false, false, 0.2, -102.4};
  l.signals["l3_radial_distance_velocity_quality"] = {464, 8, false, false, 1, 0};
  l.signals["l3_elevation_angle_quality"] = {472, 8, false, false, 1, 0};
  l.signals["l3_azimuth_angle_quality"] = {480, 8, false, false, 1, 0};
  l.signals["l3_rssi"] = {488, 10, false, false, 0.1, 0};
  l.signals["l3_azimuthal_partner_id"] = {498, 10, false, false, 1, 0};
  l.signals["l3_measurement_status"] = {508, 4, false, false, 1, 0};

  // Fill message definitions
  for (const auto & sensor_pair : sensors_) {
    const SensorInfo & info = sensor_pair.second;

    if (!info.active) {continue;}

    uint32_t base_location_id = info.can_fd_source_address;
    uint16_t num_locations = did_to_loc_number_.at(info.max_number_locations);

    total_number_of_locations_[info.can_fd_source_address & 0x0F] = num_locations;
    for (uint16_t i = 0; i < num_locations; ++i) {
      uint32_t id = base_location_id + i * 256;
      m[id] = l;

      // Replace "XXX" in the message name with the location index
      auto & name = m[id].name;
      name = std::regex_replace(name, std::regex("XXX"), std::to_string(i));
    }
  }

  return m;
}

void Receiver::process(std_msgs::msg::Header header, const FrameId & id, Message & message)
{
  using off_highway_can::auto_static_cast;

  auto id_suffix = static_cast<uint16_t>(id & 0xFF);
  header.frame_id = id_to_sensor_.at(id_suffix);
  location_base_id_ = sensors_.at(header.frame_id).can_fd_source_address;

  int32_t location_frame_id = (id - location_base_id_) / 256;

  if (location_frame_id >= 0 &&
    location_frame_id < did_to_loc_number_.at(sensors_.at(header.frame_id).max_number_locations))
  {
    Location l;

    uint16_t index =
      [](const std::array<uint16_t, 4> & number_of_locations, uint8_t counter) -> uint16_t {
        return std::accumulate(
          number_of_locations.begin(),
          number_of_locations.begin() + counter, uint16_t{0});
      }(total_number_of_locations_, id_suffix & 0x0F);

    uint16_t location_id_in_locations = location_frame_id + index;
    l.id = location_id_in_locations;
    l.header.stamp = now();
    l.header.frame_id = header.frame_id;

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
      l.location1.measurement_status, message.signals["l1_measurement_status"].value);

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
    auto_static_cast(
      l.location2.measurement_status, message.signals["l2_measurement_status"].value);

    auto_static_cast(
      l.location3.radial_distance, message.signals["l3_radial_distance"].value);
    auto_static_cast(
      l.location3.radial_velocity, message.signals["l3_radial_velocity"].value);
    auto_static_cast(
      l.location3.azimuth_angle,
      message.signals["l3_azimuth_angle"].value * kDegToRad);
    auto_static_cast(
      l.location3.elevation_angle, message.signals["l3_elevation_angle"].value * kDegToRad);
    auto_static_cast(
      l.location3.radial_distance_variance,
      message.signals["l3_radial_distance_variance"].value);
    auto_static_cast(
      l.location3.radial_velocity_variance,
      message.signals["l3_radial_velocity_variance"].value);
    auto_static_cast(
      l.location3.azimuth_angle_variance,
      message.signals["l3_azimuth_angle_variance"].value * kDegToRad * kDegToRad);
    auto_static_cast(
      l.location3.elevation_angle_variance,
      message.signals["l3_elevation_angle_variance"].value * kDegToRad * kDegToRad);
    auto_static_cast(
      l.location3.radial_distance_velocity_covariance,
      message.signals["l3_radial_distance_velocity_covariance"].value);
    auto_static_cast(l.location3.rcs, message.signals["l3_rcs"].value);
    auto_static_cast(l.location3.rssi, message.signals["l3_rssi"].value);
    auto_static_cast(
      l.location3.radial_distance_velocity_quality,
      message.signals["l3_radial_distance_velocity_quality"].value);
    auto_static_cast(
      l.location3.azimuth_angle_quality, message.signals["l3_azimuth_angle_quality"].value);
    auto_static_cast(
      l.location3.elevation_angle_quality,
      message.signals["l3_elevation_angle_quality"].value);
    auto_static_cast(
      l.location3.azimuthal_partner_id, message.signals["l3_azimuthal_partner_id"].value);
    auto_static_cast(
      l.location3.measurement_status, message.signals["l3_measurement_status"].value);
    locations_[l.id] = l;
  }
}

bool Receiver::filter(const Location & location)
{
  if (location.location1.radial_distance == 0 && location.location2.radial_distance == 0 &&
    location.location3.radial_distance == 0)
  {
    return true;
  }
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

void Receiver::process_location(
  PclPointLocation & location,
  std::string & frame_id,
  pcl::PointCloud<PclPointLocation> & locations_pcl,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
  try {
    geometry_msgs::msg::TransformStamped transform_stamped =
      tf_buffer->lookupTransform("base_link", frame_id, tf2::TimePointZero);

    // Extract translation
    double tx = transform_stamped.transform.translation.x;
    double ty = transform_stamped.transform.translation.y;
    double tz = transform_stamped.transform.translation.z;

    // Extract rotation (quaternion)
    tf2::Quaternion q(
      transform_stamped.transform.rotation.x,
      transform_stamped.transform.rotation.y,
      transform_stamped.transform.rotation.z,
      transform_stamped.transform.rotation.w);

    // Convert quaternion to roll, pitch, yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Apply the transform to the point
    tf2::Vector3 point(location.x, location.y, location.z);
    tf2::Vector3 transformed_point = tf2::Transform(q, tf2::Vector3(tx, ty, tz)) * point;

    // Update the location with the transformed coordinates
    PclPointLocation transformed_location(location);
    transformed_location.x = transformed_point.x();
    transformed_location.y = transformed_point.y();
    transformed_location.z = transformed_point.z();

    locations_pcl.emplace_back(transformed_location);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "Transform from base_link to %s is not available: %s",
      frame_id.c_str(), ex.what());
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

  geometry_msgs::msg::PointStamped point_stamped;
  for (auto & location : locations_) {
    if (location) {
      std::string frame_id = location->header.frame_id;
      if (location->location1.radial_distance != 0) {
        PclPointLocation loc1(location->location1);
        process_location(loc1, frame_id, locations_pcl, tf_buffer_);
      }
      if (location->location2.radial_distance != 0) {
        PclPointLocation loc2(location->location2);
        process_location(loc2, frame_id, locations_pcl, tf_buffer_);
      }
      if (location->location3.radial_distance != 0) {
        PclPointLocation loc3(location->location3);
        process_location(loc3, frame_id, locations_pcl, tf_buffer_);
      }
    }
  }

  sensor_msgs::msg::PointCloud2 pointcloud2;
  pcl::toROSMsg(locations_pcl, pointcloud2);
  pub_locations_pcl_->publish(pointcloud2);
}

void Receiver::declare_and_get_parameters()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;

  param_desc.description =
    "Allowed age corresponding to output cycle time of sensor plus safety margin";
  declare_parameter<double>("allowed_age", 0.1, param_desc);
  allowed_age_ = get_parameter("allowed_age").as_double();

  param_desc.description =
    "Frequency at which current location list (point cloud) is published. Corresponds to ~100 ms "
    "radar sending cycle time.";
  declare_parameter<double>("publish_frequency", 10.0, param_desc);
  publish_frequency_ = get_parameter("publish_frequency").as_double();

  declare_parameter<bool>("sensors.sensor1.active", false);
  sensors_["sensor1"].active = get_parameter("sensors.sensor1.active").as_bool();
  declare_parameter<int>("sensors.sensor1.max_number_locations", 1);
  sensors_["sensor1"].max_number_locations =
    get_parameter("sensors.sensor1.max_number_locations").as_int();
  declare_parameter<int>("sensors.sensor1.can_fd_source_address", 0x18FF04B0);
  sensors_["sensor1"].can_fd_source_address =
    get_parameter("sensors.sensor1.can_fd_source_address").as_int();
  id_to_sensor_[sensors_["sensor1"].can_fd_source_address & 0xFF] = "sensor1";

  declare_parameter<bool>("sensors.sensor2.active", false);
  sensors_["sensor2"].active = get_parameter("sensors.sensor2.active").as_bool();
  declare_parameter<int>("sensors.sensor2.max_number_locations", 1);
  sensors_["sensor2"].max_number_locations =
    get_parameter("sensors.sensor2.max_number_locations").as_int();
  declare_parameter<int>("sensors.sensor2.can_fd_source_address", 0x18FF04B1);
  sensors_["sensor2"].can_fd_source_address =
    get_parameter("sensors.sensor2.can_fd_source_address").as_int();
  id_to_sensor_[sensors_["sensor2"].can_fd_source_address & 0xFF] = "sensor2";

  declare_parameter<bool>("sensors.sensor3.active", false);
  sensors_["sensor3"].active = get_parameter("sensors.sensor3.active").as_bool();
  declare_parameter<int>("sensors.sensor3.max_number_locations", 1);
  sensors_["sensor3"].max_number_locations =
    get_parameter("sensors.sensor3.max_number_locations").as_int();
  declare_parameter<int>("sensors.sensor3.can_fd_source_address", 0x18FF04B2);
  sensors_["sensor3"].can_fd_source_address =
    get_parameter("sensors.sensor3.can_fd_source_address").as_int();
  id_to_sensor_[sensors_["sensor3"].can_fd_source_address & 0xFF] = "sensor3";

  declare_parameter<bool>("sensors.sensor4.active", false);
  sensors_["sensor4"].active = get_parameter("sensors.sensor4.active").as_bool();
  declare_parameter<int>("sensors.sensor4.max_number_locations", 1);
  sensors_["sensor4"].max_number_locations =
    get_parameter("sensors.sensor4.max_number_locations").as_int();
  declare_parameter<int>("sensors.sensor4.can_fd_source_address", 0x18FF04B3);
  sensors_["sensor4"].can_fd_source_address =
    get_parameter("sensors.sensor4.can_fd_source_address").as_int();
  id_to_sensor_[sensors_["sensor4"].can_fd_source_address & 0xFF] = "sensor4";
}

}  // namespace corner_radar_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(corner_radar_driver::Receiver)
