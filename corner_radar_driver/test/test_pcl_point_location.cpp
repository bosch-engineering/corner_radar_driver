// Copyright 2024 Robert Bosch GmbH and its subsidiaries
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

#include <numbers>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl_ros/impl/transforms.hpp"
#include "rclcpp/rclcpp.hpp"

#include "corner_radar_driver/pcl_point_location.hpp"

static constexpr double kDegToRad = std::numbers::pi / 180.0;

TEST(TestCornerRadarPclPointLocation, testPclTransform)
{
  geometry_msgs::msg::TransformStamped transform;
  corner_radar_driver_msgs::msg::LocationValues location;
  pcl::PointCloud<corner_radar_driver::PclPointLocation> cloud_in, cloud_out;
  cloud_in.emplace_back(location);
  EXPECT_NO_THROW(pcl_ros::transformPointCloud(cloud_in, cloud_out, transform));
}

TEST(TestCornerRadarPclPointLocation, testPclTransformAnyValues)
{
  corner_radar_driver_msgs::msg::LocationValues polar;

  polar.radial_distance = 511.0;
  polar.radial_distance_variance = 0.999984741210938;
  polar.radial_velocity = 127.99609375;
  polar.radial_velocity_variance = 0.999984741210938;
  polar.radial_distance_velocity_covariance = 0.124996185302734;
  polar.radial_distance_velocity_quality = 255.0;
  polar.elevation_angle = 30.0 * kDegToRad;
  polar.elevation_angle_quality = 255.0;
  polar.elevation_angle_variance = 255.99609375;
  polar.azimuth_angle = 40.0 * kDegToRad;
  polar.azimuth_angle_quality = 255.0;
  polar.azimuth_angle_variance = 255.99609375;
  polar.azimuthal_partner_id = 255.0;
  polar.rcs = 127.99609375;
  polar.rssi = 127.998046875;

  const float & phi = polar.elevation_angle;
  const float & theta = polar.azimuth_angle;
  float cos_phi = cos(phi);
  float sin_theta = sin(theta);

  corner_radar_driver::PclPointLocation test_point(polar);

  EXPECT_EQ(test_point.x, polar.radial_distance * sqrt(cos_phi * cos_phi - sin_theta * sin_theta));
  EXPECT_EQ(test_point.y, polar.radial_distance * sin_theta);
  EXPECT_EQ(test_point.z, polar.radial_distance * sin(phi));
}

TEST(TestCornerRadarPclPointLocation, testPclTransformMaxValues)
{
  corner_radar_driver_msgs::msg::LocationValues polar;

  polar.radial_distance = 511.0;
  polar.radial_distance_variance = 0.999984741210938;
  polar.radial_velocity = 127.99609375;
  polar.radial_velocity_variance = 0.999984741210938;
  polar.radial_distance_velocity_covariance = 0.124996185302734;
  polar.radial_distance_velocity_quality = 255.0;
  polar.elevation_angle = 30.0 * kDegToRad;
  polar.elevation_angle_quality = 255.0;
  polar.elevation_angle_variance = 255.99609375;
  polar.azimuth_angle = -30.0 * kDegToRad;
  polar.azimuth_angle_quality = 255.0;
  polar.azimuth_angle_variance = 255.99609375;
  polar.azimuthal_partner_id = 255.0;
  polar.rcs = 127.99609375;
  polar.rssi = 127.998046875;

  const float & phi = polar.elevation_angle;
  const float & theta = polar.azimuth_angle;
  float cos_phi = cos(phi);
  float sin_theta = sin(theta);

  corner_radar_driver::PclPointLocation test_point(polar);

  EXPECT_EQ(test_point.x, polar.radial_distance * sqrt(cos_phi * cos_phi - sin_theta * sin_theta));
  EXPECT_EQ(test_point.y, polar.radial_distance * sin_theta);
  EXPECT_EQ(test_point.z, polar.radial_distance * sin(phi));
}

TEST(TestCornerRadarPclPointLocation, testPclTransformMinValues)
{
  corner_radar_driver_msgs::msg::LocationValues polar;

  polar.radial_distance = 511.0;
  polar.radial_distance_variance = 0.999984741210938;
  polar.radial_velocity = 127.99609375;
  polar.radial_velocity_variance = 0.999984741210938;
  polar.radial_distance_velocity_covariance = 0.124996185302734;
  polar.radial_distance_velocity_quality = 255.0;
  polar.elevation_angle = -30.0 * kDegToRad;
  polar.elevation_angle_quality = 255.0;
  polar.elevation_angle_variance = 255.99609375;
  polar.azimuth_angle = -59.9 * kDegToRad;
  polar.azimuth_angle_quality = 255.0;
  polar.azimuth_angle_variance = 255.99609375;
  polar.azimuthal_partner_id = 255.0;
  polar.rcs = 127.99609375;
  polar.rssi = 127.998046875;

  const float & phi = polar.elevation_angle;
  const float & theta = polar.azimuth_angle;
  float cos_phi = cos(phi);
  float sin_theta = sin(theta);

  corner_radar_driver::PclPointLocation test_point(polar);

  EXPECT_EQ(test_point.x, polar.radial_distance * sqrt(cos_phi * cos_phi - sin_theta * sin_theta));
  EXPECT_EQ(test_point.y, polar.radial_distance * sin_theta);
  EXPECT_EQ(test_point.z, polar.radial_distance * sin(phi));
}

TEST(TestCornerRadarPclPointLocation, testPclTransformOutOfRangeValues)
{
  corner_radar_driver_msgs::msg::LocationValues polar;

  polar.radial_distance = 511.0;
  polar.radial_distance_variance = 0.999984741210938;
  polar.radial_velocity = 127.99609375;
  polar.radial_velocity_variance = 0.999984741210938;
  polar.radial_distance_velocity_covariance = 0.124996185302734;
  polar.radial_distance_velocity_quality = 255.0;
  polar.elevation_angle = -31.0 * kDegToRad;
  polar.elevation_angle_quality = 255.0;
  polar.elevation_angle_variance = 255.99609375;
  polar.azimuth_angle = -61.0 * kDegToRad;
  polar.azimuth_angle_quality = 255.0;
  polar.azimuth_angle_variance = 255.99609375;
  polar.azimuthal_partner_id = 255.0;
  polar.rcs = 127.99609375;
  polar.rssi = 127.998046875;

  const float & phi = polar.elevation_angle;
  const float & theta = polar.azimuth_angle;
  float cos_phi = cos(phi);
  float sin_theta = sin(theta);

  corner_radar_driver::PclPointLocation test_point(polar);

  EXPECT_TRUE(isnan(polar.radial_distance * sqrt(cos_phi * cos_phi - sin_theta * sin_theta)));
  EXPECT_EQ(test_point.y, polar.radial_distance * sin_theta);
  EXPECT_EQ(test_point.z, polar.radial_distance * sin(phi));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
