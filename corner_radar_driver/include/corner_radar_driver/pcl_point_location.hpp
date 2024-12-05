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

#include "pcl/pcl_macros.h"
#include "pcl/point_types.h"
#include "pcl/register_point_struct.h"

#include "corner_radar_driver_msgs/msg/location_values.hpp"

namespace corner_radar_driver
{

/**
 * \brief Location as PCL point
 */
struct EIGEN_ALIGN16 PclPointLocation
{
  PCL_ADD_POINT4D;
  float radial_velocity{0.};
  float radial_distance_variance{0.};
  float radial_velocity_variance{0.};
  float rcs{0.};
  float rssi{0.};
  float azimuth_angle_variance{0.};
  float elevation_angle_variance{0.};
  float radial_distance_velocity_covariance{0.};
  float radial_distance_velocity_quality{0.};
  float azimuth_angle_quality{0.};
  float elevation_angle_quality{0.};
  float azimuthal_partner_id{0.};

  /**
   * \brief Construct a new PclPointLocation object from a location
   *
   * \param l Location input
   */
  explicit PclPointLocation(const corner_radar_driver_msgs::msg::LocationValues & l)
  : radial_velocity(static_cast<float>(l.radial_velocity)),
    radial_distance_variance(static_cast<float>(l.radial_distance_variance)),
    radial_velocity_variance(static_cast<float>(l.radial_velocity_variance)),
    rcs(static_cast<float>(l.rcs)),
    rssi(static_cast<float>(l.rssi)),
    azimuth_angle_variance(static_cast<float>(l.azimuth_angle_variance)),
    elevation_angle_variance(static_cast<float>(l.elevation_angle_variance)),
    radial_distance_velocity_covariance(static_cast<float>(l.radial_distance_velocity_covariance)),
    radial_distance_velocity_quality(static_cast<float>(l.radial_distance_velocity_quality)),
    azimuth_angle_quality(static_cast<float>(l.azimuth_angle_quality)),
    elevation_angle_quality(static_cast<float>(l.elevation_angle_quality)),
    azimuthal_partner_id(static_cast<float>(l.azimuthal_partner_id))
  {
    const float & phi = l.elevation_angle;
    const float & theta = l.azimuth_angle;
    float cos_phi = cos(phi);
    float sin_theta = sin(theta);

    // Assumption that the angles are in a valid range of cone coordinates
    x = l.radial_distance * sqrt(cos_phi * cos_phi - sin_theta * sin_theta);
    y = l.radial_distance * sin_theta;
    z = l.radial_distance * sin(phi);
  }
  PclPointLocation() {}
};  // SSE padding

}  // namespace corner_radar_driver

/* *INDENT-OFF* */
POINT_CLOUD_REGISTER_POINT_STRUCT(corner_radar_driver::PclPointLocation,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, radial_velocity, radial_velocity)
                                  (float, radial_distance_variance, radial_distance_variance)
                                  (float, radial_velocity_variance, radial_velocity_variance)
                                  (float, azimuth_angle_variance, azimuth_angle_variance)
                                  (float, rcs, rcs)
                                  (float, rssi, rssi)
                                  (float, elevation_angle_variance, elevation_angle_variance)
                                  (float, radial_distance_velocity_covariance, radial_distance_velocity_covariance)  // NOLINT
                                  (float, radial_distance_velocity_quality, radial_distance_velocity_quality)  // NOLINT
                                  (float, azimuth_angle_quality, azimuth_angle_quality)
                                  (float, elevation_angle_quality, elevation_angle_quality)
                                  (float, azimuthal_partner_id, azimuthal_partner_id))
/* *INDENT-ON* */
