# Radial distance [m]
float32 radial_distance
# Radial distance variance [m^2]
float32 radial_distance_variance

# Relative radial velocity [m/s]
float32 radial_velocity
# Variance of radial velocity [m^2/s^2]
float32 radial_velocity_variance

# Covariance of radial distance and velocity [m^2/s]
float32 radial_distance_velocity_covariance
# Quality of velocity resolution processing in [0,1]
float32 radial_distance_velocity_quality

# Elevation angle (phi) [rad]
float32 elevation_angle
# Quality of elevation angle in [0,1]
float32 elevation_angle_quality
# Variance of elevation angles [rad^2]
float32 elevation_angle_variance

# Azimuth angle (theta) [rad]
float32 azimuth_angle
# Quality of azimuth angle in [0,1]
float32 azimuth_angle_quality
# Variance of azimuth angles [rad^2]
float32 azimuth_angle_variance
# ID of the azimuthal partner location
float32 azimuthal_partner_id

# Radar cross section [dBm2]
float32 rcs
# Received Signal Strength Indication (RSSI) [no unit]
float32 rssi
