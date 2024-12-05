# corner_radar_driver_msgs

This package provides the ROS messages for the corner_radar_driver package.

## Messages

* [Location](msg/Location.msg): Composed radar message containing location1 and location2
* [LocationArray](msg/LocationArray.msg): List of radar location message
* [LocationValues](msg/LocationValues.msg): Composed message location containing: radial distance,
  radial distance variance, radial velocity, radial velocity variance, radial distance velocity
  covariance, radial distance velocity quality, elevation angle, elevation angle quality, elevation
  angle variance, azimuth angle, azimuth angle quality, azimuth angle variance, azimuthal partner
  id, rcs and rssi
