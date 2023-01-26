/*
******************************************************
|                                                    |
|           _     __                    _            |
|    _   _ | |_  / _| _ __            _| |__   __    |
|   | | | || __|| |_ | '__|         / _` |\ \ / /    |
|   | |_| || |_ |  _|| |           | (_| | \ V /     |
|    \__,_| \__||_|  |_|    _____   \__,_|  \_/      |
|                          |_____|                   |
|                                                    |
|                                                    |
******************************************************
*
* file: math.cpp
* auth: Kelvin Cui
* auth: Trevor Foote
* desc: math common functions
*/

#include <math.hpp>
#include <utfr_msgs/msg/cone_map.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <iostream>

namespace utfr_dv {
namespace util {

bool doubleEQ(const double a, const double b){
  double abs_diff = fabs(a-b);
  return (abs_diff < EPS);
}

double radToDeg(const double rad){
  return rad*180/M_PI;
}

double degToRad(const double deg){
  return deg*M_PI/180;
}

double wrapRad(const double rad) {
  if (rad >= 0 && rad < 2*M_PI) {return rad;} 
  else if (rad < 0) {return wrapRad(rad + 2*M_PI);}
  else {return wrapRad(rad - 2*M_PI);}
}

double wrapDeg(const double deg) {
  if (deg >= 0 && deg < 360) {return deg;} 
  else if (deg < 0) {return wrapDeg(deg + 360);}
  else {return wrapDeg(deg - 360);}
}

double euclidianDistance2D(const geometry_msgs::msg::Point& a, 
                         const geometry_msgs::msg::Point& b){
  return euclidianDistance2D(a.x, b.x, a.y, b.y);
}

double euclidianDistance2D(const geometry_msgs::msg::Vector3& a, 
                         const geometry_msgs::msg::Vector3& b){
  return euclidianDistance2D(a.x, b.x, a.y, b.y);
}

double euclidianDistance2D(const double x1, const double x2,
    const double y1, const double y2) {
      return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}

int factorial (const unsigned int n) {
  if (n <= 1) {return 1;} else {return factorial(n - 1) * n;}
}


std::vector<utfr_msgs::msg::Cone> convertLidarToCoordinates(const sensor_msgs::msg::LaserScan& scan, const double range_threshold)
{
  // Initialize an empty vector to hold the cones
  std::vector<utfr_msgs::msg::Cone> cones;

  // Loop through all the ranges in the scan
  for (size_t i = 0; i < scan.ranges.size(); ++i)
  {
    // If the range is within a certain threshold, consider it a cone
    if (scan.ranges[i] > 0 && scan.ranges[i] < range_threshold)
    {
      // Convert the range and angle to x and y coordinates
      double x = scan.ranges[i] * std::cos(scan.angle_min + scan.angle_increment * i);
      double y = scan.ranges[i] * std::sin(scan.angle_min + scan.angle_increment * i);

      // Create a Cone object with the coordinates and an unknown type
      utfr_msgs::msg::Cone cone;
      cone.pos.x = x;
      cone.pos.y = y;
      cone.type = utfr_msgs::msg::Cone::UNKNOWN;

      // Add the cone to the vector
      cones.push_back(cone);
    }
  }

  return cones;
}

geometry_msgs::msg::Vector3 convertLLAtoNED(const geometry_msgs::msg::Vector3& target, const geometry_msgs::msg::Vector3& world_datum) {
  // Constants we need
  double semimajor_axis = 6378137.0;
  double semiminor_axis = 6356752.314245;

  double target_lat = degToRad(target.x);
  double target_lon = degToRad(target.y);
  double world_datum_lat = degToRad(world_datum.x);
  double world_datum_lon = degToRad(world_datum.y);

  // LLA -> NED conversion for both the target and the world datum
  double N = pow(semimajor_axis, 2)/sqrt((pow(semimajor_axis, 2) * pow(std::cos(target_lat), 2)) + (pow(semiminor_axis, 2) * pow(std::sin(target_lat), 2)));
  double x_lla = (N + target.z) * std::cos(target_lat) * std::cos(target_lon);
  double y_lla = (N + target.z) * std::cos(target_lat) * std::sin(target_lon);
  double z_lla = (N * pow((semiminor_axis/semimajor_axis), 2) + target.z) * std::sin(target_lat);

  N = pow(semimajor_axis, 2)/sqrt(pow(semimajor_axis, 2) * pow(std::cos(world_datum_lat), 2) + pow(semiminor_axis, 2) * pow(std::sin(world_datum_lat), 2));
  double x_lla0 = (N + world_datum.z) * std::cos(world_datum_lat) * std::cos(world_datum_lon);
  double y_lla0 = (N + world_datum.z) * std::cos(world_datum_lat) * std::sin(world_datum_lon);
  double z_lla0 = (N * pow((semiminor_axis/semimajor_axis), 2) + world_datum.z) * std::sin(world_datum_lat);

  // Find the distances between the two points
  double u = x_lla - x_lla0;
  double v = y_lla - y_lla0;
  double w = z_lla - z_lla0;

  double t = std::cos(world_datum_lon) * u + std::sin(world_datum_lon) * v;

  double east = -std::sin(world_datum_lon) * u + std::cos((world_datum_lon)) * v;
  double up = std::cos(world_datum_lat) * t + std::sin(world_datum_lat) * w;
  double north = -std::sin(world_datum_lat) * t + std::cos(world_datum_lat) * w;

  // Declare a Vector3 message to hold the NED coordinates
  geometry_msgs::msg::Vector3 ned;
  ned.x = north;
  ned.y = east;
  ned.z = -up;

  return ned;

}

} //namespace util
} //namespace utfr_dv
