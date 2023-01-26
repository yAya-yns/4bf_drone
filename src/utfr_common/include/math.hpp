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
* file: math.hpp
* auth: Kelvin Cui
* auth: Trevor Foote
* desc: math common header
*/
#pragma once

//System Requirements
#include <cmath>

//Message Requirements
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <utfr_msgs/msg/cone_map.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace utfr_dv {
namespace util {

#define EPS 1e-8

/*! Calculate if two input doubles are equal to higher precisions.
 *  Compares different between both doubles to the defined EPS value.
 *  @param[in] a, b doubles to be compared.
 *  @returns bool true if doubles are within EPS range
 */
bool doubleEQ(const double a, const double b);

/*! Convert Degrees to Radians
 *  
 *  @param[in] rad double of angle in radians
 *  @returns double of angle in degrees
 */
double degToRad(const double rad);

/*! Convert Radians to Degrees
 *  
 *  @param[in] deg double of angle in degrees
 *  @returns double of angle in radians
 */
double radToDeg(const double deg);

/*! Calculate equivalent radian value within 0 to 2pi period
 *  @param[in] rad double of angle in radians
 *  @returns double of angle in radians within 0 to 2pi period
 */
double wrapRad(const double rad);

/*! Calculate equivalent degree value within 0 to 360 period
 *  @param[in] deg double of angle in degrees
 *  @returns double of angle in degree within 0 to 360 period
 */
double wrapDeg(const double deg);

/*! Caluclate Euclidian Distance in 2D of two points.
 *  Wrapper function for eucldianDistance for geometry_msgs::Points.
 *
 * @param[in] a geometry_msgs::Point 1 of XY coordinate
 * @param[in] b geometry_msgs::Point 2 of XY coordiante
 * @returns double of eucldiian distance between a and b.
 */ 
double euclidianDistance2D(const geometry_msgs::msg::Point& a,
                           const geometry_msgs::msg::Point& b);

/*! Caluclate Euclidian Distance in 2D of two points.
 *  Wrapper function for eucldianDistance for geometry_msgs::Vector3.
 *
 * @param[in] a geometry_msgs::Point 1 of XY coordinate
 * @param[in] b geometry_msgs::Point 2 of XY coordiante
 * @returns double of eucldiian distance between a and b.
 */ 
double euclidianDistance2D(const geometry_msgs::msg::Vector3& a,
                           const geometry_msgs::msg::Vector3& b);

/*! Calculate Euclidian Distance between two points
 *  
 *  @param[in] x1 double of x coordinate of first point
 *  @param[in] x2 double of x coordinate of second point
 *  @param[in] y1 double of y coordinate of first point
 *  @param[in] y2 double of y coordinate of second point
 *  @returns double of Euclidian Distance
 */
double euclidianDistance2D(const double x1, const double x2,
    const double y1, const double y2);

/*! Calculate factornial of n
 *  @param[in] n unsigned integer
 *  @returns factorial of n
 */
int factorial (const unsigned int n);

/*! Convert Lidar Scan to a vector of Cone objects
 *  @param[in] scan sensor_msgs::msg::LaserScan of lidar scan in radians
 *  @param[in] range_threshold double of range threshold to filter out points
 *  @returns std::vector<utfr_msgs::msg::Cone> of cone objects
*/
std::vector<utfr_msgs::msg::Cone> convertLidarToCoordinates(const sensor_msgs::msg::LaserScan& scan, 
                                                            const double range_threshold);

/*! Convert Latitude, Longitude, Altitude to North, East, Down co-ordinates, based off a datum point
 *  @param[in] target geometry_msgs::msg::Vector3&, containing the target point. Order of Lat. (Decimal Degrees), Long. (Decimal Degrees), Alt. (M)
 *  @param[in] world_datum geometry_msgs::msg::Vector3&, containing the datum point. Order of Lat. (Decimal Degrees), Long. (Decimal Degrees), Alt. (M)
 *  @returns geometry_msgs::msg::Vector3, containing the distance in meters from the datum point. Order of North (M), East (M), Down (M)
*/
geometry_msgs::msg::Vector3 convertLLAtoNED(const geometry_msgs::msg::Vector3& target, 
                                const geometry_msgs::msg::Vector3& world_datum);


} // namespace util
} // namespace utfr_dv
