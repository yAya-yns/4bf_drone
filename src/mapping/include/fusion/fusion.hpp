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
* file: fusion.cpp
* auth: Kyrylo Kalashnikov
* auth: Kelvin Cui
* desc: fusion class header for cone matching between camera, lidar and map
*/

// Message Requirements
#include <utfr_msgs/msg/cone_detections.hpp>
#include <utfr_msgs/msg/cone_map.hpp>
#include <utfr_msgs/msg/cone.hpp>

// UTFR Common Requirements
#include <utfr_common/math.hpp>


namespace utfr_dv {
namespace mapping{
  
class Fusion
{
public:
  /*! Extract cones from camera and lidar and return a cone map
   *
   *  @param[in] lidar_cones all the cones detected by the lidar, world frame
   *  @param[in] camera_cones all the cones detected by the camera, world frame
   *  @param[in] cone_map the latest world frame cone map
   *  @param[in] range_threshold the range threshold for matching camera_cones 
   *                             and lidar_cones to create matched_cones
   *  @param[in] map_threshold the map threshold for matching matched_cones and 
   *                           mapCones
   * 
   *  @returns ConeMap of updated world frame cone map
   */
  utfr_msgs::msg::ConeMap extractMatchedCones(
      const std::vector<utfr_msgs::msg::Cone> & lidar_cones, 
      const utfr_msgs::msg::ConeDetections& camera_cones, 
      utfr_msgs::msg::ConeMap& cone_map, 
      double range_threshold, 
      double map_threshold); 

private:

};

using FusionUPtr = std::unique_ptr<Fusion>;
using FusionSPtr = std::shared_ptr<Fusion>;

} //namespace mapping
} //namespace utfr_dv