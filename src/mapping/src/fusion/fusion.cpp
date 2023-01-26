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
* desc: fusion class for cone matching between camera, lidar and map
*/

#include <fusion/fusion.hpp>

namespace utfr_dv {
namespace mapping{

utfr_msgs::msg::ConeMap Fusion::extractMatchedCones(
  const std::vector<utfr_msgs::msg::Cone> & lidar_cones, 
  const utfr_msgs::msg::ConeDetections& camera_cones, 
  utfr_msgs::msg::ConeMap& cone_map, 
  double range_threshold,
  double map_threshold){

  utfr_msgs::msg::ConeDetections matchedCones;

  // Go through left, right, and other cones in the camera_cones variable
  for (const auto& camera_cones : camera_cones.left_cones)
  {
    // Find pairs in the lidar_cones variable within the range_threshold
    for (const auto& lidar_cones : lidar_cones)
    {
      double distance = 
          util::euclidianDistance2D(camera_cones.pos, lidar_cones.pos);

      if (distance <= range_threshold)
      {
        // If they are in range, combine them by weighting lidar point 60% and 
        // camera point 40%
        utfr_msgs::msg::Cone combinedCone;
        combinedCone.pos.x = 
            (lidar_cones.pos.x * 0.6) + (camera_cones.pos.x * 0.4);
        combinedCone.pos.y = 
            (lidar_cones.pos.y * 0.6) + (camera_cones.pos.y * 0.4);
        combinedCone.type = camera_cones.type;
        matchedCones.left_cones.push_back(combinedCone);
        break;
      }
    }
  }

  for (const auto& camera_cones : camera_cones.right_cones)
  {
    for (const auto& lidar_cones : lidar_cones)
    {
      double distance = 
          utfr_dv::util::euclidianDistance2D(camera_cones.pos, lidar_cones.pos);
      if (distance <= range_threshold)
      {
        utfr_msgs::msg::Cone combinedCone;
        combinedCone.pos.x = 
            (lidar_cones.pos.x * 0.6) + (camera_cones.pos.x * 0.4);
        combinedCone.pos.y = 
            (lidar_cones.pos.y * 0.6) + (camera_cones.pos.y * 0.4);
        combinedCone.type = camera_cones.type;
        matchedCones.right_cones.push_back(combinedCone);
        break;
      }
    }
  }
  for (const auto& camera_cones : camera_cones.other_cones){
    for (const auto& lidar_cones : lidar_cones){
      double distance = 
          utfr_dv::util::euclidianDistance2D(camera_cones.pos, lidar_cones.pos);
      if (distance <= range_threshold){
        utfr_msgs::msg::Cone combinedCone;
        combinedCone.pos.x = 
            (lidar_cones.pos.x * 0.6) + (camera_cones.pos.x * 0.4);
        combinedCone.pos.y = 
            (lidar_cones.pos.y * 0.6) + (camera_cones.pos.y * 0.4);
        combinedCone.type = camera_cones.type;
        matchedCones.other_cones.push_back(combinedCone);
        break;
      }
    }
  }

  // if cones are within the map_threshold, 
  // update the cone_map existing cone on cone_map with average distance. 
  // Otherwise, add the cone to the cone_map
  for (const auto& matchedCone : matchedCones.left_cones)
  {
    bool coneFound = false;
    for (auto& mapCone : cone_map.left_cones)
    {
      double distance = 
          utfr_dv::util::euclidianDistance2D(matchedCone.pos, mapCone.pos);
      if (distance <= map_threshold)
      {
        // If there is a pair within map_threshold, change the location of 
        // existing cone on cone_map by weighting new location 30% and old 
        // location 70%
        mapCone.pos.x= (matchedCone.pos.x * 0.3) + (mapCone.pos.x * 0.7);
        mapCone.pos.y = (matchedCone.pos.y * 0.3) + (mapCone.pos.y * 0.7);
        coneFound = true;
        break;
      }
    }
    if (!coneFound)
    {
      // If the cone is not found in the cone_map, append it to the map
      cone_map.left_cones.push_back(matchedCone);
    }
  }
  //checking right cones 
  for (const auto& matchedCone : matchedCones.right_cones)
  {
    bool coneFound = false;
    for (auto& mapCone : cone_map.right_cones)
    {
      double distance = 
          utfr_dv::util::euclidianDistance2D(matchedCone.pos, mapCone.pos);
      if (distance <= map_threshold)
      {
        mapCone.pos.x= (matchedCone.pos.x * 0.3) + (mapCone.pos.x * 0.7);
        mapCone.pos.y = (matchedCone.pos.y * 0.3) + (mapCone.pos.y * 0.7);
        coneFound = true;
        break;
      }
    }
    if (!coneFound)
    {
      cone_map.right_cones.push_back(matchedCone);
    }
  }
  //checking other cones 
  for (const auto& matchedCone : matchedCones.other_cones)
  {
    bool coneFound = false;
    for (auto& mapCone : cone_map.other_cones)
    {
      double distance = 
          utfr_dv::util::euclidianDistance2D(matchedCone.pos, mapCone.pos);
      if (distance <= map_threshold)
      {
        mapCone.pos.x= (matchedCone.pos.x * 0.3) + (mapCone.pos.x * 0.7);
        mapCone.pos.y = (matchedCone.pos.y * 0.3) + (mapCone.pos.y * 0.7);
        coneFound = true;
        break;
      }
    }
    if (!coneFound)
    {
      cone_map.other_cones.push_back(matchedCone);
    }
  } 
    return cone_map;
}

} //namespace mapping
} //namespace utfr_dv