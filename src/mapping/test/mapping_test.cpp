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
* file: mapping_test.cpp
* auth: Kelvin Cui
* desc: mapping functions unit tests
*/

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <fusion/fusion.hpp>

using namespace utfr_dv::mapping;

TEST(fusion, extractMatchedCones){
  FusionUPtr fusion = std::make_unique<Fusion>();
  utfr_msgs::msg::ConeMap current_map;
  std::vector<utfr_msgs::msg::Cone> lidar_cone_array;
  utfr_msgs::msg::ConeDetections camera_cone_detections;

  //Build Lidar Cone Array:
  utfr_msgs::msg::Cone lidar_cone;
  lidar_cone.type = utfr_msgs::msg::Cone::UNKNOWN;

  lidar_cone.pos.x = 1.0;
  lidar_cone.pos.y = 2.0,
  lidar_cone_array.push_back(lidar_cone);

  lidar_cone.pos.x = 3.0;
  lidar_cone.pos.y = 2.0,
  lidar_cone_array.push_back(lidar_cone);

  //Build Camera Cone Detectons:
  utfr_msgs::msg::Cone camera_cone;

  camera_cone.type = utfr_msgs::msg::Cone::BLUE;
  camera_cone.pos.x = 1.1;
  camera_cone.pos.y = 2.5;
  camera_cone_detections.left_cones.push_back(camera_cone);

  camera_cone.type = utfr_msgs::msg::Cone::YELLOW;
  camera_cone.pos.x = 3.3;
  camera_cone.pos.y = 1.9;
  camera_cone_detections.right_cones.push_back(camera_cone);

  //Update Map:
  double range_threshold = 2.0;
  double map_threshold = 2.0;
  current_map = fusion->extractMatchedCones(
      lidar_cone_array, 
      camera_cone_detections, 
      current_map, 
      range_threshold, 
      map_threshold);
  
  // Verify Sizes
  long unsigned int expected_left_cone_array_size = 1;
  long unsigned int expected_right_cone_array_size = 1;
  long unsigned int expected_other_cone_array_size = 0;
  ASSERT_EQ(current_map.left_cones.size(), expected_left_cone_array_size);
  ASSERT_EQ(current_map.right_cones.size(), expected_right_cone_array_size);
  ASSERT_EQ(current_map.other_cones.size(), expected_other_cone_array_size);

  // Verify New Cone Positions
  ASSERT_DOUBLE_EQ(current_map.left_cones[0].pos.x, 1.04);
  ASSERT_DOUBLE_EQ(current_map.left_cones[0].pos.y, 2.2);

  ASSERT_DOUBLE_EQ(current_map.right_cones[0].pos.x, 3.12);
  ASSERT_DOUBLE_EQ(current_map.right_cones[0].pos.y, 1.96);


  // Add orange camera-only cone:
  camera_cone.type = utfr_msgs::msg::Cone::SMALL_ORANGE;
  camera_cone.pos.x = 4.0;
  camera_cone.pos.y = 0.0;
  camera_cone_detections.other_cones.push_back(camera_cone);

  // Update Map:
  current_map = fusion->extractMatchedCones(
    lidar_cone_array, 
    camera_cone_detections, 
    current_map, 
    range_threshold, 
    map_threshold);

  // Verify Updated Sizes
  /* TODO - fix behaviour - camera only detections should still be added
  expected_other_cone_array_size = 1;
  ASSERT_EQ(current_map.left_cones.size(), expected_left_cone_array_size);
  ASSERT_EQ(current_map.right_cones.size(), expected_right_cone_array_size);
  ASSERT_EQ(current_map.other_cones.size(), expected_other_cone_array_size);

  // Verify New Cone positions and type
  ASSERT_DOUBLE_EQ(current_map.other_cones[0].pos.x, 4.0);
  ASSERT_DOUBLE_EQ(current_map.other_cones[0].pos.y, 0.0);
  ASSERT_EQ(current_map.other_cones[0].type, utfr_msgs::msg::Cone::SMALL_ORANGE);
  */
  
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}