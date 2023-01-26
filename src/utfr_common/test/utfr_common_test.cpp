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
* desc: math common functions unit tests
*/

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <math.hpp>

using namespace utfr_dv::util;

TEST(math, doubleEQ){
  double a = 1.00;
  double b = 1.0;
  ASSERT_TRUE(doubleEQ(a,b));

  a = 2.0;
  b = 1.0;
  ASSERT_FALSE(doubleEQ(a,b));

  a = 1.000000023;
  b = 1.000000024;
  ASSERT_TRUE(doubleEQ(a,b));

  a = 1.00000023;
  b = 1.00000024;
  ASSERT_FALSE(doubleEQ(a,b));

}

TEST(Math, radToDeg){
  double rad = 0;
  ASSERT_DOUBLE_EQ(radToDeg(rad), 0.0);
  rad = M_PI;
  ASSERT_DOUBLE_EQ(radToDeg(rad), 180.0);
  rad = 2*M_PI;
  ASSERT_DOUBLE_EQ(radToDeg(rad), 360.0);
  rad = M_PI/2;
  ASSERT_DOUBLE_EQ(radToDeg(rad), 90.0);
  rad = M_PI/4;
  ASSERT_DOUBLE_EQ(radToDeg(rad), 45.0);
  rad = -M_PI;
  ASSERT_DOUBLE_EQ(radToDeg(rad), -180.0);
  rad = -2*M_PI;
  ASSERT_DOUBLE_EQ(radToDeg(rad), -360.0);
  rad = -M_PI/2;
  ASSERT_DOUBLE_EQ(radToDeg(rad), -90.0);
  rad = -M_PI/4;
  ASSERT_DOUBLE_EQ(radToDeg(rad), -45.0);
  rad = 4*M_PI/3;
  ASSERT_DOUBLE_EQ(radToDeg(rad), 240.0);
}

TEST(Math, degToRad){
  double deg = 0;
  ASSERT_DOUBLE_EQ(degToRad(deg), 0.0);
  deg = 120;
  ASSERT_DOUBLE_EQ(degToRad(deg), 2*M_PI/3);
  deg = 180;
  ASSERT_DOUBLE_EQ(degToRad(deg), M_PI);
  deg = 360;
  ASSERT_DOUBLE_EQ(degToRad(deg), 2*M_PI);
  deg = 510;
  ASSERT_DOUBLE_EQ(degToRad(deg), 17*M_PI/6);

  deg = -20;
  ASSERT_DOUBLE_EQ(degToRad(deg), -M_PI/9);
  deg = -45;
  ASSERT_DOUBLE_EQ(degToRad(deg), -M_PI/4);
  deg = -90;
  ASSERT_DOUBLE_EQ(degToRad(deg), -M_PI/2);
}

TEST(Math, convertLidarToCoordinates){
  
  sensor_msgs::msg::LaserScan lidar_scan;
  lidar_scan.angle_min = 0.0;
  lidar_scan.angle_max = 2.0 * M_PI;  
  
  //Build Lidar message:
  int lidar_points = 4;
  double angle_increment = (2* M_PI)/lidar_points;  

  lidar_scan.angle_increment = angle_increment;
  lidar_scan.ranges = {1.0, 2.0, 3.0, 4.0};

  double threshold = 3.5;
  std::vector<utfr_msgs::msg::Cone> result = 
      convertLidarToCoordinates(lidar_scan, threshold);

  const long unsigned int expected_size = 3;
  ASSERT_EQ(result.size(), expected_size);

  ASSERT_NEAR(result[0].pos.x, 1.0, 1e-6);
  ASSERT_NEAR(result[0].pos.y, 0.0, 1e-6);

  ASSERT_NEAR(result[1].pos.x, 0.0, 1e-6);
  ASSERT_NEAR(result[1].pos.y, 2.0, 1e-6);

  ASSERT_NEAR(result[2].pos.x, -3.0, 1e-6);
  ASSERT_NEAR(result[2].pos.y, 0.0, 1e-6);

  // Case 1: Angles 0 to 360 
  lidar_scan.angle_min = 0.0;
  lidar_scan.angle_max = 2 * M_PI;

  int lidar_points1 = 10;
  double angle_increment1 = 2*M_PI/lidar_points1;

  lidar_scan.angle_increment = angle_increment1;
  lidar_scan.ranges = {1.31276231, 3.89812345, 0.29384676, 3.11267283, 2.64238054, 2.73485001, 2.135912547, 3.52482117, 1.12634345, 0.29186443};

  double threshold_new = 4;
  std::vector<utfr_msgs::msg::Cone> result1 =
      convertLidarToCoordinates(lidar_scan, threshold_new);

  const long unsigned int expected_size1 = 10;
  ASSERT_EQ(result1.size(), expected_size1);

  // cone 0
  ASSERT_NEAR(result1[0].pos.x, 1.31276226, 1e-6);
  ASSERT_NEAR(result1[0].pos.y, 0.0, 1e-6);
  // cone 1
  ASSERT_NEAR(result1[1].pos.x, 3.15364812, 1e-6);
  ASSERT_NEAR(result1[1].pos.y, 2.29125947, 1e-6);
  // cone 2
  ASSERT_NEAR(result1[2].pos.x, 0.090803643, 1e-6);
  ASSERT_NEAR(result1[2].pos.y, 0.279464876, 1e-6);
  // cone 3
  ASSERT_NEAR(result1[3].pos.x, -0.961868802, 1e-6);
  ASSERT_NEAR(result1[3].pos.y, 2.96032778, 1e-6);
  // cone 4
  ASSERT_NEAR(result1[4].pos.x, -2.13773076, 1e-6);
  ASSERT_NEAR(result1[4].pos.y, 1.55315231, 1e-6);
  // cone 5
  ASSERT_NEAR(result1[5].pos.x, -2.73485001, 1e-6);
  ASSERT_NEAR(result1[5].pos.y, 0.0, 1e-6);
  // cone 6
  ASSERT_NEAR(result1[6].pos.x, -1.72798957, 1e-6);
  ASSERT_NEAR(result1[6].pos.y, -1.25545791, 1e-6);
  // cone 7
  ASSERT_NEAR(result1[7].pos.x, -1.08922964, 1e-6);
  ASSERT_NEAR(result1[7].pos.y, -3.35230414, 1e-6);
  // cone 8
  ASSERT_NEAR(result1[8].pos.x, 0.348059268, 1e-6);
  ASSERT_NEAR(result1[8].pos.y, -1.07121628, 1e-6);
  // cone 9
  ASSERT_NEAR(result1[9].pos.x, 0.236123284, 1e-6);
  ASSERT_NEAR(result1[9].pos.y, -0.171553608, 1e-6);

  // Case 2: Angles -90 to 90
  lidar_scan.angle_min = -M_PI/2;
  lidar_scan.angle_max = M_PI/2;

  int lidar_points2 = 10;
  double angle_increment2 = M_PI/lidar_points2;

  lidar_scan.angle_increment = angle_increment2;
  lidar_scan.ranges={0.924365835, 3.413582573, 2.51439135, 1.439572457, 1.98431421, 3.49832869, 3.41578217, 0.98761403, 2.22435655, 0.43793127, 1.09137246};
  std::vector<utfr_msgs::msg::Cone> result2 =
      convertLidarToCoordinates(lidar_scan, threshold_new);

  const long unsigned int expected_size2 = 11;
  ASSERT_EQ(result2.size(), expected_size2);
  
  // cone 0
  ASSERT_NEAR(result2[0].pos.x, 0.0, 1e-6);
  ASSERT_NEAR(result2[0].pos.y, -0.924365835, 1e-6); 
  // cone 1
  ASSERT_NEAR(result2[1].pos.x, 1.05485503, 1e-6);
  ASSERT_NEAR(result2[1].pos.y, -3.24650995, 1e-6); 
  // cone 2
  ASSERT_NEAR(result2[2].pos.x, 1.47792215, 1e-6);
  ASSERT_NEAR(result2[2].pos.y, -2.03418533, 1e-6); 
  // cone 3
  ASSERT_NEAR(result2[3].pos.x, 1.16463858, 1e-6);
  ASSERT_NEAR(result2[3].pos.y, -0.84615946, 1e-6); 
  // cone 4
  ASSERT_NEAR(result2[4].pos.x, 1.88719496, 1e-6);
  ASSERT_NEAR(result2[4].pos.y, -0.613186813, 1e-6); 
  // cone 5
  ASSERT_NEAR(result2[5].pos.x, 3.49832869, 1e-6);
  ASSERT_NEAR(result2[5].pos.y, 0.0, 1e-6); 
  // cone 6
  ASSERT_NEAR(result2[6].pos.x, 3.24860189, 1e-6);
  ASSERT_NEAR(result2[6].pos.y, 1.05553474, 1e-6); 
  // cone 7
  ASSERT_NEAR(result2[7].pos.x, 0.798996534, 1e-6);
  ASSERT_NEAR(result2[7].pos.y, 0.580504962, 1e-6); 
  // cone 8
  ASSERT_NEAR(result2[8].pos.x, 1.30744398, 1e-6);
  ASSERT_NEAR(result2[8].pos.y, 1.79954225, 1e-6); 
  // cone 9
  ASSERT_NEAR(result2[9].pos.x, 0.135328205, 1e-6);
  ASSERT_NEAR(result2[9].pos.y, 0.416497388, 1e-6); 
  // cone 10
  ASSERT_NEAR(result2[10].pos.x, 0.0, 1e-6);
  ASSERT_NEAR(result2[10].pos.y, 1.09137246, 1e-6); 

}

TEST(Math, convertLLAtoNED){
  //regular test 1
  geometry_msgs::msg::Vector3 lla;
  lla.x = 45.976;
  lla.y = 7.658;
  lla.z = 4531;

  geometry_msgs::msg::Vector3 lla0;
  lla0.x = 46.017;
  lla0.y = 7.750;
  lla0.z = 1673;

  geometry_msgs::msg::Vector3 temp1 = convertLLAtoNED(lla, lla0);
  ASSERT_NEAR(temp1.x, -4556.32151384457, 1e-8);
  ASSERT_NEAR(temp1.y, -7134.75719597987, 1e-8);
  ASSERT_NEAR(temp1.z, -2852.39042394501, 1e-8);

  //regular test 2
  geometry_msgs::msg::Vector3 lla2;
  lla2.x = -7.397357;
  lla2.y = 109.846979;
  lla2.z = 0;

  geometry_msgs::msg::Vector3 lla02;
  lla02.x = -7.405680150087858;
  lla02.y = 109.86242302211231;
  lla02.z = 0;
  

  geometry_msgs::msg::Vector3 temp2 = convertLLAtoNED(lla2, lla02);
  ASSERT_NEAR(temp2.x, 920.450056632922, 1e-8);
  ASSERT_NEAR(temp2.y, -1705.00636382082, 1e-8);
  ASSERT_NEAR(temp2.z, 0.294731949886639, 1e-8);

  //edge case 1
  geometry_msgs::msg::Vector3 lla_1;
  lla_1.x = 36.162628;
  lla_1.y = 127.833212;
  lla_1.z = 0;

  geometry_msgs::msg::Vector3 lla0_1;
  lla0_1.x = 36.162628;
  lla0_1.y = 127.833212;
  lla0_1.z = 0;

  geometry_msgs::msg::Vector3 edge1 = convertLLAtoNED(lla_1, lla0_1);
  ASSERT_NEAR(edge1.x, 0, 1e-8);
  ASSERT_NEAR(edge1.y, 0, 1e-8); 
  ASSERT_NEAR(edge1.z, 0, 1e-8);

  //edge case 2
  geometry_msgs::msg::Vector3 lla_2;
  lla_2.x = 0;
  lla_2.y = 0;
  lla_2.z = 0;

  geometry_msgs::msg::Vector3 lla0_2;
  lla0_2.x = 0.000123;
  lla0_2.y = 0.000213;
  lla0_2.z = 0;

  geometry_msgs::msg::Vector3 edge2 = convertLLAtoNED(lla_2, lla0_2);
  ASSERT_NEAR(edge2.x, -13.600635926045872, 1e-8);
  ASSERT_NEAR(edge2.y, -23.7110515388584, 1e-8);
  ASSERT_NEAR(edge2.z, 5.8672390872005e-05, 1e-8);

  //edge case 3
  geometry_msgs::msg::Vector3 lla_3;
  lla_3.x = 0.000123;
  lla_3.y = 0.000213;
  lla_3.z = 0;

  geometry_msgs::msg::Vector3 lla0_3;
  lla0_3.x = 0;
  lla0_3.y = 0;
  lla0_3.z = 0;

  geometry_msgs::msg::Vector3 edge3 = convertLLAtoNED(lla_3, lla0_3);
  ASSERT_NEAR(edge3.x, 13.600635926045872, 1e-8);
  ASSERT_NEAR(edge3.y, 23.7110515388584, 1e-8);
  ASSERT_NEAR(edge3.z, 5.8672390872005e-05, 1e-8);


}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
