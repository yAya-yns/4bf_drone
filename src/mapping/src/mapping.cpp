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
* file: mapping.cpp
* auth: Kelvin Cui
* desc: mapping main executable
*/
#include <rclcpp/rclcpp.hpp>
#include <mapping_node.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<utfr_dv::mapping::MappingNode>());
  rclcpp::shutdown();
  return 0;
}

