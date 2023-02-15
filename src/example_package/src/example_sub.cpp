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
* file: example_sub.cpp
* auth: Kelvin Cui
* desc: example main executable for ros2 template
*/
#include <rclcpp/rclcpp.hpp>
#include <example_sub_node.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<utfr_dv::example::ExampleSubNode>());
  rclcpp::shutdown();
  return 0;
}
