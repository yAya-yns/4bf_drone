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
* file: utfr_sim_bridge.cpp
* auth: Youssef Elhadad
* desc: utfr sim bridge main executable for ros2 sub
*/
#include <rclcpp/rclcpp.hpp>
#include <utfr_sim_bridge_node.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<utfr_dv::utfr_sim_bridge::UTFRSimBridge>());
  rclcpp::shutdown();
  return 0;
}
