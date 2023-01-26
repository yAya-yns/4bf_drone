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
* file: mission_manager.cpp
* auth: Kelvin Cui
* desc: mission manager main executable
*/
#include <rclcpp/rclcpp.hpp>
#include <mission_manager_node.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<utfr_dv::mission_manager::MissionManagerNode>());
  rclcpp::shutdown();
  return 0;
}

