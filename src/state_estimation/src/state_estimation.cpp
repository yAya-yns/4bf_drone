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
* file: state_estimation.cpp
* auth: Alyssa Wing
* desc: state estimation main executable for ros2 
*/
#include <rclcpp/rclcpp.hpp>
#include <state_estimation_node.hpp>

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(
        std::make_shared<utfr_dv::state_estimation::StateEstimationNode>());
  rclcpp::shutdown();
  return 0;
}