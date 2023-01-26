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
* file: utfr_sim_bridge_unit_tests.cpp
* auth: Youssef Elhadad
* desc: sim_interface common functions unit tests
*/

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <utfr_sim_bridge_node.hpp>

using namespace utfr_dv::utfr_sim_bridge;


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
