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
* file: topics.hpp
* auth: Kelvin Cui
* desc: ros2 topics list
*/
#pragma once

//System Requirements
#include <string>

namespace utfr_dv {
namespace frames {

const std::string kWorld{"world"};
const std::string kBase{"base_link"};
const std::string kCamera{"base_camera"};
const std::string kLidar{"base_lidar"};

} // namespace util
} // namespace utfr_dv