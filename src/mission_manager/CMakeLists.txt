cmake_minimum_required(VERSION 3.5)
project(mission_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## List all Dependencies:
set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    rclcpp
    rclcpp_components
    std_msgs
    utfr_msgs
    sensor_msgs
    geometry_msgs
    utfr_common
)

## Find Dependencies
foreach(DEPENDENCY IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

## Export Dependencies for downstream packages
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

## Add Executables
add_executable(mission_manager src/mission_manager.cpp)
target_link_libraries(mission_manager mission_manager_library)
# Set Target Dependencies  
ament_target_dependencies(mission_manager ${THIS_PACKAGE_INCLUDE_DEPENDS})

## Add Headers to executables
target_include_directories(mission_manager
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)

## Add Libaries
add_library(mission_manager_library
   src/mission_manager_node.cpp
   src/heartbeat_monitor/heartbeat_monitor.cpp)
# Set Target Dependencies  
ament_target_dependencies(mission_manager_library ${THIS_PACKAGE_INCLUDE_DEPENDS})

## Add Headers to library
target_include_directories(mission_manager_library
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)

#############
## Install ##
#############

## Install Headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

## Install Libraries
install(TARGETS mission_manager_library
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include/${PROJECT_NAME})

## Install Executables
install(TARGETS mission_manager 
    DESTINATION lib/${PROJECT_NAME})

## Install Launch file
install (
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

## Install Config file
install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_example_package.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
ament_package()
