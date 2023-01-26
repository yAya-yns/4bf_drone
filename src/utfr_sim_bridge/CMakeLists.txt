cmake_minimum_required(VERSION 3.5)
project(utfr_sim_bridge)

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
    geometry_msgs
    utfr_common
    utfr_msgs
    eufs_msgs
    ackermann_msgs
)

## Find Dependencies
foreach(DEPENDENCY IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

## Export Dependencies for downstream packages
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})


## Add Executables
add_executable(utfr_sim_bridge src/utfr_sim_bridge.cpp)
target_link_libraries(utfr_sim_bridge utfr_sim_bridge_library)
# Set Target Dependencies  
ament_target_dependencies(utfr_sim_bridge ${THIS_PACKAGE_INCLUDE_DEPENDS})


## Add Headers to executables
target_include_directories(utfr_sim_bridge
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
      

## Add Libaries
add_library(utfr_sim_bridge_library
   src/utfr_sim_bridge_node.cpp)
# Set Target Dependencies  
ament_target_dependencies(utfr_sim_bridge_library ${THIS_PACKAGE_INCLUDE_DEPENDS})

## Add Headers to library
target_include_directories(utfr_sim_bridge_library
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
install(TARGETS utfr_sim_bridge_library
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include/${PROJECT_NAME})

## Install Executables
install(TARGETS utfr_sim_bridge_library utfr_sim_bridge
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
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ## Define Test Executables
  ament_add_gtest(utfr_sim_bridge_unit_tests
    test/utfr_sim_bridge_unit_tests.cpp
  )
  ament_target_dependencies(utfr_sim_bridge_unit_tests ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ## Find Headers for Testing files
  target_include_directories(utfr_sim_bridge_unit_tests
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ## Install testing files
  install(TARGETS utfr_sim_bridge_unit_tests
      DESTINATION test/${PROJECT_NAME})

endif()


ament_package()