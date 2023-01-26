cmake_minimum_required(VERSION 3.5)
project(mapping)

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
    tf2
    tf2_ros
    tf2_geometry_msgs
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
add_executable(mapping src/mapping.cpp)
target_link_libraries(mapping mapping_library)
# Set Target Dependencies  
ament_target_dependencies(mapping ${THIS_PACKAGE_INCLUDE_DEPENDS})

## Add Headers to executables
target_include_directories(mapping
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)

## Add Libaries
add_library(mapping_library
   src/mapping_node.cpp
   src/fusion/fusion.cpp)
# Set Target Dependencies  
ament_target_dependencies(mapping_library ${THIS_PACKAGE_INCLUDE_DEPENDS})

## Add Headers to library
target_include_directories(mapping_library
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
install(TARGETS mapping_library
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include/${PROJECT_NAME})

## Install Executables
install(TARGETS mapping 
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
  ament_add_gtest(mapping_test
    test/mapping_test.cpp
  )
  ament_target_dependencies(mapping_test ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ## Find Headers for Testing files
  target_include_directories(mapping_test
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ## Link required files for testing:
  target_sources(mapping_test PRIVATE src/fusion/fusion.cpp)

  ## Install testing files
  install(TARGETS mapping_test
      DESTINATION test/${PROJECT_NAME})

endif()

ament_package()
