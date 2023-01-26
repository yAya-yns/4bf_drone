cmake_minimum_required(VERSION 3.8)
project(state_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Default to C++14, copied from example
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## List all Dependencies:, copied from example
set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    rclcpp
    rclcpp_components
    std_msgs
    geometry_msgs
    tf2
    tf2_ros
    utfr_common
    utfr_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

#copied from example
foreach(DEPENDENCY IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

## Export Dependencies for downstream packages, from example
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

#ament_package()

## Add Executables
add_executable(state_estimation src/state_estimation.cpp)
target_link_libraries(state_estimation state_estimation_library)
# Set Target Dependencies  
ament_target_dependencies(state_estimation ${THIS_PACKAGE_INCLUDE_DEPENDS})

## Add Headers to executables
target_include_directories(state_estimation
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)

## Add Libaries
add_library(state_estimation_library
   src/state_estimation_node.cpp)
# Set Target Dependencies  
ament_target_dependencies(state_estimation_library 
    ${THIS_PACKAGE_INCLUDE_DEPENDS})

## Add Headers to library
target_include_directories(state_estimation_library
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
install(TARGETS state_estimation_library
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include/${PROJECT_NAME})

## Install Executables
install(TARGETS state_estimation
    DESTINATION lib/${PROJECT_NAME})

## Install Launch file
install (
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

## Install Config files
install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_state_estimation_package.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
ament_package()

