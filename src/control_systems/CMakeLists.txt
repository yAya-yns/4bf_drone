cmake_minimum_required(VERSION 3.5)
project(control_systems)

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
)

## Find Dependencies
foreach(DEPENDENCY IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

## Export Dependencies for downstream packages
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})


## Add Executables
add_executable(control_systems src/control_systems.cpp)
target_link_libraries(control_systems control_systems_library)
# Set Target Dependencies  
ament_target_dependencies(control_systems ${THIS_PACKAGE_INCLUDE_DEPENDS})


## Add Headers to executables
target_include_directories(control_systems
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
      

## Add Libaries
add_library(control_systems_library
   src/control_systems_node.cpp
   src/controller/pure_pursuit_controller.cpp
   src/controller/pid_controller.cpp)
# Set Target Dependencies  
ament_target_dependencies(control_systems_library ${THIS_PACKAGE_INCLUDE_DEPENDS})

## Add Headers to library
target_include_directories(control_systems_library
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
install(TARGETS control_systems_library
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include/${PROJECT_NAME})

## Install Executables
install(TARGETS control_systems_library control_systems
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
  ament_add_gtest(control_systems_unit_tests
    test/controllers_unit_tests.cpp
  )
  ament_target_dependencies(control_systems_unit_tests ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ## Find Headers for Testing files
  target_include_directories(control_systems_unit_tests
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ## Link required files for testing:
  target_sources(control_systems_unit_tests PRIVATE src/controller/pure_pursuit_controller.cpp)

  ## Install testing files
  install(TARGETS control_systems_unit_tests
      DESTINATION test/${PROJECT_NAME})

endif()


ament_package()
