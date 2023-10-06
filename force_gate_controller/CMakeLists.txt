cmake_minimum_required(VERSION 3.16)
project(force_gate_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  hardware_interface
  moveit_ros_control_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  rsl
  tl_expected
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(force_gate_controller_parameters
  src/force_gate_controller_parameters.yaml
  include/force_gate_controller/validate_jtc_parameters.hpp
)

add_library(force_gate_controller SHARED
  src/force_gate_controller.cpp
  src/trajectory.cpp
)
target_compile_features(force_gate_controller PUBLIC cxx_std_17)
target_include_directories(force_gate_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/force_gate_controller>
)
target_link_libraries(force_gate_controller PUBLIC
  force_gate_controller_parameters
)
ament_target_dependencies(force_gate_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(force_gate_controller PRIVATE "FORCE_GATE_CONTROLLER_BUILDING_DLL" "_USE_MATH_DEFINES")
pluginlib_export_plugin_description_file(controller_interface force_gate_controller_plugin.xml)
pluginlib_export_plugin_description_file(moveit_ros_control_interface force_gate_controller_allocator_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_trajectory test/test_trajectory.cpp)
  target_link_libraries(test_trajectory force_gate_controller)

  ament_add_gmock(test_trajectory_controller
    test/test_trajectory_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_force_gate_controller.yaml)
  set_tests_properties(test_trajectory_controller PROPERTIES TIMEOUT 220)
  target_link_libraries(test_trajectory_controller
    force_gate_controller
  )

  ament_add_gmock(test_load_force_gate_controller
    test/test_load_force_gate_controller.cpp
  )
  target_link_libraries(test_load_force_gate_controller
    force_gate_controller
  )
  ament_target_dependencies(test_load_force_gate_controller
    controller_manager
    control_toolbox
    realtime_tools
    ros2_control_test_assets
  )

  ament_add_gmock(test_trajectory_actions
    test/test_trajectory_actions.cpp
  )
  set_tests_properties(test_trajectory_actions PROPERTIES TIMEOUT 300)
  target_link_libraries(test_trajectory_actions
    force_gate_controller
  )
endif()


install(
  DIRECTORY include/
  DESTINATION include/force_gate_controller
)
install(TARGETS
    force_gate_controller
    force_gate_controller_parameters
  EXPORT export_force_gate_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_force_gate_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
