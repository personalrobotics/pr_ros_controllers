cmake_minimum_required(VERSION 2.8.3)
project(pr_hardware_interfaces)

find_package(catkin REQUIRED COMPONENTS hardware_interface)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS hardware_interface
)

include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/PositionCommandInterface.cpp
  src/TriggerableInterface.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_definitions(
  ${catkin_DEFINITIONS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

