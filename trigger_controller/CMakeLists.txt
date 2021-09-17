cmake_minimum_required(VERSION 3.0.0)
project(trigger_controller VERSION 0.0.0)

find_package(catkin
  REQUIRED COMPONENTS
      actionlib
      actionlib_msgs
      controller_interface
      pr_control_msgs
      pr_hardware_interfaces
      realtime_tools
)

find_package(Boost REQUIRED)

catkin_package(
INCLUDE_DIRS include
LIBRARIES trigger_controller
CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    controller_interface
    pr_control_msgs
    pr_hardware_interfaces
    realtime_tools
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
   src/TriggerController.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES ros_control_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_trigger_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
