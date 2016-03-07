cmake_minimum_required(VERSION 2.8.3)
project(pr_ros_controllers)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  controller_interface 
  rosconsole
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS
    controller_interface
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/pr_joint_position_controller.cpp
  src/pr_joint_velocity_controller.cpp
  src/joint_mode_controller.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
