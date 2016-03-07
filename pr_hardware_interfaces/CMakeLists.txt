cmake_minimum_required(VERSION 2.8.3)
project(pr_hardware_interfaces)

find_package(catkin REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PACKAGE_NAME}
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include)

## Declare a C++ library
# add_library(pr_hardware_interfaces
#   src/${PROJECT_NAME}/pr_hardware_interfaces.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(pr_hardware_interfaces ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

