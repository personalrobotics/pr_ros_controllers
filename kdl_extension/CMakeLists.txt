cmake_minimum_required(VERSION 2.8.3)
project(kdl_extension)

# set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
find_package(catkin REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIR} # included to make orocos_kdl compile
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/JointDynamicsData.cpp
  src/${PROJECT_NAME}/KdlChainIdRne.cpp
  src/${PROJECT_NAME}/KdlTreeId.cpp
  src/${PROJECT_NAME}/KdlTreeParser.cpp
  src/${PROJECT_NAME}/KdlTreeUtilities.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(kdl_extension ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )

 install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 )

 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 )

