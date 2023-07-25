cmake_minimum_required(VERSION 3.1.2)
project(mrs_uav_px4_api)

set(CATKIN_DEPENDENCIES
  roscpp
  cmake_modules
  std_srvs
  mrs_uav_hw_api
  mavros
  mavros_msgs
  mrs_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  mrs_lib
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(mavros_msgs REQUIRED)

set(LIBRARIES
  MrsUavHwPx4Api
  )

catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  DEPENDS mavros_msgs
  LIBRARIES ${LIBRARIES}
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
  )

# Plugin

add_library(MrsUavHwPx4Api
  src/api.cpp
  )

add_dependencies(MrsUavHwPx4Api
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsUavHwPx4Api
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )


## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
