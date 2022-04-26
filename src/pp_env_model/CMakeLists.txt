cmake_minimum_required(VERSION 3.0.2)
project(pp_env_model)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -Wno-sign-compare -Wno-unknown-pragmas -Wno-reorder")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -g -Wno-sign-compare -Wno-unknown-pragmas -Wno-reorder")


add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  derived_object_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  roscpp
  roslib
  message_generation
  path_planning_msgs
  common_msgs
  geometry_msgs
  tf
  tf2
  lanelet2_extension
  geographic_msgs
)

find_package(Eigen3 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ENV_MODEL_LIB
  CATKIN_DEPENDS   
    derived_object_msgs
    lanelet2_core
    lanelet2_io
    lanelet2_projection
    roscpp
    roslib
    message_generation
    path_planning_msgs
    common_msgs
    geometry_msgs
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(ENV_MODEL_LIB
  src/ped_reg_elem.cpp
  src/stop_sign_reg_elem.cpp
)
add_dependencies(ENV_MODEL_LIB
  ${catkin_EXPORTED_TARGETS}
)

add_executable(lanelet_service
  src/lanelet_service.cpp
)
add_dependencies(lanelet_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(lanelet_service ${catkin_LIBRARIES} stdc++fs)

add_executable(env_model
  src/env_model_main.cpp
  src/env_model.cpp
  src/routing.cpp
  src/env_model_utils.cpp
)
add_dependencies(env_model ${catkin_EXPORTED_TARGETS})
target_link_libraries(env_model ${catkin_LIBRARIES} ENV_MODEL_LIB)

add_executable(ego_localization_map_service
  src/ego_localization_map_service.cpp
)
add_dependencies(ego_localization_map_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(ego_localization_map_service ${catkin_LIBRARIES})

add_executable(rviz_lanelet_destination_bridge
  src/rviz_lanelet_destination_bridge.cpp
)
add_dependencies(rviz_lanelet_destination_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(rviz_lanelet_destination_bridge ${catkin_LIBRARIES})

install(
  TARGETS ENV_MODEL_LIB
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

