cmake_minimum_required(VERSION 3.10.0)
project(airsim_moveit_navigation)

set(AIRSIM_ROOT /home/user/Code/AirSim)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

set(CMAKE_CXX_STANDARD 11) 
set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB rpc)
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

include_directories(${AIRSIM_ROOT}/AirLib/deps/eigen3)

include_directories(
  include
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    moveit_core 
    moveit_ros_planning_interface 
    std_msgs 
    actionlib
    actionlib_msgs
    message_generation
    geometry_msgs
    sensor_msgs
    pcl_conversions
    pcl_ros
)

find_package(octomap REQUIRED)
find_package(OpenMP REQUIRED)

## Generate messages in the 'msg' folder

add_action_files(
  DIRECTORY action
  FILES AirSim_Navigation.action
)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS roscpp moveit_core moveit_ros_planning_interface std_msgs actionlib_msgs geometry_msgs message_runtime)

if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
endif()

include_directories(include ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(airsim_navigator src/AirSim_Navigator.cpp src/airsim_navigator_node.cpp)
target_compile_options(airsim_navigator PRIVATE ${OpenMP_FLAGS} ${CMAKE_CXX_FLAGS})
target_link_libraries(airsim_navigator AirLib ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenMP_LIBS})

add_executable(airsim_navigator_client src/airsim_navigator_client.cpp)
target_link_libraries(airsim_navigator_client ${catkin_LIBRARIES})
add_dependencies(airsim_navigator_client ${airsim_moveit_navigation_EXPORTED_TARGETS})

add_executable(depth_segmentation src/depth_segmentation.cpp)
target_link_libraries(depth_segmentation ${catkin_LIBRARIES})
add_dependencies(depth_segmentation ${airsim_moveit_navigation_EXPORTED_TARGETS})

add_executable(airsim_nbv_planner src/airsim_nbv_planner.cpp)
target_compile_options(airsim_nbv_planner PRIVATE ${OpenMP_FLAGS} ${CMAKE_CXX_FLAGS})
target_link_libraries(airsim_nbv_planner AirLib ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenMP_LIBS})