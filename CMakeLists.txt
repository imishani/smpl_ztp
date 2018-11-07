cmake_minimum_required(VERSION 2.8.12)
project(smpl_ztp)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

#######################
# Import dependencies #
#######################

find_package(Boost REQUIRED COMPONENTS filesystem serialization)

find_package(Eigen3 REQUIRED)

find_package(
    catkin
    REQUIRED
    COMPONENTS
        eigen_conversions
        leatherman
        roscpp
        sbpl_kdl_robot_model
        smpl_ros
        smpl_moveit_interface
        moveit_ros_planning
        moveit_ros_planning_interface
        trajectory_msgs
        moveit_msgs)
find_package(smpl REQUIRED)

find_package(sbpl REQUIRED)
set(sbpl_INCLUDE_DIRS ${SBPL_INCLUDE_DIRS})
set(sbpl_LIBRARIES ${SBPL_LIBRARIES})


catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        smpl_ztp
    CATKIN_DEPENDS
    DEPENDS
        Boost
        sbpl
        smpl)

##########################
# Build smpl_ztp library #
##########################

add_library(
    smpl_ztp
    src/graph/workspace_lattice_zero.cpp
    src/heuristic/workspace_dist_heuristic.cpp
    src/search/arastar_zero.cpp
    src/ros/zero_time_planner.cpp)

target_compile_options(smpl_ztp PUBLIC -std=c++11)

target_include_directories(smpl_ztp PRIVATE include)
target_include_directories(smpl_ztp SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_include_directories(smpl_ztp SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})
target_include_directories(smpl_ztp SYSTEM PUBLIC ${SBPL_INCLUDE_DIRS})

target_link_libraries(smpl_ztp PUBLIC ${catkin_LIBRARIES})
target_link_libraries(smpl_ztp PUBLIC smpl::smpl)
target_link_libraries(smpl_ztp PUBLIC ${Boost_LIBRARIES})
target_link_libraries(smpl_ztp PUBLIC ${SBPL_LIBRARIES})

#######################################
# Build call_planner_ztp test program #
#######################################

add_executable(call_planner_ztp src/call_planner.cpp src/ros/planner_interface.cpp src/collision_space_scene.cpp)

target_compile_options(smpl_ztp PUBLIC -std=c++11)

target_include_directories(call_planner_ztp PRIVATE include)
target_include_directories(call_planner_ztp SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
target_include_directories(call_planner_ztp SYSTEM PRIVATE ${Boost_INCLUDE_DIRS})
target_include_directories(call_planner_ztp SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
target_include_directories(call_planner_ztp SYSTEM PRIVATE ${SBPL_INCLUDE_DIRS})

target_link_libraries(call_planner_ztp PRIVATE smpl_ztp)
target_link_libraries(call_planner_ztp PRIVATE ${catkin_LIBRARIES})
target_link_libraries(call_planner_ztp PRIVATE ${SBPL_LIBRARIES})
target_link_libraries(call_planner_ztp PRIVATE ${Boost_LIBRARIES})

############################
# Install smpl_ztp library #
############################

install(
    DIRECTORY include/smpl_ztp/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
    TARGETS smpl_ztp
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#################################
# Install call_planner_ztp test #
#################################

install(
    TARGETS call_planner_ztp
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

