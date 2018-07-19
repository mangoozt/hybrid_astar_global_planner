cmake_minimum_required(VERSION 2.8.3)
project(hybrid_astar_global_planner)
## C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## OPEN MOTION PLANNING LIBRARY
#find_package(OMPL REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)
set(OMPL_INCLUDE_DIRS "/usr/local/include/ompl")
set(OMPL_LIBRARIES "/usr/local/lib/libompl.so")
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${OMPL_INCLUDE_DIRS})
include_directories(include include)

find_package(catkin REQUIRED
  COMPONENTS
    angles
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf
)

generate_dynamic_reconfigure_options(
  cfg/GlobalPlanner.cfg
)

catkin_package(
  INCLUDE_DIRS include ${OMPL_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${OMPL_LIBRARIES}
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    std_msgs
    pluginlib
    roscpp
    rospy
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)
link_directories(${OMPL_LIBRARY_DIRS})
add_library(${PROJECT_NAME}
  src/hybrid_astar/algorithm.cpp
  src/hybrid_astar/dynamicvoronoi.cpp
  src/hybrid_astar/path.cpp
  src/hybrid_astar/visualize.cpp
  src/hybrid_astar/bucketedqueue.cpp
  src/hybrid_astar/collisiondetection.cpp
  src/hybrid_astar/node2d.cpp
  src/hybrid_astar/smoother.cpp
  src/hybrid_astar/dubins.cpp
  src/hybrid_astar/node3d.cpp
  src/hybrid_astar/tf_broadcaster.cpp
  src/planner_core.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

#add_executable(hybrid_planner
#  src/plan_node.cpp
#)
#add_dependencies(hybrid_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(hybrid_planner
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(hybrid_planner ${OMPL_LIBRARIES})

#install(TARGETS ${PROJECT_NAME} hybrid_planner
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  PATTERN ".svn" EXCLUDE)

#install(FILES bgp_plugin.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
