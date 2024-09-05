cmake_minimum_required(VERSION 3.5)

project(horizon_navigation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 3.3 REQUIRED)
find_package(casadi 3.5.5 REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             visualization_msgs
             nav_msgs
             tf2
             tf2_ros
             grid_map_core
             grid_map_ros)

include_directories(
    include/
    src/
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

catkin_package()

set(LIBRARY_TARGET_NAME horizon_navigation) # set library name

add_library(${LIBRARY_TARGET_NAME} SHARED
  src/obstacles_generator.cpp
  src/obstacle.cpp
  src/navigation.cpp
  src/velodyne.cpp
  src/sonar.cpp
)

target_link_libraries(${LIBRARY_TARGET_NAME}
    PUBLIC
    ${catkin_LIBRARIES}
    casadi
    yaml-cpp)

target_include_directories(${LIBRARY_TARGET_NAME}
    INTERFACE
    ${EIGEN3_INCLUDE_DIR})

# =============================

add_library(horizon_navigation_ros SHARED
    src/ros/lidar_sonar_ros.cpp
    src/ros/velodyne_ros.cpp
    src/ros/navigation_ros.cpp
    src/ros/obstacle_generator_ros.cpp
    src/ros/sonar_ros.cpp)

target_link_libraries(horizon_navigation_ros
    PUBLIC
    ${catkin_LIBRARIES}
    horizon_navigation)

# =============================

#add_executable(obstacle_generator_node
#    src/ros/obstacle_generator_node.cpp)

#target_link_libraries(obstacle_generator_node
#    horizon_navigation_ros
#    ${catkin_LIBRARIES})

# =============================

add_executable(lidar_sonar_node
    src/ros/nodes/lidar_sonar_node.cpp)

target_link_libraries(lidar_sonar_node
    horizon_navigation_ros
    ${catkin_LIBRARIES})

# =============================

add_executable(velodyne_node
    src/ros/nodes/velodyne_node.cpp)

target_link_libraries(velodyne_node
    horizon_navigation_ros
    ${catkin_LIBRARIES})

# =============================

add_executable(navigation_node
    src/ros/nodes/navigation_node.cpp)

target_link_libraries(navigation_node
    horizon_navigation_ros
    ${catkin_LIBRARIES})

# =============================

add_executable(sonar_node
    src/ros/nodes/sonar_node.cpp)

target_link_libraries(sonar_node
    horizon_navigation_ros
    ${catkin_LIBRARIES})

# =============================

add_executable(grid_map_fusion_node
    src/ros/nodes/grid_map_fusion_node.cpp)

target_link_libraries(grid_map_fusion_node
    horizon_navigation_ros
    ${catkin_LIBRARIES})

# =============================

install(TARGETS  ${LIBRARY_TARGET_NAME}
        EXPORT   ${LIBRARY_TARGET_NAME}
        LIBRARY  DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE  DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        RUNTIME  DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)

install(TARGETS  horizon_navigation_ros
        EXPORT   horizon_navigation_ros
        LIBRARY  DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE  DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        RUNTIME  DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)

#install(TARGETS obstacle_generator_node
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(TARGETS navigation_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS sonar_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS grid_map_fusion_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS velodyne_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_subdirectory(bindings/python/)
# add_subdirectory(tests/)
