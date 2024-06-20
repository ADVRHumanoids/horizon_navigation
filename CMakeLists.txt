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
  src/obstacle.cpp
  src/obstacles_generator.cpp
  src/map_transformer.cpp
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

add_executable(obstacle_generator_node
    src/obstacle_generator_node.cpp)

target_link_libraries(obstacle_generator_node
    horizon_navigation
    ${catkin_LIBRARIES})

add_executable(map_transformer_node
    src/map_transformer_node.cpp)

target_link_libraries(map_transformer_node
    horizon_navigation
    ${catkin_LIBRARIES})

add_executable(sonar_node
    src/ros/sonar_node.cpp)

target_link_libraries(sonar_node
    horizon_navigation
    ${catkin_LIBRARIES})

#include(GNUInstallDirs)
install(TARGETS  ${LIBRARY_TARGET_NAME}
        EXPORT   ${LIBRARY_TARGET_NAME}
        LIBRARY  DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE  DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        RUNTIME  DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)

install(TARGETS obstacle_generator_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS map_transformer_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS sonar_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
add_subdirectory(bindings/python/)
# add_subdirectory(tests/)
