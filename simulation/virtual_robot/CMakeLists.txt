cmake_minimum_required(VERSION 2.8.3)
project(virtual_robot)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        tf
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES virtual_robot
        #  CATKIN_DEPENDS geometry_msgs roscpp
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(virtual_robot
        src/VirtualRobotNode.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(virtual_robot
        ${catkin_LIBRARIES}
)
