cmake_minimum_required(VERSION 2.8.3)
project(amr_pose_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        amr_msgs
        geometry_msgs
        roscpp
        actionlib
        tf
)

catkin_package(

)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(pose_controller
        src/Controller.cpp
        src/PoseControllerNode.cpp
        src/RosWrapper.cpp
)

add_dependencies(pose_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pose_controller
        ${catkin_LIBRARIES}
)
