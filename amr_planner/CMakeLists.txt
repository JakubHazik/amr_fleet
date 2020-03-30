cmake_minimum_required(VERSION 2.8.3)
project(amr_planner)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        amr_msgs
        std_srvs
        )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS graph)

catkin_package(

)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        libMultiRobotPlanning/include
)

add_executable(amr_planner
        src/AmrPlannerNode.cpp
        src/RosWrapper.cpp
        src/Graph.cpp
        src/GraphSearchBoost.cpp
        src/GraphSearchMultiRobot.cpp
        src/DataTypesAndConversions.cpp
        )

add_dependencies(amr_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
        amr_planner
        ${catkin_LIBRARIES}
)
