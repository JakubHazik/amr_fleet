cmake_minimum_required(VERSION 2.8.3)
project(amr_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        message_generation
        std_msgs
        )

add_message_files(
        FILES
        Graph.msg
        Node.msg
        Point.msg
)

add_service_files(
        FILES
        PlanPathPoints.srv
        PlanPathNodes.srv
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES amr_msgs
        #  CATKIN_DEPENDS geometry_msgs message_generation std_msgs
        #  DEPENDS system_lib
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)
