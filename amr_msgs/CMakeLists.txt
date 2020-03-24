cmake_minimum_required(VERSION 2.8.3)
project(amr_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        message_generation
        std_msgs
        actionlib_msgs
        )

add_message_files(
        FILES
        Graph.msg
        Node.msg
        Point.msg
        Task.msg
        TaskId.msg
        GetTaskErrorCodes.msg
        ClientInfo.msg
        HardwareStatus.msg
)

add_service_files(
        FILES
        PlanPathPoints.srv
        PlanPathNodes.srv
        GetTask.srv
        ResetTask.srv
        LockPoint.srv
        SetNodeProperties.srv
)

add_action_files(
        FILES
        PerformGoals.action
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
        actionlib_msgs
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
