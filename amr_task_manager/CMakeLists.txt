cmake_minimum_required(VERSION 2.8.3)
project(amr_task_manager)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        amr_msgs
        amr_planner
        amr_pose_controller
        roscpp
        actionlib
        roslib
        )

find_package(yaml-cpp REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES amr_task_manager
        #  CATKIN_DEPENDS amr_msgs amr_planner amr_pose_controller roscpp
          DEPENDS yaml-cpp
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# Server Node
add_executable(task_manager_server
        src/Server/TaskManagerServer.cpp
        src/Server/TaskManagerServerNode.cpp
        src/Server/ClientRepresentation.cpp
)

add_dependencies(task_manager_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(task_manager_server
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${Boost_LIBRARIES}
)


# Client node
add_executable(task_manager_client
        src/Client/TaskManagerClientNode.cpp
        src/Client/TaskManagerClient.cpp
)

add_dependencies(task_manager_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(task_manager_client
        ${catkin_LIBRARIES}
)
