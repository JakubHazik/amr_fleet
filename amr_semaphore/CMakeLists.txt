cmake_minimum_required(VERSION 2.8.3)
project(amr_semaphore)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        amr_msgs
        roscpp
        rviz_visual_tools
        amr_graph_generator
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES semaphore_client
        #  CATKIN_DEPENDS amr_msgs amr_pose_controller roscpp
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# Server
add_executable(semaphore_server
        src/server/SemaphoreServer.cpp
        src/server/SemaphoreServerNode.cpp
        src/server/LockContainers.cpp
        )
add_dependencies(semaphore_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(semaphore_server
        ${catkin_LIBRARIES}
        )

# Client
add_library(semaphore_client
        src/client/SemaphoreClient.cpp
        src/client/SemaphoreAutomaticClient.cpp
        )
add_dependencies(semaphore_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(semaphore_client
        ${catkin_LIBRARIES}
        )
