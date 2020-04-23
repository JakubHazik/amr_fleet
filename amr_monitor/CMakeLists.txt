cmake_minimum_required(VERSION 2.8.3)
project(amr_monitor)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        amr_msgs
        amr_pose_controller
        amr_task_manager
        rviz_visual_tools
        geometry_msgs
        )
find_package(Boost COMPONENTS filesystem REQUIRED)

catkin_package(
#        INCLUDE_DIRS include
#        LIBRARIES amr_monitor_widget
        CATKIN_DEPENDS roscpp amr_msgs
        #  DEPENDS system_lib
)

include_directories(
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${catkin_INCLUDE_DIRS}
        )

add_executable(monitor_client
        src/client/MonitoringNode.cpp
        src/client/DataCollector.cpp
        )

add_dependencies(monitor_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(monitor_client
        ${catkin_LIBRARIES}
        )

# monitor logger
add_executable(monitor_logger
        src/server/MonitoringLoggerNode.cpp
)
add_dependencies(monitor_logger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(monitor_logger
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
)

# monitor visualizer
add_executable(monitor_visualizer
        src/server/MonitoringVisualizer.cpp
        )
add_dependencies(monitor_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(monitor_visualizer
        ${catkin_LIBRARIES}
        )