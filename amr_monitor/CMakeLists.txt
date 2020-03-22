cmake_minimum_required(VERSION 2.8.3)
project(amr_monitor)

add_compile_options(-std=c++11)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        amr_msgs
        amr_pose_controller
        amr_task_manager
        rviz_visual_tools
        geometry_msgs
        )
find_package(Qt5Widgets REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES amr_monitor_widget
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

# ==============================
# widget
# ==============================
set (rqt_example_cpp_SRCS
        src/qt_widget/MonitorWidget.cpp
        src/qt_widget/ClientMonitorWidget.cpp
        )

set(rqt_example_cpp_HDRS
        include/amr_monitor/qt_widget/MonitorWidget.h
        include/amr_monitor/qt_widget/ClientMonitorWidget.h
        )

set(rqt_example_cpp_UIS
        src/qt_widget/MonitorWidget.ui
        src/qt_widget/ClientMonitorWidget.ui
        )

qt5_wrap_cpp(rqt_example_cpp_MOCS ${rqt_example_cpp_HDRS})
qt5_wrap_ui(rqt_example_cpp_UIS_H ${rqt_example_cpp_UIS})

add_library(amr_monitor_widget
        ${rqt_example_cpp_SRCS}
        ${rqt_example_cpp_MOCS}
        ${rqt_example_cpp_UIS_H}
        )
target_link_libraries(amr_monitor_widget
        ${catkin_LIBRARIES}
        Qt5::Widgets
        )

