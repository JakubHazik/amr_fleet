cmake_minimum_required(VERSION 2.8.3)
project(amr_gui)

add_compile_options(-std=c++11)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
  amr_monitor
  roscpp
)

find_package(Qt5 REQUIRED COMPONENTS Gui Core Widgets)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES amr_gui
#  CATKIN_DEPENDS amr_monitor roscpp
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

set (rqt_example_cpp_SRCS
        src/main.cpp
        src/MainWindow.cpp
        src/monitoring/ClientMonitorWidget.cpp
        src/monitoring/MonitorWidget.cpp
        )

set(rqt_example_cpp_HDRS
        include/amr_gui/MainWindow.h
        include/amr_gui/monitoring/ClientMonitorWidget.h
        include/amr_gui/monitoring/MonitorWidget.h
        )

set(rqt_example_cpp_UIS
        ui/MainWindow.ui
        ui/monitoring/ClientMonitorWidget.ui
        ui/monitoring/MonitorWidget.ui
        )

qt5_wrap_cpp(rqt_example_cpp_MOCS ${rqt_example_cpp_HDRS})
qt5_wrap_ui(rqt_example_cpp_UIS_H ${rqt_example_cpp_UIS})

add_executable(amr_gui
        ${rqt_example_cpp_SRCS}
        ${rqt_example_cpp_MOCS}
        ${rqt_example_cpp_UIS_H}
        )

target_link_libraries(amr_gui
        ${catkin_LIBRARIES}
        Qt5::Widgets
        )
