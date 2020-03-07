cmake_minimum_required(VERSION 2.8.3)
project(amr_graph_generator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        amr_msgs
        rviz_visual_tools
        )
find_package(Eigen3 REQUIRED)

find_library(DXF_LIB dxflib)
if (NOT DXF_LIB)
    message(WARNING "dxf library not found")
    message(WARNING "run 'sudo apt install libdxflib-dev' to install the dxf library")
endif ()

catkin_package(
        CATKIN_DEPENDS amr_msgs rviz_visual_tools
        DEPENDS dxflib
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_executable(amr_graph_generator
        src/GraphGeneratorNode.cpp
        src/PathDxfParser.cpp
        src/GraphVisualizer.cpp
        )

add_dependencies(amr_graph_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(amr_graph_generator
        ${catkin_LIBRARIES}
        dxflib
        )
