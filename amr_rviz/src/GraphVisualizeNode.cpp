//
// Created by jakub on 1.3.2020.
//


#include <amr_rviz/AmrVisualTools.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "amr_visual_tools");
    ROS_INFO_STREAM("Visual Tools AMR");

    // todo config
    AmrVisualTools amr_tools("/graph_generator/graph");

    ros::Rate r(1);
    while (ros::ok())
    {
        amr_tools.publish();
        r.sleep();
    }
    return 0;
}