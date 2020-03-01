//
// Created by jakub on 1.3.2020.
//

#ifndef PROJECT_AMRVISUALTOOLS_H
#define PROJECT_AMRVISUALTOOLS_H

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <amr_msgs/Graph.h>
#include <map>

namespace rvt = rviz_visual_tools;


class AmrVisualTools {
public:

    AmrVisualTools(const std::string& graphTopic);

    void drawGraph();

    void publish();

private:
    ros::NodeHandle nh;
    ros::Subscriber graphSubscriber;

    rvt::RvizVisualToolsPtr visual_tools;

    std::string name;

    amr_msgs::Graph graphMsg;

    std::map<unsigned int, amr_msgs::Node> graphNodes;

    geometry_msgs::Pose point2pose(const geometry_msgs::Point& p);

    void graphCb(const amr_msgs::Graph::ConstPtr& msg);

    void publishLabelHelper(geometry_msgs::Pose pose, const std::string& label);

};


#endif //PROJECT_AMRVISUALTOOLS_H
