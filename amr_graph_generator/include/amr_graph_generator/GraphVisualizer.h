//
// Created by jakub on 1.3.2020.
//

#ifndef PROJECT_GRAPHVISUALIZER_H
#define PROJECT_GRAPHVISUALIZER_H

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <amr_msgs/Graph.h>
#include <amr_graph_generator/DataTypes.h>
#include <map>

namespace rvt = rviz_visual_tools;

class GraphVisualizer {
public:

    GraphVisualizer(std::vector<Node> graph);

    void publish();

private:

    rvt::RvizVisualToolsPtr visual_tools;
    std::map<unsigned int, Node> graphNodes;

    void drawGraph();

    geometry_msgs::Pose node2pose(const Node& n);

    geometry_msgs::Point node2point(const Node& n);

    void publishLabelHelper(geometry_msgs::Pose pose, const std::string& label);

};


#endif //PROJECT_GRAPHVISUALIZER_H
