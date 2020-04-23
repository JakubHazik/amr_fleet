//
// Created by jakub on 1.3.2020.
//

#include <amr_graph_generator/GraphVisualizer.h>


GraphVisualizer::GraphVisualizer(const std::vector<Node>& graph) {

    for (const auto& node: graph) {
        graphNodes[node.uuid] = node;
    }

    // todo configurable
    visual_tools.reset(new rvt::RvizVisualTools("map", "/generated_graph"));
    visual_tools->loadMarkerPub(false, true);  // create publisher before waiting

    // Clear messages
    visual_tools->deleteAllMarkers();
    visual_tools->enableBatchPublishing();
}

void GraphVisualizer::drawGraph() {
    for (const auto& node: graphNodes) {
        // draw nodes (spheres)
        geometry_msgs::Point point = node2point(node.second);
        if (node.second.isBidirectional) {
            visual_tools->publishSphere(point, rvt::colors::CYAN, rvt::scales::LARGE);
        } else {
            visual_tools->publishSphere(point, rvt::colors::RED, rvt::scales::LARGE);
        }
        publishLabelHelper(node2pose(node.second), std::to_string(node.second.uuid));

        // draw edges (arrows)
        for (const auto& successor: node.second.successors) {
            visual_tools->publishArrow(point, node2point(graphNodes[successor]), rvt::colors::BLUE, rvt::scales::MEDIUM);
        }
    }

    visual_tools->trigger();
}

void GraphVisualizer::publishLabelHelper(geometry_msgs::Pose pose, const std::string& label) {
    pose.position.x -= 0.1;
    visual_tools->publishText(pose, label, rvt::WHITE, rvt::XXLARGE, false);
}

void GraphVisualizer::publish() {
    visual_tools->deleteAllMarkers();
    drawGraph();
}

geometry_msgs::Pose GraphVisualizer::node2pose(const Node& n) {
    geometry_msgs::Pose pose;
    pose.position.x = n.posX;
    pose.position.y = n.posY;
    return pose;
}

geometry_msgs::Point GraphVisualizer::node2point(const Node& n) {
    geometry_msgs::Point point;
    point.x = n.posX;
    point.y = n.posY;
    return point;
}