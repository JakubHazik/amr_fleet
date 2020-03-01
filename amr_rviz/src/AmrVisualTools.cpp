//
// Created by jakub on 1.3.2020.
//

#include <amr_rviz/AmrVisualTools.h>


AmrVisualTools::AmrVisualTools(const std::string& graphTopic)
        : name("amr_rviz"),
          nh("~") {

    graphSubscriber = nh.subscribe(graphTopic, 1, &AmrVisualTools::graphCb, this);


    // todo config
    visual_tools.reset(new rvt::RvizVisualTools("map", "/amr_rviz_tools"));
    visual_tools->loadMarkerPub();  // create publisher before waiting

    // Clear messages
    visual_tools->deleteAllMarkers();
    visual_tools->enableBatchPublishing();
}

void AmrVisualTools::drawGraph() {
    std_msgs::ColorRGBA color = visual_tools->getColorScale(0);
    geometry_msgs::Vector3 scale = visual_tools->getScale(rvt::LARGE);

    for (const auto& node: graphNodes) {
        // draw nodes (spheres)
        auto pose = point2pose(node.second.position);
        visual_tools->publishSphere(pose, color, scale);
        publishLabelHelper(pose, std::to_string(node.second.uuid));

        // draw edges (arrows)
        for (const auto& successor: node.second.successors) {
            visual_tools->publishArrow(node.second.position, graphNodes[successor].position);
        }
    }

    visual_tools->trigger();
}

void AmrVisualTools::publishLabelHelper(geometry_msgs::Pose pose, const std::string& label) {
    pose.position.x -= 0.2;
    visual_tools->publishText(pose, label, rvt::WHITE, rvt::XXLARGE, false);
}

void AmrVisualTools::graphCb(const amr_msgs::Graph::ConstPtr& msg) {
    ROS_INFO_STREAM_NAMED(name, "Graph message received");

    for (const auto& node: msg->nodes) {
        graphNodes[node.uuid] = node;
    }
}

void AmrVisualTools::publish() {
    drawGraph();
}

geometry_msgs::Pose AmrVisualTools::point2pose(const geometry_msgs::Point& p) {
    geometry_msgs::Pose pose;
    pose.position.x = p.x;
    pose.position.y = p.y;
    return pose;
}
