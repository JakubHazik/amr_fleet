//
// Created by jakub on 21.2.2020.
//

#include <amr_planner/RosWrapper.h>

#include <boost/graph/directed_graph.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>



RosWrapper::RosWrapper(ros::NodeHandle& nh) {
    segSubscriber = nh.subscribe("segments", 5, &RosWrapper::newSegmentsCb, this);
}

void RosWrapper::newSegmentsCb(const tuw_multi_robot_msgs::Graph::ConstPtr& segments) {
    ROS_INFO("New graph segments received");

    for (const auto &node: segments->vertices) {

        Node nFrom;
        nFrom.uuid = node.id;
        nFrom.posX = node.path[0].x;
        nFrom.posY = node.path[0].y;

        for (const auto &nextNodeIds: node.successors) {
            auto nextNode = segments->vertices[nextNodeIds];
            Node nTo;
            nTo.uuid = nextNode.id;
            nTo.posX = nextNode.path[0].x;
            nTo.posY = nextNode.path[0].y;

            graph.add_edge(nFrom, nTo);
        }
    }

    graph.write_to_file("/home/jakub/amr_ws/ros_result.dot");

    ROS_INFO("Graph has been generated successfully");
}
