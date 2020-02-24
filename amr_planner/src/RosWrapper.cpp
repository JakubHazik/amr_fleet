//
// Created by jakub on 21.2.2020.
//

#include <amr_planner/RosWrapper.h>

#include <boost/graph/directed_graph.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <amr_planner/GraphSearchMultiRobot.h>


RosWrapper::RosWrapper(ros::NodeHandle& nh) {
    segSubscriber = nh.subscribe("segments", 5, &RosWrapper::newSegmentsCb, this);
    planPathSrv = nh.advertiseService("plan_path", &RosWrapper::planPathSrvCallback, this);
}

void RosWrapper::newSegmentsCb(const tuw_multi_robot_msgs::Graph::ConstPtr& segments) {
    ROS_INFO("New graph segments received");

    graph.clear();

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

            graph.addEdge(nFrom, nTo);
        }
    }

//    Node n;
//    n.uuid = 5;
//    graph.getNeighbors(n);

    graph.writeToFile("/home/jakub/amr_ws/ros_result.dot");

    ROS_INFO("Graph has been generated successfully");
}

std::vector<Node> RosWrapper::getPlan(const Node& start, const Node& end) {

    graphSearch = std::make_shared<GraphSearchMultiRobot>(graph, GraphSearchMultiRobot::SearchMethod::A_STAR);
    auto result = graphSearch->getPath(start, end);
    return std::vector<Node>();
}

bool RosWrapper::planPathSrvCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    Node s;
    Node e;
    s.uuid = 28;
    e.uuid = 27;

    auto plan = getPlan(s, e);

    return true;

}
