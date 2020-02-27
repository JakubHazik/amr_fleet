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

    for (const tuw_multi_robot_msgs::Vertex &node: segments->vertices) {
        Node nFrom(node.id, node.path[0].x, node.path[0].y);
        for (const auto &nextNodeIds: node.successors) {
            auto nextNode = segments->vertices[nextNodeIds];
            Node nTo(nextNode.id, nextNode.path[0].x, nextNode.path[0].y);
            graph.addEdge(nFrom, nTo);
        }
    }

    graph.writeToFile("/home/jakub/amr_ws/ros_result.dot");
    ROS_INFO("Graph has been generated successfully");
}


bool RosWrapper::planPathSrvCallback(amr_planner::PlanPathRequest& req, amr_planner::PlanPathResponse& res) {

    graphSearch = std::make_shared<GraphSearchMultiRobot>(graph, GraphSearchMultiRobot::SearchMethod::A_STAR);
    auto startNode = graphSearch->getNearestNode(req.startPose.position.x, req.startPose.position.y);
    auto endNode = graphSearch->getNearestNode(req.endPose.position.x, req.endPose.position.y);
    auto result = graphSearch->getPath(startNode, endNode);

    std::cout<<"Result path:\n";
    for (auto n: result) {
        std::cout<<n.uuid<<" -> ";
    }
    std::cout<<std::endl;

    res.pathWaypoints = nodes2poses(result);

    return true;
}
