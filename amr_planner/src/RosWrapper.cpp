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
    // todo configure
    segSubscriber = nh.subscribe("/graph_generator/graph", 5, &RosWrapper::newGraphCb, this);
    planPathByPointsSrv = nh.advertiseService("plan_path_by_points", &RosWrapper::planPathPointsCallback, this);
    planPathByNodesSrv = nh.advertiseService("plan_path_by_nodes", &RosWrapper::planPathNodesCallback, this);
    setNodePropertiesSrv = nh.advertiseService("set_node_properties", &RosWrapper::setNodePropertiesCallback, this);
}

void RosWrapper::newGraphCb(const amr_msgs::Graph::ConstPtr& graphMsg) {
    ROS_INFO("New graph segments received");

    graph.clear();

    for (const amr_msgs::Node &node: graphMsg->nodes) {
        Node nFrom(node.point.uuid, node.point.pose.x, node.point.pose.y);
        for (const auto &successorNode: node.successors) {
            auto nodesIt = std::find_if(graphMsg->nodes.begin(), graphMsg->nodes.end(),
                    [&successorNode](const amr_msgs::Node& obj) {return obj.point.uuid == successorNode;});
            auto index = std::distance(graphMsg->nodes.begin(), nodesIt);
            auto nextNode = graphMsg->nodes[index];
            Node nTo(nextNode.point.uuid, nextNode.point.pose.x, nextNode.point.pose.y);
            graph.addEdge(nFrom, nTo);
        }
    }

    graph.writeToFile("/home/jakub/amr_ws/ros_result.dot");
    ROS_INFO("Graph has been generated successfully");
}


bool RosWrapper::planPathPointsCallback(amr_msgs::PlanPathPoints::Request& req, amr_msgs::PlanPathPoints::Response& res) {

    graphSearch = std::make_shared<GraphSearchMultiRobot>(graph, GraphSearchMultiRobot::SearchMethod::A_STAR);
    auto startNode = graphSearch->getNearestNode(req.startPose.position.x, req.startPose.position.y);
    auto endNode = graphSearch->getNearestNode(req.endPose.position.x, req.endPose.position.y);
    auto result = graphSearch->getPath(startNode, endNode);

    std::cout<<"Result path:\n";
    for (auto n: result) {
        std::cout<<n.uuid<<" -> ";
    }
    std::cout<<std::endl;

    res.pathWaypoints = nodes2msgPoints(result);

    return true;
}

bool RosWrapper::planPathNodesCallback(amr_msgs::PlanPathNodes::Request& req, amr_msgs::PlanPathNodes::Response& res) {
    graphSearch = std::make_shared<GraphSearchMultiRobot>(graph, GraphSearchMultiRobot::SearchMethod::A_STAR);
    auto startNode = graphSearch->getNode(req.startUuid);
    auto endNode = graphSearch->getNode(req.endUuid);
    auto result = graphSearch->getPath(startNode, endNode);

    std::cout<<"Result path:\n";
    for (auto n: result) {
        std::cout<<n.uuid<<" -> ";
    }
    std::cout<<std::endl;

    res.pathWaypoints = nodes2msgPoints(result);

    return true;
}

bool RosWrapper::setNodePropertiesCallback(amr_msgs::SetNodeProperties::Request& req,
                                           amr_msgs::SetNodeProperties::Response& res) {

    try {
        graph.setNodeReachability(req.point.uuid, req.reachability);
    } catch (const std::out_of_range& ex) {
        res.success = false;
        res.message = ex.what();
        return true;
    }

    res.success = true;
    return true;
}
