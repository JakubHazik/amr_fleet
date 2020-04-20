//
// Created by jakub on 21.2.2020.
//

#include <amr_planner/RosWrapper.h>

#include <boost/graph/astar_search.hpp>
#include <boost/graph/random.hpp>
#include <amr_planner/GraphSearchMultiRobot.h>


RosWrapper::RosWrapper(ros::NodeHandle& nh) {
    // todo configure
    segSubscriber = nh.subscribe("/graph_generator/graph", 5, &RosWrapper::newGraphCb, this);
    planPathByPointsSrv = nh.advertiseService("plan_path", &RosWrapper::planPathCallback, this);
    planPathByNodesSrv = nh.advertiseService("plan_path_by_nodes", &RosWrapper::planPathNodesCallback, this);
    setNodePropertiesSrv = nh.advertiseService("set_node_properties", &RosWrapper::setNodePropertiesCallback, this);
}

void RosWrapper::newGraphCb(const amr_msgs::Graph::ConstPtr& graphMsg) {
    ROS_INFO("New graph segments received");

    graph.readNewGraph(*graphMsg);

//    graph.writeToFile("/home/jakub/amr_ws/ros_result.dot");
    ROS_INFO("Graph has been generated successfully");
}


bool RosWrapper::planPathCallback(amr_msgs::PlanPath::Request& req, amr_msgs::PlanPath::Response& res) {

    graphSearch = std::make_shared<GraphSearchMultiRobot>(graph, GraphSearchMultiRobot::SearchMethod::A_STAR);

    Node startNode, endNode;
    if (req.startPoint.uuid == 0) {
        // start point defined by [x,y]
        startNode = graphSearch->getNearestNode(req.startPoint.pose.x, req.startPoint.pose.y);
    } else {
        // start point defined by UUID
        startNode = graphSearch->getNode(req.startPoint.uuid);
    }

    if (req.endPoint.uuid == 0) {
        // end point defined by [x,y]
        endNode = graphSearch->getNearestNode(req.endPoint.pose.x, req.endPoint.pose.y);
    } else {
        // end point defined by UUID
        endNode = graphSearch->getNode(req.endPoint.uuid);
    }

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
