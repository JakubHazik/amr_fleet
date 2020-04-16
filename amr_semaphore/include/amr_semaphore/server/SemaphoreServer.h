//
// Created by jakub on 15.3.2020.
//

#ifndef PROJECT_SEMAPHORESERVER_H
#define PROJECT_SEMAPHORESERVER_H

#include <ros/ros.h>
#include <amr_msgs/LockPoint.h>
#include <amr_msgs/Point.h>
#include <amr_msgs/Graph.h>
#include <amr_msgs/ClientPath.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <amr_graph_representation/Graph.h>
#include <amr_graph_representation/DataTypesAndConversions.h>

#include <queue>
#include <deque>
#include <set>
#include <list>
#include <memory>


namespace rvt = rviz_visual_tools;


class NodesOccupancyContainer {
public:
    NodesOccupancyContainer(unsigned int occupancyLength);

    bool lockNode(const std::string &ownerId, const Node &node, bool lockVirtually);

    bool unlockNode(const std::string& ownerId, const Node& node);

    void unlockAllNodes(const std::string& ownerId);

    std::map<std::string, std::list<Node>> getOccupancyData();

    bool isNodeAlreadyLocked(const Node& node);

    bool isNodeAlreadyLockedBy(const std::string& ownerId, const Node& node);

    void checkMaxNodesAndRemove(const std::string& ownerId, const Node& referencedNode);

    // return owner ID if node is locked in critical buffer of robot length
    std::string isNodeReallyLocked(const Node& node);

    // return owner ID if node is locked and is not in critical buffer of robot length (bidirectioanl nodes)
    std::string isNodeVirtuallyLocked(const Node& node);


private:
    std::map<std::string, std::list<Node>> data;
    unsigned int bufferMaxLength;
};



class SemaphoreServer {

public:
    SemaphoreServer(ros::NodeHandle& nh);

private:
    Graph graph;

    ros::Subscriber graphSub;
    ros::Subscriber clientsPathsSub;
    ros::ServiceServer lockNodeSrv;
    rvt::RvizVisualToolsPtr visual_tools;

    std::shared_ptr<NodesOccupancyContainer> nodesOccupancy;
    std::map<std::string, std::vector<Node>> clientsPaths;

    bool lockNodeCb(amr_msgs::LockPoint::Request& req, amr_msgs::LockPoint::Response& res);

    void graphCb(const amr_msgs::GraphPtr& msg);

    void clientPathsCb(const amr_msgs::ClientPathConstPtr& msg);

    void visualizeNodesOccupancy();

    void lockAllBidirectionalNeighbours(const std::string& clientId, const Node& node);

    Node getClientNextWaypoint(const std::string& clientId, const Node& node);

};


#endif //PROJECT_SEMAPHORESERVER_H
