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
#include <amr_msgs/SetupSemaphore.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <amr_graph_representation/Graph.h>
#include <amr_graph_representation/DataTypesAndConversions.h>
#include <amr_semaphore/server/LockContainers.h>

#include <queue>
#include <deque>
#include <set>
#include <list>
#include <memory>


namespace rvt = rviz_visual_tools;

class SemaphoreServer {

public:
    SemaphoreServer();

private:
    Graph graph;

    ros::Subscriber graphSub;
    ros::Subscriber clientsPathsSub;
    ros::ServiceServer lockNodeSrv;
    ros::ServiceServer setupSemaphoreSrv;
    rvt::RvizVisualToolsPtr visual_tools;

    std::shared_ptr<NodesOccupancyContainer> nodesOccupancyLocks;
    std::shared_ptr<AreaBasedLocks> areaLocks;
    std::map<std::string, std::vector<Node>> clientsPaths;
    bool lockingOff;

    bool lockNodeCb(amr_msgs::LockPoint::Request& req, amr_msgs::LockPoint::Response& res);

    void graphCb(const amr_msgs::GraphPtr& msg);

    bool setupSemaphoreCb(amr_msgs::SetupSemaphore::Request& req, amr_msgs::SetupSemaphore::Response& res);

    void clientPathsCb(const amr_msgs::ClientPathConstPtr& msg);

    void visualizeNodesOccupancy();

    void lockAllBidirectionalNeighbours(const std::string& clientId, const Node& node);

    Node getClientNextWaypoint(const std::string& clientId, const Node& node);

};


#endif //PROJECT_SEMAPHORESERVER_H
