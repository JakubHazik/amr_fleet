//
// Created by jakub on 15.3.2020.
//

#ifndef PROJECT_SEMAPHORESERVER_H
#define PROJECT_SEMAPHORESERVER_H

#include <ros/ros.h>
#include <amr_msgs/LockPoint.h>
#include <amr_msgs/Point.h>
#include <rviz_visual_tools/rviz_visual_tools.h>


#include <queue>
#include <deque>
#include <set>
#include <list>
#include <memory>


//template <typename T, unsigned int MaxLen, typename Container=std::deque<T>>
//class FixedQueue : public std::queue<T, Container> {
//public:
//    void push(const T& value) {
//        if (this->size() == MaxLen) {
//            this->c.pop_front();
//        }
//        std::queue<T, Container>::push(value);
//    }
//};

namespace rvt = rviz_visual_tools;


class NodesOccupancyContainer {
public:
    NodesOccupancyContainer(unsigned int occupancyLength);

    bool lockNode(const std::string& ownerId, const amr_msgs::Point& node);

    bool unlockNode(const std::string& ownerId, const amr_msgs::Point& node);

    void unlockAllNodes(const std::string& ownerId);

    std::map<std::string, std::list<amr_msgs::Point>> getOccupancyData();

private:
    std::map<std::string, std::list<amr_msgs::Point>> data;
    unsigned int bufferMaxLength;

    bool isNodeAlreadyLocked(const amr_msgs::Point& node);
};



class SemaphoreServer {

public:
    SemaphoreServer(ros::NodeHandle& nh);

private:
    ros::ServiceServer lockNodeSrv;
    rvt::RvizVisualToolsPtr visual_tools;

    std::shared_ptr<NodesOccupancyContainer> nodesOccupancy;
//    std::map<unsigned int, std::string> lockedNodes;  // <nodeId, clientId>

    bool lockNodeCb(amr_msgs::LockPoint::Request& req, amr_msgs::LockPoint::Response& res);

    void visualizeNodesOccupancy();

};


#endif //PROJECT_SEMAPHORESERVER_H
