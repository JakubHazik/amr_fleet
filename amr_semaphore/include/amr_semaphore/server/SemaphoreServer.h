//
// Created by jakub on 15.3.2020.
//

#ifndef PROJECT_SEMAPHORESERVER_H
#define PROJECT_SEMAPHORESERVER_H

#include <ros/ros.h>
#include <amr_msgs/LockPoint.h>
#include <amr_msgs/Point.h>

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

class NodesOccupancyContainer {
public:
    NodesOccupancyContainer(unsigned int occupancyLength);

    bool lockNode(const std::string& ownerId, const amr_msgs::Point& node);

    bool isNodeAlreadyLocked(const amr_msgs::Point& node);

    std::map<std::string, std::list<amr_msgs::Point>> getOccupancyData();

private:
    std::map<std::string, std::list<amr_msgs::Point>> data;
    unsigned int bufferMaxLength;


};



class SemaphoreServer {

public:
    SemaphoreServer(ros::NodeHandle& nh);

private:
    ros::ServiceServer lockNodeSrv;

    std::shared_ptr<NodesOccupancyContainer> nodesOccupancy;
//    std::map<unsigned int, std::string> lockedNodes;  // <nodeId, clientId>

    bool lockNodeCb(amr_msgs::LockPoint::Request& req, amr_msgs::LockPoint::Response& res);

};


#endif //PROJECT_SEMAPHORESERVER_H
