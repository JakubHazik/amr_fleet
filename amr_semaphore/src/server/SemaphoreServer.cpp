//
// Created by jakub on 15.3.2020.
//

#include <amr_semaphore/server/SemaphoreServer.h>
#include <algorithm>

NodesOccupancyContainer::NodesOccupancyContainer(const unsigned int occupancyLength) {
    bufferMaxLength = occupancyLength;
}

std::map<std::string, std::list<amr_msgs::Point>> NodesOccupancyContainer::getOccupancyData() {
    return std::map<std::string, std::list<amr_msgs::Point>>();
}

bool NodesOccupancyContainer::lockNode(const std::string &ownerId, const amr_msgs::Point& node) {
    if (isNodeAlreadyLocked(node)) {
        return false;
    } else {
        if (data.find(ownerId) == data.end()) {
            // ownerId has not exist already, add it
            std::pair<std::string, std::list<amr_msgs::Point>> ownerArray(ownerId, {});
            data.insert(ownerArray);
        }

        auto& nodesArray = data[ownerId];
        nodesArray.push_back(node);
        if (nodesArray.size() > bufferMaxLength) {
            nodesArray.pop_front();
        }
    }

    return false;
}

bool NodesOccupancyContainer::isNodeAlreadyLocked(const amr_msgs::Point& node) {
    for (const auto& ownerNodes : data) {
        for (const auto& nodesIt : ownerNodes.second) {
            if (nodesIt.uuid == node.uuid) {
                return true;
            }
        }
    }

    return false;
}


SemaphoreServer::SemaphoreServer(ros::NodeHandle& nh) {

    //todo config
    nodesOccupancy = std::make_shared<NodesOccupancyContainer>(2);

    lockNodeSrv = nh.advertiseService("lock_node", &SemaphoreServer::lockNodeCb, this);
}

bool SemaphoreServer::lockNodeCb(amr_msgs::LockPoint::Request& req, amr_msgs::LockPoint::Response& res) {
//    if (req.lock) {
        // lock node

        if (nodesOccupancy->lockNode(req.clientId, req.point)) {
            // successfully locked
            res.success = true;

        } else {
            // not locked

            res.message = "Node is already locked";
            res.success = false;

        }

    return true;


//        auto lockIt = lockedNodes.find(req.node.point.uuid);
//        if (lockIt == lockedNodes.end()) {
//            // node is not locked, we can lock it
//            std::pair<unsigned int, std::string> nodeLock(req.node.point.uuid, req.clientId);
//            lockedNodes.insert(nodeLock);
//            res.success = true;
//            return true;
//        } else {
//            // node is already locked
//            res.message = "Node is already locked by " + lockIt->second;
//            res.success = false;
//            return true;
//        }


//    } else {
//        // unlock node
//        auto lockIt = lockedNodes.find(req.node.point.uuid);
//        if (lockIt == lockedNodes.end()) {
//            // node is missing in locked array
//            res.message = "Can not unlock the node: " + std::to_string(req.node.point.uuid)
//                          + ", because node has not been locked, yet [Client: " + req.clientId + "]";
//            res.success = false;
//            return true;
//        } else {
//            // we can unlock node normally
//            lockedNodes.erase(lockIt);
//            res.success = true;
//            return true;
//        }
//    }

    return true;
}

