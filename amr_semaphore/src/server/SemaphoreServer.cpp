//
// Created by jakub on 15.3.2020.
//

#include <amr_semaphore/server/SemaphoreServer.h>
#include <algorithm>
#include <future>

NodesOccupancyContainer::NodesOccupancyContainer(const unsigned int occupancyLength) {
    bufferMaxLength = occupancyLength;
}

std::map<std::string, std::list<amr_msgs::Point>> NodesOccupancyContainer::getOccupancyData() {
    return data;
}

bool NodesOccupancyContainer::lockNode(const std::string &ownerId, const amr_msgs::Point& node) {
    if (isNodeAlreadyLocked(node)) {
        return false;
    } else {
        auto targetOwnerIt = data.find(ownerId);
        if (targetOwnerIt == data.end()) {
            // ownerId has not exist already, add it
            std::pair<std::string, std::list<amr_msgs::Point>> ownerArray(ownerId, {});
            auto insertResult = data.insert(ownerArray);
            targetOwnerIt = insertResult.first;
        }

        auto& nodesArray = targetOwnerIt->second;
        nodesArray.push_back(node);
        if (nodesArray.size() > bufferMaxLength) {
            nodesArray.pop_front();
        }
        return true;
    }
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

void NodesOccupancyContainer::unlockAllNodes(const std::string &ownerId) {
    data[ownerId] = std::list<amr_msgs::Point>();
}


SemaphoreServer::SemaphoreServer(ros::NodeHandle& nh) {

    //todo config
    nodesOccupancy = std::make_shared<NodesOccupancyContainer>(3);

    lockNodeSrv = nh.advertiseService("lock_node", &SemaphoreServer::lockNodeCb, this);

    // prepare visualizer
    visual_tools.reset(new rvt::RvizVisualTools("map", "/amr_occupied_nodes"));
    visual_tools->loadMarkerPub(false, true);  // create publisher before waiting
    // clear messages
    visual_tools->deleteAllMarkers();
    visual_tools->enableBatchPublishing();
}

bool SemaphoreServer::lockNodeCb(amr_msgs::LockPoint::Request& req, amr_msgs::LockPoint::Response& res) {
    if (req.unlockAll) {
        nodesOccupancy->unlockAllNodes(req.clientId);
        res.success = true;
        return true;
    }

    if (nodesOccupancy->lockNode(req.clientId, req.point)) {
        // successfully locked
        res.success = true;

        // update visualization
        std::async(std::launch::async, &SemaphoreServer::visualizeNodesOccupancy, this);
    } else {
        // not locked
        res.message = "Node is already locked";
        res.success = false;
    }

    return true;
}

rvt::colors getColorHash(const std::string& ownerId) {
    return static_cast<rvt::colors>(std::hash<std::string>{}(ownerId) % 18);
}

void SemaphoreServer::visualizeNodesOccupancy() {
    visual_tools->deleteAllMarkers();

    std_msgs::ColorRGBA color = visual_tools->getColorScale(0);
    geometry_msgs::Vector3 scale = visual_tools->getScale(rvt::LARGE);

    for (const auto& ownerNodes: nodesOccupancy->getOccupancyData()) {
        for (const auto& nodes: ownerNodes.second) {
            geometry_msgs::Point point;
            point.x = nodes.pose.x;
            point.y = nodes.pose.y;

            visual_tools->publishSphere(point, getColorHash(ownerNodes.first), rvt::scales::XLARGE);
            geometry_msgs::Pose pose;
            pose.position = point;
            pose.position.x += 0.1;
            visual_tools->publishText(pose, ownerNodes.first, rvt::WHITE, rvt::XXLARGE, false);
        }
    }

    visual_tools->trigger();
}

