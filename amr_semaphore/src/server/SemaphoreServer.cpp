//
// Created by jakub on 15.3.2020.
//

#include <amr_semaphore/server/SemaphoreServer.h>
#include <algorithm>
#include <future>

SemaphoreServer::SemaphoreServer() {
    ros::NodeHandle nh("~");

    // setup area locks
    areaLocks = std::make_shared<AreaBasedLocks>();
    XmlRpc::XmlRpcValue areasList;
    nh.getParam("limitedAreas", areasList);
    for (int i = 0; i < areasList.size(); ++i) {
        XmlRpc::XmlRpcValue area = areasList[i]["area"];
        XmlRpc::XmlRpcValue nodes = area["nodes"];
        int maxNumClients = area["maxNumClients"];

        std::set<int> areaNodes;
        for (int in = 0; in < nodes.size(); ++in) {
            areaNodes.insert(nodes[in]);
        }
        areaLocks->setupArea(areaNodes, maxNumClients);
    }

    // setup occupancy of graph vertexes
//    std::map<std::string, int> occupancyParams;
//    nh.getParam("occupancyLengths", occupancyParams);
    nodesOccupancyLocks = std::make_shared<NodesOccupancyContainer>();
//    for (const auto& occupancy: occupancyParams) {
//        nodesOccupancyLocks->setupClient(occupancy.first, occupancy.second);
//    }

    nh.getParam("lockingOff", lockingOff);

    graphSub = nh.subscribe("/graph_generator/graph", 5, &SemaphoreServer::graphCb, this);
    clientsPathsSub = nh.subscribe("/task_manager_server/client_paths", 10, &SemaphoreServer::clientPathsCb, this);
    lockNodeSrv = nh.advertiseService("lock_node", &SemaphoreServer::lockNodeCb, this);
    setupSemaphoreSrv = nh.advertiseService("setup_semaphore", &SemaphoreServer::setupSemaphoreCb, this);

    // prepare visualizer
    visual_tools.reset(new rvt::RvizVisualTools("map", "/amr_occupied_nodes"));
    visual_tools->loadMarkerPub(false, true);  // create publisher before waiting
    // clear messages
    visual_tools->deleteAllMarkers();
    visual_tools->enableBatchPublishing();
    visual_tools->trigger();
}

bool SemaphoreServer::lockNodeCb(amr_msgs::LockPoint::Request& req, amr_msgs::LockPoint::Response& res) {
    if (lockingOff) {
        res.success = true;
        return true;
    }

    if (req.unlockAll) {
        nodesOccupancyLocks->unlockAllNodes(req.clientId);
        areaLocks->unlockAllNodes(req.clientId);
        res.success = true;
        return true;
    }

    auto node = graph.getNode(req.point.uuid);

    // unlock node
    if (!req.lock) {
        if (nodesOccupancyLocks->unlockNode(req.clientId, node)) {
            areaLocks->unlockNode(req.clientId, node);
            // successfully unlocked
            res.success = true;
        } else {
            // node has not been unlocked
            res.message = "Target node has not been locked yet or client ID is wrong.";
            res.success = true;
        }

        std::async(std::launch::async, &SemaphoreServer::visualizeNodesOccupancy, this);
        return true;
    }

    // area locking
    if (!areaLocks->canBeNodeLocked(req.clientId, node)) {
        res.message = "Node can not be locked, area is full";
        res.success = false;
        return true;
    }

    std::string realNodeOwner = nodesOccupancyLocks->isNodeReallyLocked(node);
    if (realNodeOwner.empty()) {
        // node is not really locked
        std::string bidNodeOwner = nodesOccupancyLocks->isNodeVirtuallyLocked(node);
        if (bidNodeOwner.empty()) {
            // node is not not virtually locked and also not really locked
            // we can lock the node
            nodesOccupancyLocks->lockNode(req.clientId, node);
            areaLocks->lockNode(req.clientId, node);
//            auto removedNodes = nodesOccupancyLocks->checkMaxNodesAndRemove(req.clientId, node);
//            areaLocks->unlockNode(req.clientId, removedNodes);

            auto clientNextNode = getClientNextWaypoint(req.clientId, node);
            if (node.isBidirectional && clientNextNode.isValid() && clientNextNode.isBidirectional) {
                lockAllBidirectionalNeighbours(req.clientId, node);
            }
            res.message = "Node successfully locked";
            res.success = true;
        } else {
            // node is virtually locked
            if (bidNodeOwner == req.clientId) {
                // owner of virtual node is requested client, we can lock it
                nodesOccupancyLocks->lockNode(req.clientId, node);
                areaLocks->lockNode(req.clientId, node);
//                auto removedNodes = nodesOccupancyLocks->checkMaxNodesAndRemove(req.clientId, node);
//                areaLocks->unlockNode(req.clientId, removedNodes);
                res.message = "Node successfully locked";
                res.success = true;
            } else {
                // this virtual node is owned by other client
                // check if client path continue wit bidirectional paths
                auto clientNextNode = getClientNextWaypoint(req.clientId, node);
                if (clientNextNode.isValid() && clientNextNode.isBidirectional) {
                    res.message = "Node is already locked";
                    res.success = false;
                } else {
                    nodesOccupancyLocks->lockNode(req.clientId, node);
                    areaLocks->lockNode(req.clientId, node);
//                    auto removedNodes = nodesOccupancyLocks->checkMaxNodesAndRemove(req.clientId, node);
//                    areaLocks->unlockNode(req.clientId, removedNodes);
                    res.message = "Node successfully locked";
                    res.success = true;
                }
            }
        }
    } else {
        // node is really locked
        if (realNodeOwner == req.clientId) {
            // by requested client
            res.message = "Node is already locked by you";
            res.success = true;
        } else {
            // by other owner
            res.message = "Node is already locked";
            res.success = false;
        }
    }

    // update visualization
    std::async(std::launch::async, &SemaphoreServer::visualizeNodesOccupancy, this);

    return true;
}

rvt::colors getColorHash(const std::string& ownerId) {
//    return static_cast<rvt::colors>(std::hash<std::string>{}(ownerId) % 15);

    int robotId = std::stoi(std::string(ownerId.substr(1, std::string::npos)));
    return static_cast<rvt::colors>(robotId);
}

void SemaphoreServer::visualizeNodesOccupancy() {
    visual_tools->deleteAllMarkers();

//    srand( time(NULL) );

    for (const auto& ownerNodes: nodesOccupancyLocks->getOccupancyData()) {
        for (const auto& nodes: ownerNodes.second) {
            geometry_msgs::Point point;
            point.x = nodes.posX;
            point.y = nodes.posY;

            if (nodes.isBidirectional) {
                double randNum = -1 + (std::rand() % ( 1 - (-1) + 1 ));
                point.x += randNum/100;
            }

            visual_tools->publishSphere(point, getColorHash(ownerNodes.first), rvt::scales::XLARGE);
            geometry_msgs::Pose pose;
            pose.position = point;
            pose.position.x += 0.1;
            visual_tools->publishText(pose, ownerNodes.first, rvt::WHITE, rvt::XXLARGE, false);
        }
    }

    visual_tools->trigger();
}

void SemaphoreServer::graphCb(const amr_msgs::GraphPtr &msg) {
    graph.readNewGraph(*msg);
}

void SemaphoreServer::clientPathsCb(const amr_msgs::ClientPathConstPtr &msg) {
    clientsPaths[msg->clientId] = {};
    for (const auto& wp: msg->waypoints) {
        auto node = graph.getNode(wp.uuid);
        clientsPaths[msg->clientId].push_back(node);
    }
}

void SemaphoreServer::lockAllBidirectionalNeighbours(const std::string& clientId, const Node& node) {
    auto neighbors = graph.getNeighbors(node).first;
    std::list<Node> bidNodes(neighbors.begin(), neighbors.end());
    while (!bidNodes.empty()) {
        // todo check isNodeAlreadyLocked only for target owner, it would be better
        if (!bidNodes.front().isBidirectional || nodesOccupancyLocks->isNodeAlreadyLocked(bidNodes.front())) {
            bidNodes.pop_front();
            continue;
        }

        // this is bidirectional node
        nodesOccupancyLocks->lockNode(clientId, bidNodes.front(), true);
        areaLocks->lockNode(clientId, bidNodes.front());
        neighbors = graph.getNeighbors(bidNodes.front()).first;
        bidNodes.pop_front();
        bidNodes.insert(bidNodes.end(), neighbors.begin(), neighbors.end());
    }
}

Node SemaphoreServer::getClientNextWaypoint(const std::string& clientId, const Node& node) {
    auto clientIt = clientsPaths.find(clientId);
    if (clientIt == clientsPaths.end()) {
        // client waypoints has not been received
        return {};
    }
    auto waypointNodes = clientIt->second;
    auto currentNodeIt = std::find(waypointNodes.begin(), waypointNodes.end(), node);

    if (currentNodeIt == waypointNodes.end()) {
        // node is not between waypoints
        return {};
    } else {
        currentNodeIt++;
        if (currentNodeIt == waypointNodes.end()) {
            // this is last node, it have not successors
            return {};
        }

        return *(currentNodeIt);  // return next node
    }
}

bool SemaphoreServer::setupSemaphoreCb(amr_msgs::SetupSemaphore::Request& req, amr_msgs::SetupSemaphore::Response& res) {
    nodesOccupancyLocks->setupClient(req.clientId, req.numReallyLockedNodes);
    res.success = true;
    return true;
}