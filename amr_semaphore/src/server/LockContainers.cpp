//
// Created by jakub on 17. 4. 2020.
//

#include <amr_semaphore/server/LockContainers.h>

#include <utility>


NodesOccupancyContainer::NodesOccupancyContainer() {

}

void NodesOccupancyContainer::setupClient(const std::string &ownerId, unsigned int occupancyLength) {
    clientOccupancyLength[ownerId] = occupancyLength;
}

std::map<std::string, std::list<Node>> NodesOccupancyContainer::getOccupancyData() {
    return data;
}

bool NodesOccupancyContainer::lockNode(const std::string &ownerId, const Node &node, bool lockVirtually) {
    if (!lockVirtually && !isNodeReallyLocked(node).empty()) {
        return false;
    } else if (ownerId == isNodeVirtuallyLocked(node)) {
        // node is already owned by this owner, but only virtually, next lock is not needed
        return true;
    } else {
        auto targetOwnerIt = data.find(ownerId);
        if (targetOwnerIt == data.end()) {
            // ownerId has not exist already, add it
            std::pair<std::string, std::list<Node>> ownerArray(ownerId, {});
            auto insertResult = data.insert(ownerArray);
            targetOwnerIt = insertResult.first;
        }

        auto& nodesArray = targetOwnerIt->second;
        nodesArray.push_back(node);
    }
}

bool NodesOccupancyContainer::unlockNode(const std::string &ownerId, const Node &node) {
    auto targetOwnerIt = data.find(ownerId);
    if (targetOwnerIt == data.end()) {
        return false; // owner does not exist
    }

    auto& nodesArray = targetOwnerIt->second;
    for (auto lockedNodeIt = nodesArray.begin(); lockedNodeIt != nodesArray.end(); lockedNodeIt++) {
        if (lockedNodeIt->uuid == node.uuid) {
            nodesArray.erase(lockedNodeIt);
            return true;
        }
    }

    return false;   // node is not locked
}

bool NodesOccupancyContainer::isNodeAlreadyLocked(const Node& node) {
    for (const auto& ownerNodes : data) {
        for (const auto& nodesIt : ownerNodes.second) {
            if (nodesIt.uuid == node.uuid) {
                return true;
            }
        }
    }

    return false;
}

bool NodesOccupancyContainer::isNodeAlreadyLockedBy(const std::string& ownerId, const Node &node) {
    auto ownerNodesIt = data.find(ownerId);
    if (ownerNodesIt == data.end()) {
        // owner does not exist
        return false;
    }

    for (const auto& n: ownerNodesIt->second) {
        if (n.uuid == node.uuid) {
            return true;
        }
    }
    return false;
}

void NodesOccupancyContainer::unlockAllNodes(const std::string &ownerId) {
    data[ownerId] = std::list<Node>();
}

std::vector<Node> NodesOccupancyContainer::checkMaxNodesAndRemove(const std::string& ownerId, const Node& referencedNode) {
    std::vector<Node> removedNodes;

    auto targetOwnerIt = data.find(ownerId);
    if (targetOwnerIt == data.end()) {
        // owner does not exist, nothing to check
        return removedNodes;
    }

    auto& nodesArray = targetOwnerIt->second;
    if (nodesArray.size() > clientOccupancyLength[ownerId]) {
        auto refNodeIt = std::find(nodesArray.begin(), nodesArray.end(), referencedNode);

        for (int i = 0; i < clientOccupancyLength[ownerId]; i++) {
            if (refNodeIt == nodesArray.begin()) {
                // max number of nodes has not been achieved
                return removedNodes;
            }
            refNodeIt--;
        }
        ++refNodeIt; // achieve correct iterator

        removedNodes.insert(removedNodes.begin(), nodesArray.begin(), refNodeIt);
        nodesArray.erase(nodesArray.begin(), refNodeIt);
    }

    return removedNodes;
}

std::string NodesOccupancyContainer::isNodeReallyLocked(const Node &node) {
    for (const auto& ownerNodes : data) {
        auto targetNodeIt = std::find(ownerNodes.second.begin(), ownerNodes.second.end(), node);

        if (targetNodeIt == ownerNodes.second.end()) {
            // node is not in this owner array
            continue;
        }

        // check if node is in critical buffer range
        if (std::distance(ownerNodes.second.begin(), targetNodeIt) < clientOccupancyLength[ownerNodes.first]) {
            return ownerNodes.first;
        }
    }
    return {};
}

std::string NodesOccupancyContainer::isNodeVirtuallyLocked(const Node& node) {
    for (const auto& ownerNodes : data) {
        auto targetNodeIt = std::find(ownerNodes.second.begin(), ownerNodes.second.end(), node);

        if (targetNodeIt == ownerNodes.second.end()) {
            // node is not in this owner array
            continue;
        }

        // check if node is in critical buffer range
        if (std::distance(ownerNodes.second.begin(), targetNodeIt) >= clientOccupancyLength[ownerNodes.first]) {
            return ownerNodes.first;
        }
    }
    return {};
}




void AreaBasedLocks::setupArea(std::set<int> areaNodeUuids, unsigned int maxClientsInArea) {
    Area area;
    area.areaNodes = std::move(areaNodeUuids);
    area.maxClientsNum = maxClientsInArea;
    data.push_back(area);
}

bool AreaBasedLocks::lockNode(const std::string &ownerId, const Node &node) {
    for (auto& area: data) {
        auto nodeFoundIt = area.areaNodes.find(node.uuid);
        if (nodeFoundIt != area.areaNodes.end()) {
            // node contain to this area
            auto clientFoundIt = area.clientsLockedNodes.find(ownerId);
            if (clientFoundIt == area.clientsLockedNodes.end()) {
                // client is not in area, yet
                if (area.clientsLockedNodes.size() < area.maxClientsNum) {
                    // area is not full, we can insert client to area
                    area.clientsLockedNodes.insert({ownerId, {node}});
                } else {
                    // area is full, we can not add other clients inside
                    return false;
                }
            } else {
                clientFoundIt->second.push_back(node);
            }
        }
    }

    return true;
}

void AreaBasedLocks::unlockNode(const std::string &ownerId, const Node &node) {
    for (auto& area: data) {
        for (auto lockedNodesIt = area.clientsLockedNodes.begin(); lockedNodesIt != area.clientsLockedNodes.end(); ) {
            if (lockedNodesIt->first == ownerId) {
                auto lockedNodeIt = std::find(lockedNodesIt->second.begin(), lockedNodesIt->second.end(), node);
                if (lockedNodeIt != lockedNodesIt->second.end()) {
                    lockedNodesIt->second.erase(lockedNodeIt);
                }
            }
            // remove owner ID key from map it it does not contain nodes
            if (lockedNodesIt->second.empty()) {
                lockedNodesIt = area.clientsLockedNodes.erase(lockedNodesIt);
            } else {
                lockedNodesIt++;
            }
        }
    }
}

void AreaBasedLocks::unlockNode(const std::string &ownerId, const std::vector<Node> &nodes) {
    for (const auto n: nodes) {
        unlockNode(ownerId, n);
    }
}

void AreaBasedLocks::unlockAllNodes(const std::string& ownerId) {
    for (auto& area: data) {
        auto clientLocksIt = area.clientsLockedNodes.find(ownerId);
        if (clientLocksIt != area.clientsLockedNodes.end()) {
            area.clientsLockedNodes.erase(clientLocksIt);
        }
    }
}

bool AreaBasedLocks::canBeNodeLocked(const std::string& ownerId, const Node& node) {
    for (const auto& area: data) {
        auto nodeFoundIt = area.areaNodes.find(node.uuid);
        if (nodeFoundIt != area.areaNodes.end()) {
            // node contain to this area

            auto ownerFoundIt = area.clientsLockedNodes.find(ownerId);
            if (ownerFoundIt == area.clientsLockedNodes.end() && area.clientsLockedNodes.size() >= area.maxClientsNum) {
                // owner is not presented in thi area
                // and number of clients in area has been reached
                return false;
            }
        }
    }

    return true;
}
