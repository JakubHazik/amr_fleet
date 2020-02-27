//
// Created by jakub on 27.2.2020.
//

#include <amr_planner/DataTypesAndConversions.h>


double Node::distance(const Node& n1, const Node& n2) {
    return std::sqrt(sqr(n1.posX - n2.posX) + sqr(n1.posY - n2.posY));
}

double Node::distance(const Node& n1, double nx, double ny) {
    return std::sqrt(sqr(n1.posX - nx) + sqr(n1.posY - ny));
}

bool Node::operator==(const Node& n) const {
    return this->uuid == n.uuid;
}


std::ostream& operator<<(std::ostream& os, const Node& n) {
    return os << n.uuid;
}

Node::Node(int uuid, double x, double y) {
    this->uuid = uuid;
    this->posX = x;
    this->posY = y;
}

geometry_msgs::Pose node2pose(const Node& n) {
    geometry_msgs::Pose p;
    p.position.x = n.posX;
    p.position.y = n.posY;
    return p;
}

std::vector<geometry_msgs::Pose> nodes2poses(const std::vector<Node>& nodes) {
    std::vector<geometry_msgs::Pose> poses;
    for (const auto& n: nodes) {
        poses.push_back(node2pose(n));
    }
    return poses;
}


