//
// Created by jakub on 27.2.2020.
//

#include <amr_graph_representation/DataTypesAndConversions.h>


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

Node::Node(int uuid, double x, double y, bool bidirectional) {
    this->uuid = uuid;
    this->posX = x;
    this->posY = y;
    this->isBidirectional = bidirectional;
}

geometry_msgs::Pose node2pose(const Node& n) {
    geometry_msgs::Pose p;
    p.position.x = n.posX;
    p.position.y = n.posY;
    return p;
}

geometry_msgs::Pose2D node2pose2D(const Node& n) {
    geometry_msgs::Pose2D p;
    p.x = n.posX;
    p.y = n.posY;
    return p;
}

amr_msgs::Point node2point(const Node& n) {
    amr_msgs::Point point;
    point.uuid = n.uuid;
    point.pose = node2pose2D(n);
    return point;
}

std::vector<amr_msgs::Point> nodes2msgPoints(const std::vector<Node>& nodes) {
    std::vector<amr_msgs::Point> points;
    for (const auto& n: nodes) {
        points.push_back(node2point(n));
    }
    return points;
}


