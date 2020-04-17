//
// Created by jakub on 26.2.2020.
//

#ifndef PROJECT_DATATYPESANDCONVERSIONS_H
#define PROJECT_DATATYPESANDCONVERSIONS_H

#include <cmath>
#include <ostream>
#include <vector>
#include <amr_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#define sqr(x) ((x)*(x))

class Node {
public:
    int uuid;
    double posX;
    double posY;
    bool isReachable = true;
    bool isBidirectional = false;

    Node() = default;

    Node(int uuid, double x, double y, bool bidirectional);

    bool isValid();

    static double distance(const Node& n1, const Node& n2);

    static double distance(const Node& n1, double nx, double ny);

    bool operator==(const Node& n) const;

    bool operator<(const Node& n) const;

    friend std::ostream& operator<<(std::ostream& os, const Node& n);
};

struct Edge {
    double distance;
};

struct Neighbor {
    Node node;
    double weight;
};


geometry_msgs::Pose node2pose(const Node& n);
geometry_msgs::Pose2D node2pose2D(const Node& n);
amr_msgs::Point node2point(const Node& n);
std::vector<amr_msgs::Point> nodes2msgPoints(const std::vector<Node>& nodes);


#endif //PROJECT_DATATYPESANDCONVERSIONS_H
