//
// Created by jakub on 26.2.2020.
//

#ifndef PROJECT_DATATYPESANDCONVERSIONS_H
#define PROJECT_DATATYPESANDCONVERSIONS_H

#include <cmath>
#include <ostream>
#include <vector>
#include <geometry_msgs/Pose.h>

#define sqr(x) ((x)*(x))

class Node {
public:
    int uuid;
    double posX;
    double posY;

    Node() = default;

    Node(int uuid, double x, double y);

    static double distance(const Node& n1, const Node& n2);

    static double distance(const Node& n1, double nx, double ny);

    bool operator==(const Node& n) const;

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
std::vector<geometry_msgs::Pose> nodes2poses(const std::vector<Node>& nodes);


#endif //PROJECT_DATATYPESANDCONVERSIONS_H
