//
// Created by jakub on 1.3.2020.
//

#ifndef PROJECT_DATATYPES_H
#define PROJECT_DATATYPES_H

#include <vector>
#include <Eigen/Eigen>

struct Line {
    Eigen::Vector2d start;
    Eigen::Vector2d end;
    Line(Eigen::Vector2d _start, Eigen::Vector2d _end) : start(std::move(_start)), end(std::move(_end)) {}
    Line(double xStart, double yStart, double xEnd, double yEnd) {
        start << xStart, yStart;
        end << xEnd, yEnd;
    }
    Line() {}
};

struct Node {
    unsigned int uuid;
    double x;
    double y;
    std::vector<unsigned int> successors;
};


#endif //PROJECT_DATATYPES_H
