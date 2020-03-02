//
// Created by jakub on 28.2.2020.
//

#include <amr_graph_generator/PathDxfParser.h>
#include <dxflib/dl_dxf.h>
#include <string>
#include <cmath>


PathDxfParser::PathDxfParser(const string& dxfFilepath, double maxEdgeLenght)
    :   maxEdgeLength(maxEdgeLenght){
    DL_Dxf dxf;

    if(!dxf.in(dxfFilepath, this)) {
        throw std::invalid_argument("File path: " + dxfFilepath + " does not exist!");
    }
}

void PathDxfParser::addLine(const DL_LineData& line) {
    if (!currentBlockName.empty()) {
        if (attributes.getColor() == 1) {
            // short red arrow line
            blocks.at(currentBlockName).arrowLines.push_back(line);
        } else {
            // path line
            blocks.at(currentBlockName).pathLines.push_back(line);
        }
    }
}

void PathDxfParser::addArc(const DL_ArcData& arc) {
    if (!currentBlockName.empty()) {
        blocks.at(currentBlockName).arcs.push_back(arc);
    }
}

void PathDxfParser::addImage(const DL_ImageData& image) {
    images.push_back(image);
}

void PathDxfParser::addBlock(const DL_BlockData& block) {
    auto found = block.name.find("path");

    if (found != std::string::npos) {
        currentBlockName = block.name;
        blocks[currentBlockName] = Block();
    }
}

void PathDxfParser::addInsert(const DL_InsertData &insert) {
    if (insert.name.find("path") == std::string::npos) {
        return;
    }
    auto segments = convertBlockToSegments(blocks.at(insert.name), insert);
    if (!segments.empty()) {
        sampledPathNodes.push_back(segments);
    }
}

std::vector<Node> PathDxfParser::convertBlockToSegments(const Block& block, const DL_InsertData& insert) {

    if (!block.arcs.empty()) {
        // path is defined by arc

    } else {

        // block have to contain just one line
        if (block.pathLines.size() != 1) {
            throw std::runtime_error("Block '" + insert.name + "' contain wrong number of path lines: "
                                     + std::to_string(block.pathLines.size()) + ". Required is 1 line");
        }
        DL_LineData pathLine = block.pathLines.back();

        // detect intersection between arrow lines and path lines
        bool p1Intersection = false, p2Intersection = false;
        for (auto arrowLine: block.arrowLines) {
            if ((pathLine.x1 == arrowLine.x1 && pathLine.y1 == arrowLine.y1)
                || (pathLine.x1 == arrowLine.x2 && pathLine.y1 == arrowLine.y2)) {
                p1Intersection = true;
            }
            if ((pathLine.x2 == arrowLine.x1 && pathLine.y2 == arrowLine.y1)
                || (pathLine.x2 == arrowLine.x2 && pathLine.y2 == arrowLine.y2)) {
                p2Intersection = true;
            }
        }

        if (p1Intersection && p2Intersection) {
            throw std::runtime_error("Detected two arrows in '" + insert.name + "' block.");
        } else if (!p1Intersection && !p2Intersection) {
            throw std::runtime_error("No arrows detected in '" + insert.name + "' block.");
        }

        if (p1Intersection) {
            // start is in x2,y2, end is in x1,y1
            Eigen::Vector2d start{(pathLine.x2 + insert.ipx) * insert.sx, (pathLine.y2 + insert.ipy) * insert.sy};
            Eigen::Vector2d end{(pathLine.x1 + insert.ipx) * insert.sx, (pathLine.y1 + insert.ipy) * insert.sy};
            return sampleLine(start, end);
        } else if (p2Intersection) {
            // start is in x1,y1, end is in x2,y2,
            Eigen::Vector2d end{(pathLine.x2 + insert.ipx) * insert.sx, (pathLine.y2 + insert.ipy) * insert.sy};
            Eigen::Vector2d start{(pathLine.x1 + insert.ipx) * insert.sx, (pathLine.y1 + insert.ipy) * insert.sy};
            return sampleLine(start, end);
        }
    }

    return std::vector<Node>();
}

void PathDxfParser::endBlock() {
    currentBlockName = "";
}

std::vector<Node> PathDxfParser::sampleLine(const Eigen::Vector2d& start, const Eigen::Vector2d& end) {
    std::vector<Node> samplePoints;

    double length = (end - start).norm();
    auto splits = static_cast<unsigned int>(std::ceil(length / maxEdgeLength));

    Eigen::Vector2d increment = (end - start) / splits;
    Eigen::Vector2d current = start;

    for (unsigned int i = 0; i < splits; i++) {
        Node n;
        n.uuid = addNodeCounter;
        n.x = current[0];
        n.y = current[1];
        samplePoints.push_back(n);

        // add successor for the previous node
        if (i > 0) {
            samplePoints.at(i - 1).successors.push_back(addNodeCounter);
        }

        addNodeCounter++;
        current += increment;
    }

    if (!samplePoints.empty()) {
        Node n;
        n.uuid = addNodeCounter;
        n.x = end[0];
        n.y = end[1];
        samplePoints.back().successors.push_back(addNodeCounter);
        samplePoints.push_back(n);
        addNodeCounter++;
    } else {
        Node nEnd;
        nEnd.uuid = addNodeCounter;
        nEnd.x = end[0];
        nEnd.y = end[1];
        samplePoints.push_back(nEnd);
        addNodeCounter++;

        Node nStart;
        nStart.uuid = addNodeCounter;
        nStart.x = start[0];
        nStart.y = start[1];
        nStart.successors.push_back(addNodeCounter -1);
        samplePoints.push_back(nStart);
        addNodeCounter++;
    }

    return samplePoints;
}

std::vector<Line> PathDxfParser::splitCircle(const DL_CircleData& _circle, const float _segLength) {
    DL_ArcData _arc(_circle.cx, _circle.cy, _circle.cz, _circle.radius, 0, 360);
    return splitArc(_arc, _segLength);
}

std::vector<Line> PathDxfParser::splitArc(const DL_ArcData& _arc, const float _segLength) {
    std::vector<Line> segments;

    //Take care DL_ uses Degrees
    float angle1 = _arc.angle1 / 180 * M_PI;
    float angle2 = _arc.angle2 / 180 * M_PI;;

    if (_arc.angle1 >= _arc.angle2)
        angle2 += 2 * M_PI;

    float arcLength = (angle2 - angle1) * _arc.radius;
    uint32_t splits = arcLength / _segLength;

    float angleIncrement_radians = (arcLength / (float)splits) / _arc.radius;
    float current_angle_radians = angle1;
    Eigen::Vector2d current_point(_arc.cx + _arc.radius * cos(current_angle_radians),
                                  _arc.cy + _arc.radius * sin(current_angle_radians));

    for (uint32_t i = 0; i < splits; i++) {
        current_angle_radians += angleIncrement_radians;
        Eigen::Vector2d nextPoint(_arc.cx + _arc.radius * cos(current_angle_radians),
                                  _arc.cy + _arc.radius * sin(current_angle_radians));
        Line l(current_point, nextPoint);
        segments.push_back(l);

        current_point = nextPoint;
    }

    return segments;
}

double PathDxfParser::lineLength(const DL_LineData& line) {
    return sqrt(sqr(line.x1 - line.x2) + sqr(line.y1 - line.y2));
}

int PathDxfParser::getLargestLine(std::vector<DL_LineData>& lines) {
    double largestLength = 0;
    int largestLineIndex = -1;
    for (unsigned int i = 0; i < lines.size(); i++) {
        double len = lineLength(lines[i]);
        if (len > largestLength) {
            largestLength = len;
            largestLineIndex = i;
        }
    }

    return largestLineIndex;
}

bool PathDxfParser::nodesPoseIsEqual(const Node& n1, const Node& n2) {
    return NODES_INTERSECTION_TOLERANCE > std::abs(std::abs(n1.x) - std::abs(n2.x))
        && NODES_INTERSECTION_TOLERANCE > std::abs(std::abs(n1.y) - std::abs(n2.y));
}

std::vector<Node> PathDxfParser::generateGraph() {

    // unify first/end Nodes ids
    for (auto currentSegmentIt = sampledPathNodes.begin(); currentSegmentIt < sampledPathNodes.end(); currentSegmentIt++) {
        for (auto segmentInsideIt = currentSegmentIt + 1; segmentInsideIt < sampledPathNodes.end(); segmentInsideIt++) {
            // unify end Nodes
            if (nodesPoseIsEqual(currentSegmentIt->back(), segmentInsideIt->back())) {
                segmentInsideIt->erase(segmentInsideIt->end());     // remove last node in vector
                segmentInsideIt->back().successors[0] = (currentSegmentIt->back().uuid);    // change successor
            }
            // unify first nodes
            if (nodesPoseIsEqual(currentSegmentIt->front(), segmentInsideIt->front())) {
                segmentInsideIt->front().uuid = currentSegmentIt->front().uuid;
            }
        }
    }

    // chain Nodes segments
    // if a end and first Nodes are the same, remove first node and add successor for the end Node
    for (auto segmentEndIt = sampledPathNodes.begin(); segmentEndIt < sampledPathNodes.end(); segmentEndIt++) {
        for (auto segmentFrontIt = sampledPathNodes.begin(); segmentFrontIt < sampledPathNodes.end(); segmentFrontIt++) {
            if (nodesPoseIsEqual(segmentEndIt->back(), segmentFrontIt->front())) {
                segmentFrontIt->erase(segmentFrontIt->begin());                             // remove useless Node
                segmentEndIt->back().successors.push_back(segmentFrontIt->front().uuid);    //chain segments
            }
        }
    }

    // merge all Nodes from all segments to one vector
    std::vector<Node> nodes;
    for (const std::vector<Node>& sampledPath : sampledPathNodes) {
        nodes.insert(nodes.end(), sampledPath.begin(), sampledPath.end());
    }
    return nodes;
}

