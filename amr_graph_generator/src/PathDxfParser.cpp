//
// Created by jakub on 28.2.2020.
//

#include <amr_graph_generator/PathDxfParser.h>
#include <dxflib/dl_dxf.h>
#include <string>
#include <cmath>
#include <algorithm>


PathDxfParser::PathDxfParser(const std::string& dxfFilepath, double maxEdgeLenght)
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
        blocks.at(currentBlockName).pathArcs.push_back(arc);
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

    // block have to contain just one path line
    if (block.pathArcs.size() != 1 && block.pathLines.size() != 1) {
        throw std::runtime_error("Block '" + insert.name + "' contain wrong number of path lines, "
              + "number of lines=" + std::to_string(block.pathLines.size())
              + ", number of arcs=" + std::to_string(block.pathArcs.size()) + ". Required is just 1 path line");
    }

    // block have to contain 1 and more arrow lines
    if (block.arrowLines.empty()) {
        throw std::runtime_error("Block '" + insert.name + "' contain no arrow lines, number of arrow lines: "
                                 + std::to_string(block.arrowLines.size()) + ". Required is 1 and more");
    }

    if (!block.pathArcs.empty()) {
        // path is defined by arc

        DL_ArcData pathArc = block.pathArcs.back();
        auto direction = detectPathDirection(
                {pathArc.cx + pathArc.radius * cos(DEG2RAD(pathArc.angle1)), pathArc.cy + pathArc.radius * sin(DEG2RAD(pathArc.angle1))},
                {pathArc.cx + pathArc.radius * cos(DEG2RAD(pathArc.angle2)), pathArc.cy + pathArc.radius * sin(DEG2RAD(pathArc.angle2))},
                block.arrowLines);
        assertPathDirection(direction, insert.name);

        // move center of arc about insert point
        pathArc.cx += insert.ipx;
        pathArc.cy += insert.ipy;

        if (direction.first) {
            // start is in angle2, end is in angle1
            // switch angles
            double tmpAngle = pathArc.angle1;
            pathArc.angle1 = pathArc.angle2;
            pathArc.angle2 = tmpAngle;
            return sampleArc(pathArc);
        } else if (direction.second) {
            // start is in angle1, end is in angle2
            return sampleArc(pathArc);
        }

    } else if (!block.pathLines.empty()){
        // path is defined by Line
        DL_LineData pathLine = block.pathLines.back();

        auto direction = detectPathDirection(
                {pathLine.x1, pathLine.y1},
                {pathLine.x2, pathLine.y2},
                block.arrowLines);
        assertPathDirection(direction, insert.name);

        Eigen::Vector2d start{pathLine.x2 + insert.ipx, pathLine.y2 + insert.ipy};
        Eigen::Vector2d end{pathLine.x1 + insert.ipx, pathLine.y1 + insert.ipy};

        if (direction.first && direction.second) {
            // bidirectional
            return sampleLine(start, end, true);
        } else if (direction.first) {
            // start is in x2,y2, end is in x1,y1
            return sampleLine(start, end);
        } else if (direction.second) {
            // start is in x1,y1, end is in x2,y2,
            return sampleLine(end, start);
        }
    }

    return std::vector<Node>();
}

void PathDxfParser::endBlock() {
    currentBlockName = "";
}

std::vector<Node> PathDxfParser::sampleLine(const Eigen::Vector2d &start,
                                            const Eigen::Vector2d &end,
                                            const bool bidirectional) {
    std::vector<Node> samplePoints;

    double length = std::abs((end - start).norm());
    auto splits = static_cast<unsigned int>(std::ceil(length / maxEdgeLength));

    Eigen::Vector2d increment = (end - start) / splits;
    Eigen::Vector2d current = start;

    for (unsigned int i = 0; i < splits; i++) {
        Node n;
        n.uuid = addNodeCounter;
        n.isBidirectional = bidirectional;
        n.posX = current[0];
        n.posY = current[1];
        samplePoints.push_back(n);

        // add successor for the previous node
        if (i > 0) {
            samplePoints.at(i - 1).successors.push_back(addNodeCounter);

            if (bidirectional) {
                samplePoints.at(i).successors.push_back(addNodeCounter - 1);
            }
        }

        addNodeCounter++;
        current += increment;
    }

    if (!samplePoints.empty()) {
        // add end node, due to precision
        Node n;
        n.uuid = addNodeCounter;
        n.isBidirectional = bidirectional;
        n.posX = end[0];
        n.posY = end[1];
        samplePoints.back().successors.push_back(addNodeCounter);
        if (bidirectional) {
            n.successors.push_back(addNodeCounter - 1);
        }
        samplePoints.push_back(n);
        addNodeCounter++;
    } else {
        // no points has been sampled, add only start and end points
        Node nEnd;
        nEnd.uuid = addNodeCounter;
        nEnd.isBidirectional = bidirectional;
        nEnd.posX = end[0];
        nEnd.posY = end[1];
        if (bidirectional) {
            nEnd.successors.push_back(addNodeCounter + 1);
        }
        samplePoints.push_back(nEnd);
        addNodeCounter++;

        Node nStart;
        nStart.uuid = addNodeCounter;
        nStart.isBidirectional = bidirectional;
        nStart.posX = start[0];
        nStart.posY = start[1];
        nStart.successors.push_back(addNodeCounter -1);
        samplePoints.push_back(nStart);
        addNodeCounter++;
    }

    return samplePoints;
}


std::vector<Node> PathDxfParser::sampleArc(const DL_ArcData& arc) {
    std::vector<Node> samplePoints;

    // use radians
    double angleStart = DEG2RAD(arc.angle2);
    double angleEnd = DEG2RAD(arc.angle1);

    // change angle sign if it si needed
//    if (arc.angle1 >= arc.angle2) {
//        angleEnd += 2 * M_PI;
//    }

    double arcLength = std::abs((angleEnd - angleStart) * arc.radius);
    auto splits = static_cast<unsigned int>(std::ceil(arcLength / maxEdgeLength));
    double angleIncrement = (arcLength / splits) / arc.radius;

    for (unsigned int i = 0; i < splits; i++) {
        double angle = angleEnd - i * angleIncrement;
        Node n;
        n.uuid = addNodeCounter;
        n.posX = arc.cx + arc.radius * cos(angle);
        n.posY = arc.cy + arc.radius * sin(angle);
        samplePoints.push_back(n);

        // add successor for the previous node
        if (i > 0) {
            samplePoints.at(i - 1).successors.push_back(addNodeCounter);
        }
        addNodeCounter++;
    }

    if (!samplePoints.empty()) {
        Node n;
        n.uuid = addNodeCounter;
        n.posX = arc.cx + arc.radius * cos(angleStart);
        n.posY = arc.cy + arc.radius * sin(angleStart);
        samplePoints.back().successors.push_back(addNodeCounter);
        samplePoints.push_back(n);
        addNodeCounter++;
    } else {
        Node nEnd;
        nEnd.uuid = addNodeCounter;
        nEnd.posX = arc.cx + arc.radius * cos(angleEnd);
        nEnd.posY = arc.cy + arc.radius * sin(angleEnd);
        samplePoints.push_back(nEnd);
        addNodeCounter++;

        Node nStart;
        nStart.uuid = addNodeCounter;
        nStart.posX = arc.cx + arc.radius * cos(angleStart);
        nStart.posY = arc.cy + arc.radius * sin(angleStart);
        nStart.successors.push_back(addNodeCounter -1);
        samplePoints.push_back(nStart);
        addNodeCounter++;
    }

    return samplePoints;
}

bool PathDxfParser::nodePosesAreEqual(const Node& n1, const Node& n2) {
    return pointPositionsAreEqual(n1.posX, n1.posY, n2.posX, n2.posY);
}

bool PathDxfParser::pointPositionsAreEqual(double x1, double y1, double x2, double y2) {
    return NODES_INTERSECTION_TOLERANCE >= std::abs(std::abs(x1) - std::abs(x2))
           && NODES_INTERSECTION_TOLERANCE >= std::abs(std::abs(y1) - std::abs(y2));
}

std::pair<bool, bool> PathDxfParser::detectPathDirection(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                                                         const std::vector<DL_LineData>& arrowLines) {

    std::pair<bool, bool> direction;
    for (auto arrowLine: arrowLines) {
        if (intersectPointWithLineEndpoints(p1[0], p1[1], arrowLine)) {
            direction.first = true;
        }
        if (intersectPointWithLineEndpoints(p2[0], p2[1], arrowLine)) {
            direction.second = true;
        }
    }

    return direction;
}

bool PathDxfParser::intersectPointWithLineEndpoints(double x, double y, const DL_LineData& line) {
    return pointPositionsAreEqual(x, y, line.x1, line.y1) || pointPositionsAreEqual(x, y, line.x2, line.y2);
}

void PathDxfParser::assertPathDirection(const std::pair<bool, bool>& direction, const std::string& blockName) {
//    if (direction.first && direction.second) {
//        throw std::runtime_error("Detected two arrows in '" + blockName + "' block.");
//    } else
    if (!direction.first && !direction.second) {
        throw std::runtime_error("No direction detected in '" + blockName + "' block.");
    }
}

std::map<unsigned int, Node>::iterator PathDxfParser::findSameNodePosition(
        std::map<unsigned int, Node>& graph,
        const Node& node) {
    for (auto nodeIt = graph.begin(); nodeIt != graph.end(); nodeIt++) {
        if (nodePosesAreEqual(nodeIt->second, node)) {
            return nodeIt;
        }
    }

    return graph.end();
}

std::vector<Node> PathDxfParser::generateGraph() {
    std::map<unsigned int, Node> graph;
    std::map<unsigned int, Node> mergedNodes;

    for (const auto& segment: sampledPathNodes) {
        for (const auto& node : segment) {
            if (node.uuid == segment.front().uuid || node.uuid == segment.back().uuid) {
                auto sameNode = findSameNodePosition(graph, node);
                if (sameNode == graph.end()) {
                    // same position has not been found
                    graph[node.uuid] = node;
                } else {
                    // add it to merged nodes map
                    mergedNodes[node.uuid] = sameNode->second;
                    sameNode->second.isBidirectional = sameNode->second.isBidirectional || node.isBidirectional;
                    sameNode->second.successors.insert(sameNode->second.successors.end(),
                                                       node.successors.begin(),
                                                       node.successors.end());

                }
            } else {
                graph[node.uuid] = node;
            }
        }
    }

    for (auto& node: graph) {
        for (auto& successor: node.second.successors) {
            Node suc;
            suc.uuid = successor;

            auto foundIt = mergedNodes.find(successor);
            if (foundIt != mergedNodes.end()) {
                successor = foundIt->second.uuid;
            }
        }
    }

    std::vector<Node> nodes;
    for (const auto& graphNodes: graph) {
        nodes.push_back(graphNodes.second);
    }
    return nodes;
}

template<typename T>
bool PathDxfParser::removeElemByValue(std::vector<T>& v, T value) {
    auto position = std::find(v.begin(), v.end(), value);
    if (position != v.end()) {
        v.erase(position);
        return true;
    } else {
        return false;
    }
}
