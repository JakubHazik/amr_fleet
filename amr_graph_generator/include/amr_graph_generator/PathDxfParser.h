#include <utility>

//
// Created by jakub on 28.2.2020.
//

#ifndef PROJECT_PATHDXFPARSER_H
#define PROJECT_PATHDXFPARSER_H

#include <dxflib/dl_creationadapter.h>
#include <dxflib/dl_entities.h>
#include <vector>
#include <eigen3/Eigen/StdVector>
#include <amr_graph_generator/DataTypes.h>
#include <boost/asio/detail/shared_ptr.hpp>
#include <memory>

constexpr double NODES_INTERSECTION_TOLERANCE = 0.000001;

template<typename T>
constexpr T sqr(T x) {
    return ((x)*(x));
}

template<typename T>
constexpr T DEG2RAD(T x) {
    return x * M_PI / 180;
}


struct Block {
    std::vector<DL_LineData> pathLines;
    std::vector<DL_ArcData> pathArcs;
    std::vector<DL_LineData> arrowLines;
};


/**
 * https://qcad.org/en/90-dxflib
 * https://qcad.org/doc/dxflib/2.5/classref/class_d_l___creation_interface.html#a34
 */
class PathDxfParser : public DL_CreationAdapter {
public:

    explicit PathDxfParser(const std::string& dxfFilepath, double edgeLength);

    std::vector<Node> generateGraph();


private:
    //overloaded functions for receiving the dxf entities
    void addLine(const DL_LineData& line) override;
    void addArc(const DL_ArcData& arc) override;
    void addImage(const DL_ImageData& image) override;
    void addBlock(const DL_BlockData& block) override;
    void addInsert(const DL_InsertData &insert) override;
    void endBlock() override;

    /**
     *
     * @param start
     * @param end
     * @return vector of path samples, ordered from start to end
     */
    std::vector<Node> sampleLine(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const bool bidirectional=false);
    std::vector<Node> sampleArc(const DL_ArcData& arc);

    std::vector<Node> convertBlockToSegments(const Block& block, const DL_InsertData& insert);

    std::pair<bool, bool> detectPathDirection(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                                                  const std::vector<DL_LineData>& arrowLines);

    bool intersectPointWithLineEndpoints(double x, double y, const DL_LineData& line);

    void assertPathDirection(const std::pair<bool, bool>& direction, const std::string& blockName);

    bool nodePosesAreEqual(const Node& n1, const Node& n2);

    bool pointPositionsAreEqual(double x1, double y1, double x2, double y2);

    std::map<unsigned int, Node>::iterator findSameNodePosition(std::map<unsigned int, Node>& graph, const Node& node);

    template<typename T>
    bool removeElemByValue(std::vector<T>& v, T value);

    unsigned int addNodeCounter = 1;    // nodes uuid-s will start from 1, uuid=0 is not valid node
    double maxEdgeLength;

    std::string currentBlockName;
    std::map<std::string, Block> blocks;
    std::vector<DL_ImageData> images;
    std::vector<std::vector<Node>> sampledPathNodes;
};


#endif //PROJECT_PATHDXFPARSER_H
