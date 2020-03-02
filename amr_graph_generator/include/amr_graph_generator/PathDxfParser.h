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

constexpr double NODES_INTERSECTION_TOLERANCE = 0.0002;

template<typename T>
constexpr T sqr(T x) {
    return ((x)*(x));
}

struct Block {
    std::vector<DL_LineData> pathLines;
    std::vector<DL_LineData> arrowLines;
    std::vector<DL_ArcData> arcs;
};


/**
 * https://qcad.org/en/90-dxflib
 * https://qcad.org/doc/dxflib/2.5/classref/class_d_l___creation_interface.html#a34
 */
class PathDxfParser : public DL_CreationAdapter {
public:

    explicit PathDxfParser(const string& dxfFilepath, double edgeLength);

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
    std::vector<Node> sampleLine(const Eigen::Vector2d& start, const Eigen::Vector2d& end);
    std::vector<Line> splitCircle(const DL_CircleData &_circle, float _segLength);
    std::vector<Line> splitArc(const DL_ArcData&_arc, float _segLength);

    std::vector<Node> convertBlockToSegments(const Block& block, const DL_InsertData& insert);

    double lineLength(const DL_LineData& line);

    int getLargestLine(std::vector<DL_LineData>& lines);

    bool nodesPoseIsEqual(const Node& n1, const Node& n2);

    unsigned int addNodeCounter = 0;
    double maxEdgeLength;

    std::string currentBlockName;
    std::map<std::string, Block> blocks;
    std::vector<DL_ImageData> images;
    std::vector<std::vector<Node>> sampledPathNodes;
};


#endif //PROJECT_PATHDXFPARSER_H
