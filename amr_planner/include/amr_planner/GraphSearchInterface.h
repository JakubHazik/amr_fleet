//
// Created by jakub on 21.2.2020.
//

#ifndef PROJECT_GRAPHSEARCHINTERFACE_H
#define PROJECT_GRAPHSEARCHINTERFACE_H

#include <amr_planner/Graph.h>
#include <vector>

class GraphSearchInterface {
public:
    explicit GraphSearchInterface(const Graph& graph)
        :   graph(graph) {

    };
    virtual ~GraphSearchInterface() = default;
    virtual std::vector<Node> getPath(const Node& nStart, const Node& nEnd) = 0;
protected:
    Graph graph;
};


#endif //PROJECT_GRAPHSEARCHINTERFACE_H
