#include <utility>

//
// Created by jakub on 21.2.2020.
//

#ifndef PROJECT_GRAPHSEARCHINTERFACE_H
#define PROJECT_GRAPHSEARCHINTERFACE_H

#include <vector>
#include <amr_planner/Graph.h>

class GraphSearchInterface {
public:
    explicit GraphSearchInterface(Graph graph)
        :   graph(std::move(graph)) {

    };
    virtual ~GraphSearchInterface() = default;
    virtual std::vector<Node> getPath(const Node& nStart, const Node& nEnd) = 0;

protected:
    Graph graph;
};


#endif //PROJECT_GRAPHSEARCHINTERFACE_H
