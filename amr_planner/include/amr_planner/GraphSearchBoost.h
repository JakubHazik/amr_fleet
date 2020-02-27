//
// Created by jakub on 21.2.2020.
//

#ifndef PROJECT_GRAPHSEARCHBOOST_H
#define PROJECT_GRAPHSEARCHBOOST_H


#include <amr_planner/GraphSearchInterface.h>


template <class Graph, class CostType, class LocMap>
class distance_heuristic : public astar_heuristic<Graph, CostType>
{
public:
    typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
    distance_heuristic(LocMap l, Vertex goal)
            : m_location(l), m_goal(goal) {}
    CostType operator()(Vertex u)
    {
        CostType dx = m_location[m_goal].x - m_location[u].x;
        CostType dy = m_location[m_goal].y - m_location[u].y;
        return ::sqrt(dx * dx + dy * dy);
    }
private:
    LocMap m_location;
    Vertex m_goal;
};



class GraphSearchBoost : private GraphSearchInterface {
public:
    enum class SearchMethod {
        A_STAR,         // https://www.boost.org/doc/libs/1_58_0/libs/graph/doc/astar_search.html
        DIJGSTRA,       // https://www.boost.org/doc/libs/1_58_0/libs/graph/doc/dijkstra_shortest_paths.html
                        // example https://www.boost.org/doc/libs/1_58_0/libs/graph/example/dijkstra-example.cpp
        //todo example  https://paste.ubuntu.com/26136129/
    };

    GraphSearchBoost(const Graph& graph, GraphSearchBoost::SearchMethod searchMethod);

    std::vector<Node> getPath(const Node& nStart, const Node& nEnd) override;

private:
    SearchMethod searchMethod;




};


#endif //PROJECT_GRAPHSEARCHBOOST_H
