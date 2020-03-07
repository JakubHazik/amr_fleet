//
// Created by jakub on 21.2.2020.
//

#ifndef PROJECT_GRAPH_H
#define PROJECT_GRAPH_H

#include <ros/ros.h>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <sys/time.h>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <math.h>    // for sqrt
#include <amr_planner/DataTypesAndConversions.h>



// specify data
//enum nodes {
//    Troy, LakePlacid, Plattsburgh, Massena, Watertown, Utica,
//    Syracuse, Rochester, Buffalo, Ithaca, Binghamton, Woodstock,
//    NewYork, N
//};
//const char *name[] = {
//        "Troy", "Lake Placid", "Plattsburgh", "Massena",
//        "Watertown", "Utica", "Syracuse", "Rochester", "Buffalo",
//        "Ithaca", "Binghamton", "Woodstock", "New York"
//};
//location locations[] = { // lat/long
//        {42.73, 73.68}, {44.28, 73.99}, {44.70, 73.46},
//        {44.93, 74.89}, {43.97, 75.91}, {43.10, 75.23},
//        {43.04, 76.14}, {43.17, 77.61}, {42.89, 78.86},
//        {42.44, 76.50}, {42.10, 75.91}, {42.04, 74.11},
//        {40.67, 73.94}
//};
//edge edge_array[] = {
//        edge(Troy,Utica), edge(Troy,LakePlacid),
//        edge(Troy,Plattsburgh), edge(LakePlacid,Plattsburgh),
//        edge(Plattsburgh,Massena), edge(LakePlacid,Massena),
//        edge(Massena,Watertown), edge(Watertown,Utica),
//        edge(Watertown,Syracuse), edge(Utica,Syracuse),
//        edge(Syracuse,Rochester), edge(Rochester,Buffalo),
//        edge(Syracuse,Ithaca), edge(Ithaca,Binghamton),
//        edge(Ithaca,Rochester), edge(Binghamton,Troy),
//        edge(Binghamton,Woodstock), edge(Binghamton,NewYork),
//        edge(Syracuse,Binghamton), edge(Woodstock,Troy),
//        edge(Woodstock,NewYork)
//};
//unsigned int num_edges = sizeof(edge_array) / sizeof(edge);
//cost weights[] = { // estimated travel time (mins)
//        96, 134, 143, 65, 115, 133, 117, 116, 74, 56,
//        84, 73, 69, 70, 116, 147, 173, 183, 74, 71, 124
//};

using namespace boost;


typedef boost::adjacency_list_traits<boost::vecS, boost::vecS, boost::undirectedS> GraphTraits;

typedef GraphTraits::vertex_descriptor vertex_t;


//typedef boost::property<boost::edge_weight_t, int> EdgeWeightProperty;

typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
typedef adjacency_list<listS, vecS, directedS, Node, EdgeWeightProperty> graph_t;
using vertexIt_t = boost::graph_traits<graph_t>::vertex_iterator;
//typedef boost::graph_traits<graph_t>::edge_descriptor edge_t;
using edgeIt_t = boost::graph_traits<graph_t>::edge_iterator;
typedef graph_t::edge_descriptor edge_t;


template <class Name>
class label_writer {
public:
    label_writer(Name _name) : name(_name) {}
    template <class VertexOrEdge>
    void operator()(std::ostream& out, const VertexOrEdge& v) const {
        out << "[label=\"" << name[v] << "\"]";
    }
private:
    Name name;
};



class Graph {
public:
    Graph() = default;

    void addEdge(const Node& nFrom, const Node& nTo);
//    void add_node(const Node& n);
    void writeToFile(const std::string& filepath);

    graph_t getBoostGraph();

    bool findVertex(const Node& node, vertex_t& foundNodeIt);

    std::vector<Neighbor> getNeighbors(const Node& currentNode);

    Node getNearestNode(double x, double y);

    Node getNode(unsigned int uuid);

    void printGraph();

    void clear();

private:

    graph_t graph;


//    bool findEdge(const Node& n1, const Node& n2, edge_t& foundNodesEdge);
};


#endif //PROJECT_GRAPH_H
