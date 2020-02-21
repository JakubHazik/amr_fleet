//
// Created by jakub on 21.2.2020.
//

#include <amr_planner/Graph.h>

#include "amr_planner/Graph.h"

#include <fstream>

using namespace boost;


void Graph::add_edge(const Node& nFrom, const Node& nTo) {
    vertex_t v1, v2;
    if (!findVertex(nFrom, v1)) {
        v1 = boost::add_vertex(nFrom, graph);
    }
    if (!findVertex(nTo, v2)) {
        v2 = boost::add_vertex(nTo, graph);
    }

    // Create an edge conecting those two vertices
    // add edge if not exist
    if (!boost::edge(v1, v2, graph).second) {
        Edge edge {Node::distance(nFrom, nTo)}; // compute distance between nodes
        boost::add_edge(v1, v2, edge, graph);
    }
}

void Graph::write_to_file(const std::string& filepath) {
    std::ofstream dotfile;
    dotfile.open(filepath);

    boost::dynamic_properties dp;
    dp.property("node_id", get(&Node::uuid, graph));
    dp.property("label", get(&Node::uuid, graph));
//    dp.property("label", get(&Edge::distance, graph));

    write_graphviz_dp(dotfile, graph, dp);
    dotfile.close();
}

bool Graph::findVertex(const Node& node, vertex_t& foundNodeVertex){
    // todo simplify like edge
    vertexIt_t vi, vi_end;

    for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi) {
        if(graph[*vi].uuid == node.uuid) {
            foundNodeVertex = *vi;
            return true;
        }
    }
    return false;
}

//bool Graph::findEdge(const Node& n1, const Node& n2, edge_t& foundNodesEdge){
//    edgeIt_t ei, ei_end;
//
//    for (boost::tie(ei, ei_end) = edges(graph); ei != ei_end; ++ei) {
//        if (graph[*ei].distance) {
//
//        }
////        if(graph[*ei].uuid == node.uuid) {
////            foundNodeVertex = *vi;
////            return true;
////        }
//    }
//    return false;
//}