//
// Created by jakub on 21.2.2020.
//

#include <amr_graph_representation/Graph.h>

#include <fstream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <limits>

using namespace boost;

void Graph::readNewGraph(const amr_msgs::Graph& graphMsg) {
    clear();

    for (const amr_msgs::Node &node: graphMsg.nodes) {
        Node nFrom(node.point.uuid, node.point.pose.x, node.point.pose.y);
        for (const auto &successorNode: node.successors) {
            auto nodesIt = std::find_if(graphMsg.nodes.begin(), graphMsg.nodes.end(),
                                        [&successorNode](const amr_msgs::Node& obj) {return obj.point.uuid == successorNode;});
            auto index = std::distance(graphMsg.nodes.begin(), nodesIt);
            auto nextNode = graphMsg.nodes[index];
            Node nTo(nextNode.point.uuid, nextNode.point.pose.x, nextNode.point.pose.y);
            addEdge(nFrom, nTo);
        }
    }
}

void Graph::addEdge(const Node& nFrom, const Node& nTo) {
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
        boost::add_edge(v1, v2, Node::distance(nFrom, nTo), graph);
    }
}

void Graph::writeToFile(const std::string& filepath) {
    std::ofstream dotfile;
    dotfile.open(filepath);

    boost::dynamic_properties dp;
    dp.property("node_id", get(&Node::uuid, graph));
    dp.property("label", get(&Node::uuid, graph));
    dp.property("label", get(boost::edge_weight_t(), graph));

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


graph_t Graph::getBoostGraph() {
    return graph;
}




std::vector<Neighbor> Graph::getNeighbors(const Node& currentNode) {

    vertex_t currentVertex;
    if (findVertex(currentNode, currentVertex)) {

        std::vector<Neighbor> result;
        auto neighbours = adjacent_vertices(currentVertex, graph);


        for (auto n : make_iterator_range(neighbours)) {

            std::pair<edge_t, bool> ed = boost::edge(currentVertex, n, graph);
            double weight = get(boost::edge_weight_t(), graph, ed.first);

            Neighbor tmp {graph[n], weight};
            result.push_back(tmp);
        }

        return result;
    } else {
        return std::vector<Neighbor> {};
    }
}


void Graph::clear() {
    graph_t newGraph;
    graph = newGraph;
}

Node Graph::getNearestNode(double x, double y) {
    vertexIt_t vi, vi_end;
    std::pair<double, Node> minDistanceNode{std::numeric_limits<double>::max(), Node{}};

    for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi) {
        double distance = Node::distance(graph[*vi], x, y);

        if (distance < minDistanceNode.first) {
            minDistanceNode.first = distance;
            minDistanceNode.second = graph[*vi];
        }
    }

    return minDistanceNode.second;
}

vertexIt_t Graph::getNodeIt(unsigned int uuid) {
    vertexIt_t vi, vi_end;
    for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi) {
        if (uuid == graph[*vi].uuid) {
            return vi;
        }
    }

    throw std::out_of_range("Graph not contain node with uuid: " + std::to_string(uuid));
}

Node Graph::getNode(unsigned int uuid) {
    return graph[*getNodeIt(uuid)];
}

void Graph::setNodeReachability(unsigned int uuid, bool isReachable) {
    graph[*getNodeIt(uuid)].isReachable = isReachable;
}

//void Graph::printGraph() {
//    boost::print_graph(graph, boost::get(boost::edge_weight_t(), graph));
//}

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