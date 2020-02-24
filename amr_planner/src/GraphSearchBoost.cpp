//
// Created by jakub on 21.2.2020.
//

#include <amr_planner/GraphSearchBoost.h>
#include <boost/graph/astar_search.hpp>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <exception>

using namespace boost;


struct do_nothing_dijkstra_visitor : boost::default_dijkstra_visitor {};



GraphSearchBoost::GraphSearchBoost(const Graph& graph, GraphSearchBoost::SearchMethod searchMethod)
        : GraphSearchInterface(graph),
          searchMethod(searchMethod) {


}

std::vector<Node> GraphSearchBoost::getPath(const Node& nStart, const Node& nEnd) {

//    graph_t boost_graph = graph.getBoostGraph();
//    std::vector<vertex_t> p(boost::num_vertices(boost_graph));
//    std::vector<int> d(boost::num_vertices(boost_graph));
//
//
//    vertex_t startVertex;
//    if (!graph.findVertex(nStart, startVertex)) {
//        throw std::invalid_argument("Start Node is not in graph.");
//    }
//
//
//    boost::dijkstra_shortest_paths(boost_graph, startVertex,
//            boost::predecessor_map(boost::make_iterator_property_map(p.begin(), boost::get(boost::vertex_index, boost_graph)))
//            .distance_map(boost::make_iterator_property_map(d.begin(), boost::get(boost::vertex_index, boost_graph))));
//
//
//
//








//    graph_t boost_graph = graph.getBoostGraph();
//    std::vector<vertex_t> p(num_vertices(boost_graph));
//    std::vector<int> d(num_vertices(boost_graph));
//    vertex_t startVertex;
//    if (!graph.findVertex(nStart, startVertex)) {
//        throw std::invalid_argument("Start Node is not in graph.");
//    }
//
//    totalDist = 0;
//    std::cout << "\nPlease enter the location name to start from: ";
//    std::cin >> tempName1;
//    transform(tempName1.begin(), tempName1.end(), tempName1.begin(), ::toupper);
//    std::cout << "\nPlease enter the location name for the destination: ";
//    std::cin >> tempName2;
//    transform(tempName2.begin(), tempName2.end(), tempName2.begin(), ::toupper);
//
//    start_v = name2v[tempName1];
//    goal_v = name2v[tempName2];
//


//    dijkstra_shortest_paths(boost_graph, startVertex, get(&Node::predecessor, boost_graph), get(&Edge::distance, boost_graph),
//                            get(&Edge::distance, boost_graph), boost::identity_property_map(), std::less<double>(),
//                            std::plus<double>(), std::numeric_limits<double>::infinity(), 0.0,
//                            do_nothing_dijkstra_visitor(), get(&Node::color, boost_graph));
//    std::cout << "\n";
//    std::cout << "Shortest Path From " << tempName1 << " to " << tempName2 << ": \n";
//    cur_v = start_v;
//    while (cur_v != goal_v) {
//        std::cout << "(" << g[cur_v].name << ", ";
//        totalDist += g[cur_v].distance;
//        tempDist = g[cur_v].distance;
//        cur_v = g[cur_v].predecessor;
//        std::cout << g[cur_v].name << ")\n";
//        totalDist -= g[cur_v].distance;
//    };
//    std::cout << "Total Weight: " << totalDist << "\n";
//    name2v = name1v;
//    break;
//





//    graph_t boost_graph = graph.getBoostGraph();
//
//    std::vector<vertex_t> p(num_vertices(boost_graph));
//    std::vector<int> d(num_vertices(boost_graph));
//
//
//    vertex_t startVertex;
//    if (!graph.findVertex(nStart, startVertex)) {
//        throw std::invalid_argument("Start Node is not in graph.");
//    }
//
//    dijkstra_shortest_paths(boost_graph, startVertex, //
////             predecessor_map(&p[0]).distance_map(&d[0])
//                            predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, boost_graph))).
//                            distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, boost_graph)))
//
//                            );
//    graph_traits < graph_t >::vertex_iterator vi, vend;
//    for (boost::tie(vi, vend) = vertices(boost_graph); vi != vend; ++vi) {
//        std::cout << "distance(" << boost_graph[*vi].uuid << ") = " << d[*vi] << ", ";
//        std::cout << "parent(" << boost_graph[*vi].uuid << ") = " << p[*vi] << std::
//        endl;
//    }
//
//




//    astar_search_tree(
//            graph.getBoostGraph(),
//            startVertex,
//
//                      distance_heuristic<graph_t, cost, location*>(locations, goal),
//                      predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, g))).
//                              distance_map(make_iterator_property_map(d.begin(), get(vertex_index, g))).
//                              visitor(astar_goal_visitor<vertex>(goal)));

}

