//
// Created by jakub on 21.2.2020.
//

#include <ros/ros.h>
#include <amr_planner/RosWrapper.h>
#include <amr_planner/Graph.h>

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "planner" );
    ros::NodeHandle nh("~");

    RosWrapper planner(nh);


//    Node n1;
//    Node n2;
//
//    n1.uuid = 1;
//    n2.uuid = 2;
//    Graph g;
//    g.addEdge(n1, n2);
//    g.addEdge(n1, n2);
//    g.addEdge(n1, n2);
//    g.writeToFile("/home/jakub/amr_ws/my_test2.dot");


    ros::spin();
    return 0;
}







//
//
////
////=======================================================================
//// Copyright (c) 2004 Kristopher Beevers
////
//// Distributed under the Boost Software License, Version 1.0. (See
//// accompanying file LICENSE_1_0.txt or copy at
//// http://www.boost.org/LICENSE_1_0.txt)
////=======================================================================
////
//
//
//#include <ros/ros.h>
//#include <boost/graph/astar_search.hpp>
//#include <boost/graph/adjacency_list.hpp>
//#include <boost/graph/random.hpp>
//#include <boost/graph/connected_components.hpp>
//#include <boost/random.hpp>
//#include <boost/graph/graphviz.hpp>
//#include <sys/time.h>
//#include <vector>
//#include <list>
//#include <iostream>
//#include <fstream>
//#include <math.h>    // for sqrt
//
//using namespace boost;
//using namespace std;
//
//
//// auxiliary types
//struct location
//{
//    float y, x; // lat, long
//};
//typedef float cost;
//
//template <class Name, class LocMap>
//class city_writer {
//public:
//    city_writer(Name n, LocMap l, float _minx, float _maxx,
//                float _miny, float _maxy,
//                unsigned int _ptx, unsigned int _pty)
//            : name(n), loc(l), minx(_minx), maxx(_maxx), miny(_miny),
//              maxy(_maxy), ptx(_ptx), pty(_pty) {}
//    template <class Vertex>
//    void operator()(ostream& out, const Vertex& v) const {
//        float px = 1 - (loc[v].x - minx) / (maxx - minx);
//        float py = (loc[v].y - miny) / (maxy - miny);
//        out << "[label=\"" << name[v] << "\", pos=\""
//            << static_cast<unsigned int>(ptx * px) << ","
//            << static_cast<unsigned int>(pty * py)
//            << "\", fontsize=\"11\"]";
//    }
//private:
//    Name name;
//    LocMap loc;
//    float minx, maxx, miny, maxy;
//    unsigned int ptx, pty;
//};
//
//template <class WeightMap>
//class time_writer {
//public:
//    time_writer(WeightMap w) : wm(w) {}
//    template <class Edge>
//    void operator()(ostream &out, const Edge& e) const {
//        out << "[label=\"" << wm[e] << "\", fontsize=\"11\"]";
//    }
//private:
//    WeightMap wm;
//};
//
//
//// euclidean distance heuristic
//template <class Graph, class CostType, class LocMap>
//class distance_heuristic : public astar_heuristic<Graph, CostType>
//{
//public:
//    typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
//    distance_heuristic(LocMap l, Vertex goal)
//            : m_location(l), m_goal(goal) {}
//    CostType operator()(Vertex u)
//    {
//        CostType dx = m_location[m_goal].x - m_location[u].x;
//        CostType dy = m_location[m_goal].y - m_location[u].y;
//        return ::sqrt(dx * dx + dy * dy);
//    }
//private:
//    LocMap m_location;
//    Vertex m_goal;
//};
//
//
//struct found_goal {}; // exception for termination
//
//// visitor that terminates when we find the goal
//template <class Vertex>
//class astar_goal_visitor : public boost::default_astar_visitor
//{
//public:
//    astar_goal_visitor(Vertex goal) : m_goal(goal) {}
//    template <class Graph>
//    void examine_vertex(Vertex u, Graph& g) {
//        if(u == m_goal)
//            throw found_goal();
//    }
//private:
//    Vertex m_goal;
//};
//
//
//int main(int argc, char **argv)
//{
//    ros::init ( argc, argv, "amr_planner" );
//    ros::NodeHandle nh;
//
//    // specify some types
//    typedef adjacency_list<listS, vecS, directedS, no_property,
//            property<edge_weight_t, cost> > mygraph_t;
//    typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
//    typedef mygraph_t::vertex_descriptor vertex;
//    typedef mygraph_t::edge_descriptor edge_descriptor;
//    typedef std::pair<int, int> edge;
//
//    // specify data
//    enum nodes {
//        Troy, LakePlacid, Plattsburgh, Massena, Watertown, Utica,
//        Syracuse, Rochester, Buffalo, Ithaca, Binghamton, Woodstock,
//        NewYork, N
//    };
//    const char *name[] = {
//            "Troy", "Lake Placid", "Plattsburgh", "Massena",
//            "Watertown", "Utica", "Syracuse", "Rochester", "Buffalo",
//            "Ithaca", "Binghamton", "Woodstock", "New York"
//    };
//    location locations[] = { // lat/long
//            {42.73, 73.68}, {44.28, 73.99}, {44.70, 73.46},
//            {44.93, 74.89}, {43.97, 75.91}, {43.10, 75.23},
//            {43.04, 76.14}, {43.17, 77.61}, {42.89, 78.86},
//            {42.44, 76.50}, {42.10, 75.91}, {42.04, 74.11},
//            {40.67, 73.94}
//    };
//    edge edge_array[] = {
//            edge(Troy,Utica), edge(Troy,LakePlacid),
//            edge(Troy,Plattsburgh), edge(LakePlacid,Plattsburgh),
//            edge(Plattsburgh,Massena), edge(LakePlacid,Massena),
//            edge(Massena,Watertown), edge(Watertown,Utica),
//            edge(Watertown,Syracuse), edge(Utica,Syracuse),
//            edge(Syracuse,Rochester), edge(Rochester,Buffalo),
//            edge(Syracuse,Ithaca), edge(Ithaca,Binghamton),
//            edge(Ithaca,Rochester), edge(Binghamton,Troy),
//            edge(Binghamton,Woodstock), edge(Binghamton,NewYork),
//            edge(Syracuse,Binghamton), edge(Woodstock,Troy),
//            edge(Woodstock,NewYork)
//    };
//    unsigned int num_edges = sizeof(edge_array) / sizeof(edge);
//    cost weights[] = { // estimated travel time (mins)
//            96, 134, 143, 65, 115, 133, 117, 116, 74, 56,
//            84, 73, 69, 70, 116, 147, 173, 183, 74, 71, 124
//    };
//
//
//    // create graph
//    mygraph_t g(N);
//    WeightMap weightmap = get(edge_weight, g);
//    for(std::size_t j = 0; j < num_edges; ++j) {
//        edge_descriptor e; bool inserted;
//        boost::tie(e, inserted) = addEdge(edge_array[j].first,
//                                           edge_array[j].second, g);
//        weightmap[e] = weights[j];
//    }
//
//    {   // for bidirectionalS:
//        for (auto ed : boost::make_iterator_range(boost::out_edges(Troy, g)))
//            std::cout << "outgoing: " << ed << "\n";
//
////        for (auto ed : boost::make_iterator_range(boost::in_edges(Troy, g)))
////            std::cout << "incident: " << ed << "\n";
//    }
//
//
//    // pick random start/goal
//    boost::mt19937 gen(time(0));
//    vertex start = random_vertex(g, gen);
//    vertex goal = random_vertex(g, gen);
//
//
//    cout << "Start vertex: " << name[start] << endl;
//    cout << "Goal vertex: " << name[goal] << endl;
//
//    ofstream dotfile;
//    dotfile.open("/home/jakub/amr_ws/test-astar-cities.dot");
//    write_graphviz(dotfile, g,
//                   city_writer<const char **, location*>
//                           (name, locations, 73.46, 78.86, 40.67, 44.93,
//                            480, 400),
//                   time_writer<WeightMap>(weightmap));
//
//
//    vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
//    vector<cost> d(num_vertices(g));
//    try {
//        // call astar named parameter interface
//        astar_search_tree
//                (g, start,
//                 distance_heuristic<mygraph_t, cost, location*>
//                         (locations, goal),
//                 predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, g))).
//                         distance_map(make_iterator_property_map(d.begin(), get(vertex_index, g))).
//                         visitor(astar_goal_visitor<vertex>(goal)));
//
//
//    } catch(found_goal fg) { // found a path to the goal
//        list<vertex> shortest_path;
//        for(vertex v = goal;; v = p[v]) {
//            shortest_path.push_front(v);
//            if(p[v] == v)
//                break;
//        }
//        cout << "Shortest path from " << name[start] << " to "
//             << name[goal] << ": ";
//        list<vertex>::iterator spi = shortest_path.begin();
//        cout << name[start];
//        for(++spi; spi != shortest_path.end(); ++spi)
//            cout << " -> " << name[*spi];
//        cout << endl << "Total travel time: " << d[goal] << endl;
//        return 0;
//    }
//
//    cout << "Didn't find a path from " << name[start] << "to"
//         << name[goal] << "!" << endl;
//    return 0;
//
//}


////=======================================================================
//// Copyright 2001 Jeremy G. Siek, Andrew Lumsdaine, Lie-Quan Lee,
////
//// Distributed under the Boost Software License, Version 1.0. (See
//// accompanying file LICENSE_1_0.txt or copy at
//// http://www.boost.org/LICENSE_1_0.txt)
////=======================================================================
//#include <boost/config.hpp>
//#include <iostream>
//#include <fstream>
//
//#include <boost/graph/graph_traits.hpp>
//#include <boost/graph/adjacency_list.hpp>
//#include <boost/graph/dijkstra_shortest_paths.hpp>
//#include <boost/property_map/property_map.hpp>
//
//using namespace boost;
//
//int
//main(int, char *[])
//{
//    typedef adjacency_list < listS, vecS, directedS,
//            no_property, property < edge_weight_t, int > > graph_t;
//    typedef graph_traits < graph_t >::vertex_descriptor vertex_descriptor;
//    typedef std::pair<int, int> Edge;
//
//    const int num_nodes = 5;
//    enum nodes { A, B, C, D, E };
//    char name[] = "ABCDE";
//    Edge edge_array[] = { Edge(A, C), Edge(B, B), Edge(B, D), Edge(B, E),
//                          Edge(C, B), Edge(C, D), Edge(D, E), Edge(E, A), Edge(E, B)
//    };
//    int weights[] = { 1, 2, 1, 2, 7, 3, 1, 1, 1 };
//    int num_arcs = sizeof(edge_array) / sizeof(Edge);
//    graph_t g(edge_array, edge_array + num_arcs, weights, num_nodes);
//    property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, g);
//    std::vector<vertex_descriptor> p(num_vertices(g));
//    std::vector<int> d(num_vertices(g));
//    vertex_descriptor s = vertex(A, g);
//
//    dijkstra_shortest_paths(g, s,
//                            predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))).
//                                    distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, g))));
//
//    std::cout << "distances and parents:" << std::endl;
//    graph_traits < graph_t >::vertex_iterator vi, vend;
//    for (boost::tie(vi, vend) = vertices(g); vi != vend; ++vi) {
//        std::cout << "distance(" << name[*vi] << ") = " << d[*vi] << ", ";
//        std::cout << "parent(" << name[*vi] << ") = " << name[p[*vi]] << std::
//        endl;
//    }
//    std::cout << std::endl;
//
//    std::ofstream dot_file("figs/dijkstra-eg.dot");
//
//    dot_file << "digraph D {\n"
//             << "  rankdir=LR\n"
//             << "  size=\"4,3\"\n"
//             << "  ratio=\"fill\"\n"
//             << "  edge[style=\"bold\"]\n" << "  node[shape=\"circle\"]\n";
//
//    graph_traits < graph_t >::edge_iterator ei, ei_end;
//    for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
//        graph_traits < graph_t >::edge_descriptor e = *ei;
//        graph_traits < graph_t >::vertex_descriptor
//                u = source(e, g), v = target(e, g);
//        dot_file << name[u] << " -> " << name[v]
//                 << "[label=\"" << get(weightmap, e) << "\"";
//        if (p[v] == u)
//            dot_file << ", color=\"black\"";
//        else
//            dot_file << ", color=\"grey\"";
//        dot_file << "]";
//    }
//    dot_file << "}";
//    return EXIT_SUCCESS;
//}








//
//
//
//#include "boost/graph/adjacency_list.hpp"
//#include "boost/graph/breadth_first_search.hpp"
//#include "boost/graph/dijkstra_shortest_paths.hpp"
//#include "boost/graph/graph_utility.hpp"
//#include "boost/graph/prim_minimum_spanning_tree.hpp"
//#include "boost/graph/visitors.hpp"
//#include <fstream>
//#include <iostream>
//#include <map>
//#include <sstream>
//#include <string>
//#include <vector>
////#include "boost/property_map/property_map.hpp"
//#include "boost/graph/astar_search.hpp"
//
//typedef boost::adjacency_list_traits<boost::vecS, boost::vecS, boost::undirectedS> GraphTraits;
//
//typedef GraphTraits::vertex_descriptor Vertex;
//
//struct VertexProperty {
//    std::string name;
//    Vertex predecessor;
//    double distance;
//    boost::default_color_type color;
//    VertexProperty(const std::string &aName = "") : name(aName){};
//};
//
//struct EdgeProperty {
//    double weight; // distance to travel along this edge.
//    EdgeProperty(double aWeight = 0.0) : weight(aWeight){};
//};
//
//typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperty, EdgeProperty> Graph;
//
//Graph g;
//
//struct do_nothing_dijkstra_visitor : boost::default_dijkstra_visitor {};
//
//int main() {
//
//    std::string tempName1, tempName2, tempString, data2;
//    int weight;
//    std::string inputFile;
//    int choice;
//    Vertex cur_v, start_v, goal_v;
//    std::map<std::string, Vertex> name2v, name1v;
//    double totalDist, tempDist;
//    int numVert = 0;
//
//    while (1) {
//        std::cout << "\n-------------------------\n";
//        std::cout << "    Graph Implementation";
//        std::cout << "\n-------------------------\n\n";
//        std::cout << "1. Insert File into Graph\n";
//        std::cout << "2. Shortest Paths in a Network\n";
//        std::cout << "3. Minimum Spanning Tree\n";
//        std::cout << "4. Travelling Salesman\n";
//        std::cout << "5. Exit\n\n";
//        std::cout << "Selection: ";
//        std::cin >> choice;
//        std::cout << "\n";
//        switch (choice) {
//            case 1: {
//                std::cout << "Please enter name of file you would like to input: ";
//                std::cin >> inputFile;
//                std::cout << "\n";
//                std::ifstream fin;
//                fin.open(inputFile);
//                if (!fin) {
//                    std::cerr << "Cant open data file, goodbye...\n";
//                    system("Pause");
//                    return EXIT_FAILURE;
//                } else {
//                    std::cout << "File loaded.\n\n";
//                }
//                // build graph based on file loaded
//                getline(fin, tempString);
//                getline(fin, tempString);
//                std::stringstream tempSS(tempString);
//                while (getline(tempSS, tempName1, ',')) {
//                    name2v[tempName1] = add_vertex(VertexProperty(tempName1), g);
//                    numVert++;
//                }
//                getline(fin, tempString);
//                while (getline(fin, tempString)) {
//                    tempString.erase(tempString.begin(), tempString.begin() + tempString.find('(') + 1);
//                    tempString.erase(tempString.begin() + tempString.find(')'), tempString.end());
//                    std::stringstream temp_ss(tempString);
//                    getline(temp_ss, tempName1, ',');
//                    getline(temp_ss, tempName2, ',');
//                    temp_ss >> weight;
//                    addEdge(name2v[tempName1], name2v[tempName2], EdgeProperty(weight), g);
//                }
//                name1v = name2v;
//                std::cout << "Graph Created\n";
//                print_graph(g, get(&VertexProperty::name, g));
//                break;
//            }
//            case 2: {
//                totalDist = 0;
//                std::cout << "\nPlease enter the location name to start from: ";
//                std::cin >> tempName1;
//                transform(tempName1.begin(), tempName1.end(), tempName1.begin(), ::toupper);
//                std::cout << "\nPlease enter the location name for the destination: ";
//                std::cin >> tempName2;
//                transform(tempName2.begin(), tempName2.end(), tempName2.begin(), ::toupper);
//
//                start_v = name2v[tempName1];
//                goal_v = name2v[tempName2];
//
//                dijkstra_shortest_paths(g, goal_v, get(&VertexProperty::predecessor, g), get(&VertexProperty::distance, g),
//                                        get(&EdgeProperty::weight, g), boost::identity_property_map(), std::less<double>(),
//                                        std::plus<double>(), std::numeric_limits<double>::infinity(), 0.0,
//                                        do_nothing_dijkstra_visitor(), get(&VertexProperty::color, g));
//                std::cout << "\n";
//                std::cout << "Shortest Path From " << tempName1 << " to " << tempName2 << ": \n";
//                cur_v = start_v;
//                while (cur_v != goal_v) {
//                    std::cout << "(" << g[cur_v].name << ", ";
//                    totalDist += g[cur_v].distance;
//                    tempDist = g[cur_v].distance;
//                    cur_v = g[cur_v].predecessor;
//                    std::cout << g[cur_v].name << ")\n";
//                    totalDist -= g[cur_v].distance;
//                };
//                std::cout << "Total Weight: " << totalDist << "\n";
//                name2v = name1v;
//                break;
//            }
//            case 3: {
//                totalDist = 0;
//                Graph::vertex_descriptor start_w;
//                std::cout << "Please enter the Vertex you would like to start at: ";
//                {
//                    std::string startName;
//                    std::cin >> startName;
//                    transform(startName.begin(), startName.end(), startName.begin(), ::toupper);
//                    start_w = name2v.at(startName);
//                }
//
//                prim_minimum_spanning_tree(g, start_w, get(&VertexProperty::predecessor, g),
//                                           get(&VertexProperty::distance, g), get(&EdgeProperty::weight, g),
//                                           boost::identity_property_map(), do_nothing_dijkstra_visitor());
//                std::cout << "\n";
//                std::cout << "Minimum Spanning Tree: \n";
//                for (auto vd : boost::make_iterator_range(vertices(g))) {
//                    auto p = g[vd].predecessor;
//                    if (g[vd].name != g[p].name) {
//                        std::cout << "(" << g[vd].name << ", " << g[p].name << ")\n";
//                        totalDist += g[vd].distance;
//                    }
//                }
//                std::cout << "Total Weight: " << totalDist;
//                name2v = name1v;
//                break;
//            }
//            case 4:
//                Graph::vertex_descriptor start_x;
//                std::cout << "Please enter the Vertex you would like to start at: ";
//                {
//                    std::string startName;
//                    std::cin >> startName;
//                    transform(startName.begin(), startName.end(), startName.begin(), ::toupper);
//                    start_x = name2v.at(startName);
//                }
//                break;
//            case 5: {
//                std::cout << "Goodbye\n";
//                exit(1);
//                break;
//            }
//            default:
//                std::cout << "Make a correct choice\n";
//        }
//    }
//}
