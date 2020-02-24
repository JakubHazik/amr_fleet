//
// Created by jakub on 22. 2. 2020.
//

#ifndef SRC_GRAPHSEARCHMULTIROBOT_H
#define SRC_GRAPHSEARCHMULTIROBOT_H

#include <amr_planner/GraphSearchInterface.h>

#include <libMultiRobotPlanning/a_star.hpp>
//#include <libMultiRobotPlanning/neighbor.hpp>

#include <boost/functional/hash.hpp>


using libMultiRobotPlanning::AStar;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

//struct State {
//    State(int x, int y) : x(x), y(y) {}
//
//    State(const State&) = default;
//    State(State&&) = default;
//    State& operator=(const State&) = default;
//    State& operator=(State&&) = default;
//
//    bool operator==(const State& other) const {
//        return std::tie(x, y) == std::tie(other.x, other.y);
//    }
//
//    friend std::ostream& operator<<(std::ostream& os, const State& s) {
//        return os << "(" << s.x << "," << s.y << ")";
//    }
//
//    int x;
//    int y;
//};

typedef Node State;


namespace std {
    template <>
    struct hash<State> {
        size_t operator()(const State& s) const {
            size_t seed = 0;
            boost::hash_combine(seed, s.posX);
            boost::hash_combine(seed, s.posY);
            boost::hash_combine(seed, s.uuid);
            return seed;
        }
    };
}  // namespace std

enum class Action {
    Next
};

//std::ostream& operator<<(std::ostream& os, const Action& a) {
//    switch (a) {
//        case Action::Up:
//            os << "Up";
//            break;
//        case Action::Down:
//            os << "Down";
//            break;
//        case Action::Left:
//            os << "Left";
//            break;
//        case Action::Right:
//            os << "Right";
//            break;
//    }
//    return os;
//}

class Environment {
public:
    Environment(const Graph& graph, State goal)
            : m_goal(std::move(goal)),
                graph(graph)
    {

    }

    double admissibleHeuristic(const State& s) {
        return std::abs(s.posX - m_goal.posX) + std::abs(s.posY - m_goal.posY);
    }

    bool isSolution(const State& s) {
        return s == m_goal;
    }

    void getNeighbors(const State& s, std::vector<Neighbor<State, Action, double> >& neighbors) {
        neighbors.clear();

        auto n = graph.getNeighbors(s);

        for (auto i : n) {
            neighbors.emplace_back(Neighbor<State, Action, double>(i.node, Action::Next, i.weight));
        }
    }



    void onExpandNode(const State& /*s*/, int /*fScore*/, int /*gScore*/) {}

    void onDiscover(const State& /*s*/, int /*fScore*/, int /*gScore*/) {}

public:


private:
    State m_goal;
    Graph graph;
};












class GraphSearchMultiRobot : private GraphSearchInterface {
public:
    enum class SearchMethod {
        A_STAR,
    };

    GraphSearchMultiRobot(const Graph& graph, GraphSearchMultiRobot::SearchMethod searchMethod);

    std::vector<Node> getPath(const Node& nStart, const Node& nEnd) override;

private:
    SearchMethod searchMethod;


};

#endif //SRC_GRAPHSEARCHMULTIROBOT_H
