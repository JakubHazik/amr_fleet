//
// Created by jakub on 22. 2. 2020.
//

#include "amr_planner/GraphSearchMultiRobot.h"

GraphSearchMultiRobot::GraphSearchMultiRobot(const Graph& graph, GraphSearchMultiRobot::SearchMethod searchMethod)
    :   GraphSearchInterface(graph),
        searchMethod(searchMethod) {





}

std::vector<Node> GraphSearchMultiRobot::getPath(const Node& nStart, const Node& nEnd) {


    Environment env(graph, nEnd);

    AStar<State, Action, double, Environment> astar(env);

    PlanResult<State, Action, double> solution;

    auto success = astar.search(nStart, solution);

    std::vector<Node> result;

    std::ofstream out("/home/jakub/amr_ws/plan_result");
    if (success) {
        std::cout << "Planning successful! Total cost: " << solution.cost
                  << std::endl;
//        for (size_t i = 0; i < solution.actions.size(); ++i) {
//            std::cout << solution.states[i].second << ": " << solution.states[i].first
////                      << "->" << solution.actions[i].first
////                      << "(cost: " << solution.actions[i].second << ")"
//                      << std::endl;
//        }
//        std::cout << solution.states.back().second << ": "
//                  << solution.states.back().first << std::endl;

        out << "schedule:" << std::endl;
        out << "  agent1:" << std::endl;

        for (size_t i = 0; i < solution.states.size(); ++i) {
            out << "    - x: " << solution.states[i].first.posX << std::endl
                << "      y: " << solution.states[i].first.posY << std::endl
                << "      t: " << i << std::endl;


            result.push_back(solution.states[i].first);
        }


    } else {
        std::cout << "Planning NOT successful!" << std::endl;
    }

    out.close();


    return result;
}
