//
// Created by Arthur Mahy on 27/02/2023.
//

#ifndef TFE_MAPF_ASTAR_H
#define TFE_MAPF_ASTAR_H

#include "../Heuristics/HeuristicOptimal.h"
#include "../Problems/Problem.h"
#include "../Solution/Solution.h"

#include <algorithm>
#include <unordered_map>

// Basic A* search
// Can be applied for multi agent (operator decomposition) and single agent problems
template <class P, class S>
class AStar {
public:
    AStar(std::shared_ptr<P> problem, TypeOfHeuristic typeOfHeuristic)
        : problem(problem)
        , heuristic(getHeuristic<P,S>(problem, std::make_shared<TypeOfHeuristic>(typeOfHeuristic)))
    {}

    std::shared_ptr<Solution> solve() {
        LOG("===== A* Search ====");
        LOG("Beginning the A* search. ");

        auto start = problem->getStartState();
        fringe.insert(std::make_shared<Node<S>>(start, 0, heuristic->heuristicFunction(start)));

        int numberOfVisitedStates = 0;

        while (!fringe.empty()){
            auto it = fringe.begin();
            const auto node = *it;
            
            fringe.erase(it);

            auto nodeState = node->getState();
            if (node->getCost() > distance[nodeState]) {
                continue;
            }

            numberOfVisitedStates += 1;

            if (problem->isGoalState(nodeState)){
                return retrieveSolution(numberOfVisitedStates, node);
            }

            auto successors = problem->getSuccessors(nodeState);
            for (auto &[successor, edgeCost] : successors) {
                auto successorCost = node->getCost() + edgeCost;
                auto it = distance.find(successor);
                if (it == distance.end() || successorCost < it->second) {
                    distance[successor] = successorCost;
                    auto h = heuristic->heuristicFunction(successor);
                    fringe.insert(std::make_shared<Node<S>>(successor, successorCost, h, node));
                }
            }

        }
        
        LOG("No path has been found.");
        return std::make_shared<Solution>();
    }
private:
    std::shared_ptr<P> problem;
    std::shared_ptr<Heuristic<S>> heuristic;
    std::multiset<std::shared_ptr<Node<S>>, NodeComparator<S>> fringe;
    std::unordered_map<std::shared_ptr<S>, int, StateHasher<S>, StateEquality<S>> distance; // the closed list

    std::shared_ptr<Solution> retrieveSolution(int numberOfVisitedStates, std::shared_ptr<Node<S>> node) {
        int cost = node->getCost();

        std::vector<std::shared_ptr<S>> states;
        while (node) {
            states.push_back(node->getState());
            node = node->getParent();
        }
        std::reverse(states.begin(), states.end());

        auto positions = problem->getPositions(states);
        int numberOfTimesteps = positions[0].size();
        
        return std::make_shared<Solution>(cost, numberOfVisitedStates, numberOfTimesteps, positions, problem->getAgentIds());
    }
};


#endif //TFE_MAPF_ASTAR_H
