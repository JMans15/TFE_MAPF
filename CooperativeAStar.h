//
// Created by Arthur Mahy on 27/02/2023.
//

#ifndef TFE_MAPF_COOPERATIVEASTAR_H
#define TFE_MAPF_COOPERATIVEASTAR_H

#include "AStar.h"
#include "MultiAgentProblem.h"
#include "SingleAgentSpaceTimeProblem.h"

// Cooperative A* search - not optimal
// Only for multi agent problem
//
// Consists of (numberOfAgents) single agent A* searches
// where we update a reservation table with the several (position, timestep) points of the planned paths
// These (position, timestep) points will be avoided during the searches of the remaining agents
//
// typeOfHeuristic is the heuristic for the single agent A* searches : Manhattan (Cooperative A*) or OptimalDistance (Hierarchical Cooperative A* with a Reverse Resumable A*)
//
// The search has 3 drawbacks here:
// - Agents don't cooperate once they have a planned path (for example, problem if narrow corridor)
//   Ideally, agent should allow to move of its target to allow others to pass.
// - It depends on the order of the agents (see prioritized planning Latombe 1991)
// - To avoid edge conflict, we made the following decision:
//   when we know that a planned path has a (p,t) point, we put (p,t) in the reservation AND ALSO (p,t+1) !
//   We know this removes possible solutions
// Could be improved with WINDOWED Hierarchical Cooperative A*
class CooperativeAStar {
public:
    CooperativeAStar(std::shared_ptr<MultiAgentProblem> problem, TypeOfHeuristic typeOfHeuristic)
            : problem(problem)
            , typeOfHeuristic(typeOfHeuristic)
    {}
    Solution solve() {
        if (typeOfHeuristic == Manhattan){
            LOG("===== Cooperative A* Search ====");
        } else if (typeOfHeuristic == OptimalDistance){
            LOG("===== Hierarchical cooperative A* Search ====");
        } else {
            LOG("Not a valid heuristic.");
            return {};
        }

        ObjectiveFunction objectiveFunction;
        if (problem->getObjFunction() == Makespan || problem->getObjFunction() == SumOfCosts){
            objectiveFunction = Makespan;
        } else {
            objectiveFunction = Fuel;
        }

        auto reservationTable = problem->getSetOfConstraints();

        int cost = 0;
        int numberOfVisitedStates = 0;
        int numberOfTimesteps = 0;
        std::vector<vector<int>> positions;
        LOG("Beginning the (numberOfAgents) single agent A* searches. ");
        for (int a = 0; a < problem->getNumberOfAgents(); a++){

            // Single agent A* search for agent a
            auto singleAgentProblem = std::make_shared<SingleAgentSpaceTimeProblem>(problem->getGraph(), problem->getStarts()[a], problem->getTargets()[a], objectiveFunction, reservationTable, a);
            auto aStar = AStar<SingleAgentSpaceTimeProblem, SingleAgentSpaceTimeState>(singleAgentProblem, typeOfHeuristic);
            auto solution = aStar.solve();

            if (solution.getFoundPath()) {
                auto pathOfAgent = solution.getPathOfAgent(0);
                for (int t = 0; t < pathOfAgent.size(); t++){
                    // We force that the next agents cannot go where the already planned agents go
                    for (int agent = a+1; agent < problem->getNumberOfAgents(); agent++){

                        // agent cannot go at position pathofagent[t] at time t
                        reservationTable.insert(Constraint{agent, pathOfAgent[t], t});

                        // to avoid edge conflict, we have to add that
                        // agent cannot go at position pathofagent[t] also at time t+1
                        reservationTable.insert(Constraint{agent, pathOfAgent[t], t+1});
                    }
                }
                if (problem->getObjFunction() == SumOfCosts) {
                    cost += solution.getSumOfCostsCost();
                } else if (problem->getObjFunction() == Fuel) {
                    cost += solution.getFuelCost();
                } else {
                    cost = std::max(cost, solution.getMakespanCost());
                }
                numberOfVisitedStates += solution.getNumberOfVisitedStates();
                numberOfTimesteps = std::max(numberOfTimesteps, solution.getNumberOfTimesteps());
                positions.emplace_back(pathOfAgent);
            } else {
                LOG("No path has been found for agent " << a);
                LOG("The solution is not valid");
                return {};
            }
        }
        // We put the paths in the right format
        for (int agent = 0; agent < problem->getNumberOfAgents(); agent++) {
            while (positions[agent].size() < numberOfTimesteps) {
                positions[agent].emplace_back(problem->getTargets()[agent]);
            }
        }
        return {cost, numberOfVisitedStates, numberOfTimesteps, positions};
    }
private:
    std::shared_ptr<MultiAgentProblem> problem;
    TypeOfHeuristic typeOfHeuristic;
};


#endif //TFE_MAPF_COOPERATIVEASTAR_H
