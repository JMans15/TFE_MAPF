//
// Created by Arthur Mahy on 27/02/2023.
//

#ifndef TFE_MAPF_COOPERATIVEASTAR_H
#define TFE_MAPF_COOPERATIVEASTAR_H

#include "AStar.h"
#include "../Problems/MultiAgentProblemWithConstraints.h"
#include "../Problems/SingleAgentProblemWithConstraints.h"

// Cooperative A* search - not optimal
// Only for multi agent problem
//
// Consists of (numberOfAgents) single agent A* searches
// where we update a reservation table with the several (position, timestep) points of the planned paths
// These (position, timestep) points will be avoided during the searches of the remaining agents
//
// typeOfHeuristic is the heuristic for the single agent A* searches : Manhattan (Cooperative A*) or OptimalDistance (Hierarchical Cooperative A* with a Reverse Resumable A*)
//
// The search has 2 drawbacks here:
// - Agents don't cooperate once they have a planned path (for example, problem if narrow corridor)
//   Ideally, agent should allow to move of its target to allow others to pass.
// - It depends on the order of the agents (see prioritized planning Latombe 1991)
// Could be improved with WINDOWED Hierarchical Cooperative A*
//
// We don't take into account the setOfSoftConstraints attribute of problem
// We don't take into account the maxCost attribute of problem
class CooperativeAStar {
public:
    CooperativeAStar(std::shared_ptr<MultiAgentProblemWithConstraints> problem, TypeOfHeuristic typeOfHeuristic)
            : problem(problem)
            , typeOfHeuristic(typeOfHeuristic)
    {}

    std::shared_ptr<Solution> solve() {

        if (typeOfHeuristic == Manhattan){
            LOG("===== Cooperative A* Search ====");
        } else if (typeOfHeuristic == OptimalDistance){
            LOG("===== Hierarchical cooperative A* Search ====");
        } else {
            LOG("Not a valid heuristic.");
            return {};
        }

        if (problem->isImpossible()){
            return std::make_shared<Solution>();
        }

        ObjectiveFunction objectiveFunction;
        if (problem->getObjFunction() == Makespan || problem->getObjFunction() == SumOfCosts){
            objectiveFunction = Makespan;
        } else {
            objectiveFunction = Fuel;
        }

        auto verticesReservationTable = problem->getSetOfHardVertexConstraints();
        auto edgesReservationTable = problem->getSetOfHardEdgeConstraints();

        int numberOfTimesteps = 0;
        std::unordered_map<int, std::vector<int>> positions;
        LOG("Beginning the (numberOfAgents) single agent A* searches. ");
        for (int a = 0; a < problem->getNumberOfAgents(); a++){

            // Single agent A* search for agent a
            auto singleAgentProblem = std::make_shared<SingleAgentProblemWithConstraints>(problem->getGraph(), problem->getStarts()[a], problem->getTargets()[a], objectiveFunction, problem->getAgentIds()[a], verticesReservationTable, edgesReservationTable);
            auto solution = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(singleAgentProblem, typeOfHeuristic).solve();

            if (solution->getFoundPath()) {
                auto pathOfAgent = solution->getPathOfAgent(problem->getAgentIds()[a]);
                // We force that the next agents cannot go where the already planned agents go
                for (int agent = a+1; agent < problem->getNumberOfAgents(); agent++){
                    for (int t = 0; t < pathOfAgent.size(); t++){
                        // agent cannot go at position pathofagent[t] at time t
                        verticesReservationTable.insert({problem->getAgentIds()[agent], pathOfAgent[t], t});
                    }
                    for (int t = 1; t < pathOfAgent.size(); t++){
                        // to avoid edge conflict
                        edgesReservationTable.insert({problem->getAgentIds()[agent], pathOfAgent[t], pathOfAgent[t-1], t});
                    }
                }
                numberOfTimesteps = std::max(numberOfTimesteps, solution->getNumberOfTimesteps());
                positions[problem->getAgentIds()[a]]=pathOfAgent;
            } else {
                LOG("No path has been found for agent " << problem->getAgentIds()[a]);
                LOG("The solution is not valid");
                return std::make_shared<Solution>();
            }
        }
        // We put the paths in the right format
        for (int agentId : problem->getAgentIds()) {
            while (positions[agentId].size() < numberOfTimesteps) {
                positions[agentId].emplace_back(problem->getTargetOf(agentId));
            }
        }
        return std::make_shared<Solution>(numberOfTimesteps, positions);
    }
private:
    std::shared_ptr<MultiAgentProblemWithConstraints> problem;
    TypeOfHeuristic typeOfHeuristic;
};


#endif //TFE_MAPF_COOPERATIVEASTAR_H
