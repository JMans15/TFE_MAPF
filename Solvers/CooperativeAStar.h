//
// Created by Arthur Mahy on 27/02/2023.
//

#ifndef TFE_MAPF_COOPERATIVEASTAR_H
#define TFE_MAPF_COOPERATIVEASTAR_H

#include <utility>

#include "AStar/GeneralAStar.h"
#include "../Problems/MultiAgentProblem.h"
#include "../Problems/SingleAgentProblem.h"

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
// Could be improved with WINDOWED Hierarchical Cooperative A*)
class CooperativeAStar {
public:
    CooperativeAStar(std::shared_ptr<MultiAgentProblem> problem, TypeOfHeuristic typeOfHeuristic)
            : problem(std::move(problem))
            , typeOfHeuristic(typeOfHeuristic)
    {}

    std::shared_ptr<Solution> solve() {

        if (typeOfHeuristic == Manhattan){
            LOG("===== Cooperative A* Search ====");
        } else if (typeOfHeuristic == OptimalDistance){
            LOG("===== Hierarchical cooperative A* Search ====");
        } else {
            LOG("Not a valid heuristic.");
            return std::make_shared<Solution>();
        }

        if (problem->isImpossible()){
            return std::make_shared<Solution>();
        }

        auto verticesReservationTable = problem->getSetOfHardVertexConstraints();
        auto edgesReservationTable = problem->getSetOfHardEdgeConstraints();
        auto maxCost = problem->getMaxCost();

        int numberOfTimesteps = 0;
        std::unordered_map<int, std::vector<int>> positions;
        LOG("Beginning the (numberOfAgents) single agent A* searches. ");
        for (int a = 0; a < problem->getNumberOfAgents(); a++){

            // Single agent A* search for agent a
            auto singleAgentProblem = std::make_shared<SingleAgentProblem>(problem->getGraph(), problem->getStarts()[a], problem->getTargets()[a], problem->getObjFunction(), problem->getAgentIds()[a], verticesReservationTable, edgesReservationTable, maxCost, problem->getSetOfSoftVertexConstraints(), problem->getSetOfSoftEdgeConstraints());
            auto solution = GeneralAStar(singleAgentProblem, typeOfHeuristic).solve();

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
                if (problem->getObjFunction() != Makespan){
                    maxCost = maxCost - solution->getCost();
                }
            } else {
                LOG("No path has been found for agent " << problem->getAgentIds()[a]);
                return std::make_shared<Solution>();
            }
        }
        // We put the paths in the right format
        for (int agentId : problem->getAgentIds()) {
            while (positions[agentId].size() < numberOfTimesteps) {
                positions[agentId].emplace_back(problem->getTargetOf(agentId));
            }
        }
        return std::make_shared<Solution>(numberOfTimesteps, positions,0,0);
    }
private:
    std::shared_ptr<MultiAgentProblem> problem;
    TypeOfHeuristic typeOfHeuristic;
};


#endif //TFE_MAPF_COOPERATIVEASTAR_H
