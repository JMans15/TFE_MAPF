//
// Created by Arthur Mahy on 27/02/2023.
//

#include "CooperativeAStar.h"

CooperativeAStar::CooperativeAStar(std::shared_ptr<MultiAgentProblem> problem, TypeOfHeuristic typeOfHeuristic)
    : problem(problem)
    , typeOfHeuristic(typeOfHeuristic)
{}

Solution CooperativeAStar::solve() {
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
    std::vector<vector<int>> positionsAtTime;
    for (int t = 0; t < numberOfTimesteps; t++){
        std::vector<int> pos;
        for (int agent = 0; agent < problem->getNumberOfAgents(); agent++) {
            if (t >= positions[agent].size()) {
                pos.emplace_back(positions[agent][positions[agent].size()-1]);
            } else {
                pos.emplace_back(positions[agent][t]);
            }
        }
        positionsAtTime.emplace_back(pos);
    }
    return {cost, numberOfVisitedStates, numberOfTimesteps, positionsAtTime};
}
