//
// Created by Arthur Mahy on 27/02/2023.
//

#include "CooperativeAStar.h"

#ifdef DEBUG
#define LOG(str) cout << str << endl;
#else
#define LOG(str)
#endif

CooperativeAStar::CooperativeAStar(MultiAgentProblem* problem, TypeOfHeuristic typeOfHeuristic) {
    if (typeOfHeuristic==Manhattan){
        LOG("===== Cooperative A* Search ====");
    } else if (typeOfHeuristic==OptimalDistance){
        LOG("===== Hierarchical cooperative A* Search ====");
    } else {
        LOG("Not a valid heuristic.");
        solution = {};
        return;
    }
    set<Constraint> reservationTable = problem->getSetOfConstraints(); // hashmap instead of set ??
    int cost = 0;
    int numberOfVisitedStates = 0;
    int numberOfTimesteps = 0;
    ObjectiveFunction objectiveFunction;
    if (problem->getObjFunction()==Makespan or problem->getObjFunction()==SumOfCosts){
        objectiveFunction = Makespan;
    } else {
        objectiveFunction = Fuel;
    }
    vector<vector<int>> positions;
    LOG("Beginning the (numberOfAgents) single agent A* searches. ");
    for (int a = 0; a < problem->getNumberOfAgents(); a++){

        // Single agent A* search for agent a
        SingleAgentSpaceTimeProblem singleagentproblem = SingleAgentSpaceTimeProblem(problem->getGraph(), problem->getStarts()[a], problem->getTargets()[a], objectiveFunction, reservationTable, a);
        Solution solutionn = AStar(&singleagentproblem, typeOfHeuristic).getSolution();

        if (solutionn.getFoundPath()){
            vector<int> pathofagent = solutionn.getPathOfAgent(0);
            for (int t = 0; t < pathofagent.size(); t++){
                // We force that the next agents cannot go where the already planned agents go
                for (int agent = a+1; agent < problem->getNumberOfAgents(); agent++){

                    // agent cannot go at position pathofagent[t] at time t
                    reservationTable.insert(Constraint(agent, pathofagent[t], t));

                    // to avoid edge conflict, we have to add that
                    // agent cannot go at position pathofagent[t] also at time t+1
                    reservationTable.insert(Constraint(agent, pathofagent[t], t+1));
                }
            }
            if (problem->getObjFunction()==SumOfCosts){
                cost += solutionn.getSumOfCostsCost();
            } else if (problem->getObjFunction()==Fuel){
                cost += solutionn.getFuelCost();
            } else {
                cost = max(cost, solutionn.getMakespanCost());
            }
            numberOfVisitedStates += solutionn.getNumberOfVisitedStates();
            numberOfTimesteps = max(numberOfTimesteps, solutionn.getNumberOfTimesteps());
            positions.emplace_back(pathofagent);
        } else {
            LOG("No path has been found for agent " << a);
            LOG("The solution is not valid");
            solution = {};
            return;
        }
    }
    // We put the paths in the right format
    vector<vector<int>> positionsAtTime;
    for (int t = 0; t < numberOfTimesteps; t++){
        vector<int> pos;
        for (int agent = 0; agent < problem->getNumberOfAgents(); agent++){
            if (t >= positions[agent].size()){
                pos.emplace_back(positions[agent][positions[agent].size()-1]);
            } else {
                pos.emplace_back(positions[agent][t]);
            }
        }
        positionsAtTime.emplace_back(pos);
    }
    solution = {cost, numberOfVisitedStates, numberOfTimesteps, positionsAtTime};
}

Solution CooperativeAStar::getSolution() {
    return solution;
}
