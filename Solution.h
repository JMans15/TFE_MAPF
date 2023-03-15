//
// Created by Arthur Mahy on 23/11/2022.
//

#ifndef TFE_MAPF_SOLUTION_H
#define TFE_MAPF_SOLUTION_H

#include <string>
#include <vector>

class Solution {
public:
    Solution();
    Solution(int numberOfTimesteps, std::vector<std::vector<int>> positions, std::vector<int> agentIds);
    Solution(int cost, int numberOfVisitedStates, int numberOfTimesteps, std::vector<std::vector<int>> positions, std::vector<int> agentIds);

    int getCost() const;
    int getNumberOfVisitedStates() const;
    int getNumberOfTimesteps() const;
    std::vector<int> getPositionsAtTime(int t);
    std::vector<int> getPathOfAgent(int id);
    bool getFoundPath();
    void print();
    void write(std::string filename, int w);

    // Fuel : Total amount of distance traveled by all agents
    int getFuelCost();

    // Makespan : Total time for the last agent to reach its goal
    int getMakespanCost();

    // SumOfCosts : The sum of the time steps required for every agent to reach its goal
    int getSumOfCostsCost();

    // Returns True if all paths don't have any conflict (vertex conflict and edge conflict)
    bool isValid();

    std::vector<std::vector<int>> getPositions() const;

    // Lengthens the matrix positions so that the length of a path is length
    void lengthenPositions(int length);

private:

    // Cost of the solution / value of the objective function for this solution (if solution of a single joint A*)
    int cost;

    // The number of visited (goal tested) states (if solution of a single joint A*)
    int numberOfVisitedStates;

    // The number of needed timesteps from all agents to reach their target position
    int numberOfTimesteps;

    // Matrix of size numberOfAgents X numberOfTimesteps
    // positions[a][t] is the position of agent a at time t in this solution
    std::vector<std::vector<int>> positions;

    // true if this solution contains a path
    bool foundPath;

    // Id(s) of the agent(s) in this solution
    std::vector<int> agentIds;
};


#endif //TFE_MAPF_SOLUTION_H
