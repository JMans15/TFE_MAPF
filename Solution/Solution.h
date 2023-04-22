//
// Created by Arthur Mahy on 23/11/2022.
//

#ifndef TFE_MAPF_SOLUTION_H
#define TFE_MAPF_SOLUTION_H

#include <string>
#include <vector>
#include <unordered_map>

class Solution {
public:
    Solution();
    Solution(int numberOfTimesteps, std::unordered_map<int, std::vector<int>> positions);
    Solution(int cost, int numberOfVisitedNodes, int numberOfTimesteps, std::unordered_map<int, std::vector<int>> positions);

    int getCost() const;
    int getNumberOfVisitedNodes() const;
    int getNumberOfTimesteps() const;
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

    // Returns true if all paths don't have any conflict (vertex conflict and edge conflict)
    bool isValid();

    std::unordered_map<int, std::vector<int>> getPositions() const;

    // Lengthens the matrix positions so that the length of a path is length
    void lengthenPositions(int length);

private:

    // Cost of the solution / value of the objective function for this solution (if solution of a single joint A* or conflict based search)
    int cost;

    // The number of visited nodes
    // - goal tested states if solution of a single joint A*
    // - conflict tree nodes if solution of conflict based search
    int numberOfVisitedNodes;

    // The number of needed timesteps from all agents to reach their target position
    int numberOfTimesteps;

    // Map with the paths
    // The key is the id of an agent and the value is the path of the agent
    // positions[a][t] is the position of agent a at time t in this solution
    std::unordered_map<int, std::vector<int>> positions;

    // true if this solution contains a path
    bool foundPath;
};


#endif //TFE_MAPF_SOLUTION_H
