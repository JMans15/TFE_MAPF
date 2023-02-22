//
// Created by Arthur Mahy on 23/11/2022.
//

#ifndef TFE_MAPF_SOLUTION_H
#define TFE_MAPF_SOLUTION_H
#include <string>
#include <vector>
using namespace std;

class Solution {
public:
    Solution();
    Solution(int m_cost, int m_numberOfVisitedStates, int m_numberOfTimesteps, vector<vector<int>> m_positionsAtTime);

    int getCost() const;
    int getNumberOfVisitedStates() const;
    int getNumberOfTimesteps() const;
    vector<int> getPositionsAtTime(int t);
    vector<int> getPathOfAgent(int i);
    bool getFoundPath();
    void print();
    void write(string filename, int w);

    // Fuel : Total amount of distance traveled by all agents
    int getFuelCost();

    // Makespan : Total time for the last agent to reach its goal
    int getMakespanCost();

    // SumOfCosts : The sum of the time steps required for every agent to reach its goal
    int getSumOfCostsCost();

private:

    // Cost of the solution / value of the objective function for this solution
    int cost;

    // The number of visited (goal tested) states.
    int numberOfVisitedStates;

    // The number of needed timesteps from all agents to reach their target position
    int numberOfTimesteps;

    // Matrix of size numberOfTimesteps X numberOfAgents
    // positionsAtTime[t][a] is the position of agent a at time t in this solution
    vector<vector<int>> positionsAtTime;

    // true if this solution contains a path
    bool foundPath;
};


#endif //TFE_MAPF_SOLUTION_H
