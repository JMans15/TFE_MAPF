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
    Solution(int m_cost, string m_obj_function, int m_numberOfVisitedStates, int m_numberOfTimesteps, vector<vector<int>> m_positionsAtTime);
    int getCost() const;
    int getNumberOfVisitedStates() const;
    int getNumberOfTimesteps() const;
    vector<int> getPositionsAtTime(int t);
    vector<int> getPathOfAgent(int i);
    void print();

private:

    // Cost of the solution
    int cost;

    // Objective function : "Fuel", "Makespan" or "SumOfCosts"
    string obj_function;

    // The number of visited (goal tested) states.
    int numberOfVisitedStates;

    // The number of needed timesteps from all agents to reach their target position
    int numberOfTimesteps;

    // Matrix of size numberOfTimesteps X numberOfAgents
    // positionsAtTime[t][a] is the position of agent a at time t in this solution
    vector<vector<int>> positionsAtTime;

    // true if this solution contains a path
    bool foundpath;
};


#endif //TFE_MAPF_SOLUTION_H
