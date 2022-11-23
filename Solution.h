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
    Solution(vector<string> m_stringPath, int m_cost, string m_obj_function, int m_numberOfVisitedStates, int m_numberOfTimesteps, vector<vector<int>> m_positionsAtTime);
    vector<string> getStringPath();
    int getCost() const;
    int getNumberOfVisitedStates() const;
    int getNumberOfTimesteps() const;
    vector<int> getPositionsAtTime(int t);
    vector<int> getPathOfAgent(int i);
    void print();

private:
    bool foundpath;
    vector<string> stringPath;
    int cost; // cost of the solution
    string obj_function;
    int numberOfVisitedStates;
    int numberOfTimesteps;
    vector<vector<int>> positionsAtTime;
};


#endif //TFE_MAPF_SOLUTION_H
