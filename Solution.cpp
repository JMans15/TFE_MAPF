//
// Created by Arthur Mahy on 23/11/2022.
//

#include "Solution.h"

#include <utility>
#include <iostream>

Solution::Solution() {
    foundpath = false;
}

Solution::Solution(int m_cost, int m_numberOfVisitedStates,
                   int m_numberOfTimesteps, vector<vector<int>> m_positionsAtTime) {
    foundpath = true;
    cost = m_cost;
    numberOfVisitedStates = m_numberOfVisitedStates;
    numberOfTimesteps = m_numberOfTimesteps;
    positionsAtTime = std::move(m_positionsAtTime);
}

int Solution::getCost() const {
    return cost;
}

int Solution::getNumberOfVisitedStates() const {
    return numberOfVisitedStates;
}

int Solution::getNumberOfTimesteps() const {
    return numberOfTimesteps;
}

vector<int> Solution::getPositionsAtTime(int t) {
    return positionsAtTime[t];
}

vector<int> Solution::getPathOfAgent(int i) {
    vector<int> path;
    for (int t = 0; t < numberOfTimesteps; t++){
        path.push_back(positionsAtTime[t][i]);
    }
    return path;
}

void Solution::print() {
    if (foundpath){
        cout << " " << endl;
        cout << "A solution has been found!" << endl;
        cout << numberOfVisitedStates << " visited states (goal tested)" << endl;
        cout << "Cost of the solution = " << cost << endl;
        cout << " " << endl;
        cout << " -> Position of every agent at each time : " << endl;
        for (int t = 0; t < numberOfTimesteps; t++){
            cout << " -- Time " << t << " : " << endl;
            for (int i = 0; i < positionsAtTime[t].size(); i++){
                cout << " --- Agent " << i << " : position " << positionsAtTime[t][i] << endl;
            }
        }
        cout << " " << endl;
        cout << " -> Path of each agent : " << endl;
        for (int i = 0; i < positionsAtTime[0].size(); i++){
            cout << " -- Agent " << i << " : " << endl;
            for (int t = 0; t < numberOfTimesteps; t++){
                cout << " --- Time " << t << " : position " << positionsAtTime[t][i] << endl;
            }
        }
    }
}
