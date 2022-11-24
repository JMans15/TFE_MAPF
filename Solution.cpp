//
// Created by Arthur Mahy on 23/11/2022.
//

#include "Solution.h"

#include <utility>
#include <iostream>

Solution::Solution() {
    foundpath = false;
}

Solution::Solution(vector<string> m_stringPath, int m_cost, string m_obj_function, int m_numberOfVisitedStates,
                   int m_numberOfTimesteps, vector<vector<int>> m_positionsAtTime) {
    foundpath = true;
    stringPath = std::move(m_stringPath);
    cost = m_cost;
    obj_function = std::move(m_obj_function);
    numberOfVisitedStates = m_numberOfVisitedStates;
    numberOfTimesteps = m_numberOfTimesteps;
    positionsAtTime = std::move(m_positionsAtTime);
}

vector<string> Solution::getStringPath() {
    return stringPath;
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
    cout << " " << endl;
    cout << "A solution has been found!" << endl;
    cout << numberOfVisitedStates << " visited states (goal tested)" << endl;
    cout << "Cost of the solution = " << cost << " (" << obj_function << " cost)" << endl;
    cout << " " << endl;
    cout << " -> Solution explained in English : " << endl;
    for (const string& action: stringPath){
        cout << action << endl;
    }
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