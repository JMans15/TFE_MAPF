//
// Created by Arthur Mahy on 23/11/2022.
//

#include "Solution.h"

#include <utility>
#include <iostream>

Solution::Solution() {
    foundPath = false;
}

Solution::Solution(int m_cost, int m_numberOfVisitedStates,
                   int m_numberOfTimesteps, vector<vector<int>> m_positionsAtTime) {
    foundPath = true;
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
    if (foundPath){
        cout << " " << endl;
        cout << "==== Solution ====" << endl;
        cout << numberOfVisitedStates << " visited states (goal tested)" << endl;
        cout << "Cost of the solution = " << cost << " (value of the objective function for this solution)" << endl;
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

bool Solution::getFoundPath() {
    return foundPath;
}

int Solution::getFuelCost() {
    int Cost = 0;
    for (int a = 0; a < positionsAtTime[0].size(); a++){
        for (int t = 1; t < numberOfTimesteps; t++){
            if (positionsAtTime[t][a]!=positionsAtTime[t-1][a]){
                Cost+=1;
            }
        }
    }
    return Cost;
}

int Solution::getMakespanCost() {
    return positionsAtTime.size()-1;
}

int Solution::getSumOfCostsCost() {
    int Cost = 0;
    for (int a = 0; a < positionsAtTime[0].size(); a++){
        for (int t = numberOfTimesteps-1; t >= 0; t--){
            if (positionsAtTime[t][a]!=positionsAtTime[numberOfTimesteps-1][a]){
                Cost += t+1;
                break;
            }
        }
    }
    return Cost;
}
