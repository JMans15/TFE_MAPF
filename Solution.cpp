//
// Created by Arthur Mahy on 23/11/2022.
//

#include "Solution.h"

#include <utility>
#include <iostream>
#include <fstream>

Solution::Solution() {
    foundPath = false;
}

Solution::Solution(int cost, int numberOfVisitedStates,
                   int numberOfTimesteps, std::vector<std::vector<int>> positions)
    : foundPath(true)
    , cost(cost)
    , numberOfVisitedStates(numberOfVisitedStates)
    , numberOfTimesteps(numberOfTimesteps)
    , positions(positions)
{}

int Solution::getCost() const {
    return cost;
}

int Solution::getNumberOfVisitedStates() const {
    return numberOfVisitedStates;
}

int Solution::getNumberOfTimesteps() const {
    return numberOfTimesteps;
}

std::vector<int> Solution::getPositionsAtTime(int t) {
    std::vector<int> pos;
    for (int i = 0; i < positions.size(); i++){
        pos.push_back(positions[i][t]);
    }
    return pos;
}

std::vector<int> Solution::getPathOfAgent(int i) {
    return positions[i];
}

void Solution::write(std::string filename, int width) {
    std::ofstream file(filename.c_str());
    if (foundPath) {
        int A = static_cast<int>(positions.size());
        int T = numberOfTimesteps;
        file << A << " " << T << std::endl;
        int x, y, p;
        for (int t = 0; t < T; t++) {
            for (int a = 0; a < A; a++) {
                p = positions[a][t];
                // The compiler will likely reuse the result of the division, no need to bother optimizing ourselves
                y = p / width;
                x = p % width;
                file << x << "," << y << std::endl;
            }
        }
    }
    file.close();
}

void Solution::print() {
    if (foundPath){
        std::cout << " " << std::endl;
        std::cout << "==== Solution ====" << std::endl;
        std::cout << numberOfVisitedStates << " visited states (goal tested)" << std::endl;
        std::cout << "Cost of the solution = " << cost << " (value of the objective function for this solution)" << std::endl;
        std::cout << " " << std::endl;
        std::cout << " -> Position of every agent at each time : " << std::endl;
        for (int t = 0; t < numberOfTimesteps; t++){
            std::cout << " -- Time " << t << " : " << std::endl;
            for (int i = 0; i < positions.size(); i++){
                std::cout << " --- Agent " << i << " : position " << positions[i][t] << std::endl;
            }
        }
        std::cout << " " << std::endl;
        std::cout << " -> Path of each agent : " << std::endl;
        for (int i = 0; i < positions.size(); i++){
            std::cout << " -- Agent " << i << " : " << std::endl;
            for (int t = 0; t < numberOfTimesteps; t++){
                std::cout << " --- Time " << t << " : position " << positions[i][t] << std::endl;
            }
        }
    }
}



bool Solution::getFoundPath() {
    return foundPath;
}

int Solution::getFuelCost() {
    int cost = 0;
    for (int a = 0; a < positions.size(); a++) {
        for (int t = 1; t < numberOfTimesteps; t++) {
            if (positions[a][t] != positions[a][t-1]) {
                cost += 1;
            }
        }
    }
    return cost;
}

int Solution::getMakespanCost() {
    return positions[0].size() - 1;
}

int Solution::getSumOfCostsCost() {
    int cost = 0;
    for (int a = 0; a < positions.size(); a++) {
        for (int t = numberOfTimesteps-1; t >= 0; t--) {
            if (positions[a][t] != positions[a][numberOfTimesteps-1]) {
                cost += t+1;
                break;
            }
        }
    }
    return cost;
}
