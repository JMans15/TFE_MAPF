//
// Created by Arthur Mahy on 23/11/2022.
//

#include "Solution.h"

#include <utility>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "set"

Solution::Solution() {
    foundPath = false;
}

Solution::Solution(int numberOfTimesteps, std::unordered_map<int, std::vector<int>> positions)
: foundPath(true)
, cost(-1)
, numberOfVisitedNodes(-1)
, numberOfTimesteps(numberOfTimesteps)
, positions(positions)
{
    //if (not isValid()){
    //    std::cout << "This solution is not valid ! There are conflicts between the paths. " << std::endl;
    //}
}

Solution::Solution(int cost, int numberOfVisitedNodes,
                   int numberOfTimesteps, std::unordered_map<int, std::vector<int>> positions)
    : foundPath(true)
    , cost(cost)
    , numberOfVisitedNodes(numberOfVisitedNodes)
    , numberOfTimesteps(numberOfTimesteps)
    , positions(positions)
{
    //if (not isValid()){
    //    std::cout << "This solution is not valid ! There are conflicts between the paths. " << std::endl;
    //}
}

int Solution::getCost() const {
    return cost;
}

int Solution::getNumberOfVisitedNodes() const {
    return numberOfVisitedNodes;
}

int Solution::getNumberOfTimesteps() const {
    return numberOfTimesteps;
}

std::vector<int> Solution::getPathOfAgent(int id) {
    return positions[id];
}

void Solution::write(std::string filename, int width) {
    std::ofstream file(filename.c_str());
    if (foundPath) {
        int A = static_cast<int>(positions.size());
        int T = numberOfTimesteps;
        file << A << " " << T << std::endl;
        int x, y, p;
        for (int t = 0; t < T; t++) {
            for (auto a : positions) {
                p = a.second[t];
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
        std::cout << numberOfVisitedNodes << " visited nodes " << std::endl;
        std::cout << " - goal tested states if solution of a single joint A*" << std::endl;
        std::cout << " - conflict tree nodes if solution of conflict based search" << std::endl;
        std::cout << "Cost of the solution = " << cost << " (value of the objective function for this solution)" << std::endl;
        std::cout << " " << std::endl;
        std::cout << " -> Position of every agent at each time : " << std::endl;
        for (int t = 0; t < positions.begin()->second.size(); t++){
            std::cout << " -- Time " << t << " : " << std::endl;
            for (auto i : positions){
                std::cout << " --- Agent " << i.first << " : position " << i.second[t] << std::endl;
            }
        }
        std::cout << " " << std::endl;
        std::cout << " -> Path of each agent : " << std::endl;
        for (auto i : positions){
            std::cout << " -- Agent " << i.first << " : " << std::endl;
            for (int t = 0; t < i.second.size(); t++){
                std::cout << " --- Time " << t << " : position " << i.second[t] << std::endl;
            }
        }
    }
}



bool Solution::getFoundPath() {
    return foundPath;
}

int Solution::getFuelCost() {
    int cost = 0;
    for (auto a : positions) {
        for (int t = 1; t < numberOfTimesteps; t++) {
            if (a.second[t] != a.second[t-1]) {
                cost += 1;
            }
        }
    }
    return cost;
}

int Solution::getMakespanCost() {
    return numberOfTimesteps - 1;
}

int Solution::getSumOfCostsCost() {
    int cost = 0;
    for (auto a : positions) {
        for (int t = numberOfTimesteps-1; t >= 0; t--) {
            if (a.second[t] != a.second[numberOfTimesteps-1]) {
                cost += t+1;
                break;
            }
        }
    }
    return cost;
}

bool Solution::isValid() {
    // Vertex conflict
    for (int t = 0; t < numberOfTimesteps; t++){
        std::set<int> positionsAtThisTimestep;
        for (auto i : positions){
             if (positionsAtThisTimestep.count(i.second[t])>0){
                 return false;
             } else {
                 positionsAtThisTimestep.insert(i.second[t]);
             }
        }
    }
    // Edge conflict
    for (int t = 0; t < numberOfTimesteps-1; t++){
        std::set<std::pair<int, int>> edgesAtThisTimestep;
        for (auto i : positions){
            if (edgesAtThisTimestep.count({i.second[t], i.second[t + 1]})>0){
                return false;
            } else {
                edgesAtThisTimestep.insert({i.second[t + 1], i.second[t]});
            }
        }
    }
    return true;
}

std::unordered_map<int, std::vector<int>> Solution::getPositions() const {
    return positions;
}

void Solution::lengthenPositions(int length) {
    int actualLength = positions.begin()->second.size();
    if (actualLength < length){
        for (auto i : positions){
            positions[i.first].resize(length, i.second[actualLength-1]);
        }
    }
}
