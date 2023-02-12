#include "library.h"

#include <iostream>
#include <queue>
#include <tuple>
#include <set>
#include <algorithm>

#define LOG(str) if (verbose) {cout << str << endl;}

typedef tuple<int, Node> Tuple;

struct CompareF
{
    bool operator()(const Tuple& lhs, const Tuple& rhs) const
    {
        return get<0>(lhs) > get<0>(rhs);
    }
};

Solution retrieveSolution(int numberOfVisitedStates, Node node) {
    vector<vector<int>> positionsAtTime;
    int cost = node.getGn();
    Node* currentnode = &node;
    int numberOfTimesteps = node.getState()->getTimestep();
    int oldT = numberOfTimesteps+1;
    while (currentnode->getParent() != nullptr){
        if (oldT!=currentnode->getState()->getTimestep()){
            positionsAtTime.push_back(currentnode->getState()->getPositions());
        }
        oldT = currentnode->getState()->getTimestep();
        currentnode = currentnode->getParent();
    }
    positionsAtTime.push_back(currentnode->getState()->getPositions());
    reverse(positionsAtTime.begin(), positionsAtTime.end());
    return {cost, numberOfVisitedStates, numberOfTimesteps+1, positionsAtTime};
}

Solution aStarSearch(Problem* problem, TypeOfHeuristic typeOfHeuristic, int verbose){
    LOG("===== Search ====");
    Heuristic* heuristic;
    if (typeOfHeuristic==MIC){
        LOG("The used heuristic will be the Maximum Individual Cost (Manhattan distance).");
        heuristic = new MICheuristic(problem->getTargets(), problem->getGraph().getWidth());
    } else if (typeOfHeuristic==SIC){
        LOG("The used heuristic will be the Sum Of Individual Costs (Manhattan distance).");
        heuristic = new SICheuristic(problem->getTargets(), problem->getGraph().getWidth());
    } else { // typeOfHeuristic==Manhattan
        if (problem->getNumberOfAgents()!=1){
            LOG("We cannot use this heuristic with a single agent problem.");
            return {};
        }
        LOG("The used heuristic will be the Manhattan distance.");
        heuristic = new Manhattanheuristic(problem->getTargets()[0], problem->getGraph().getWidth());
    }
    LOG("Beginning the A* search. ");
    State* s = problem->getStartState();
    priority_queue<Tuple, vector<Tuple>, CompareF> fringe;
    fringe.emplace(0+heuristic->heuristicFunction(s), Node(s));
    set<State*> explored;  // the closed list - THERE'S A PROBLEM HERE NEW
    int numberOfVisitedStates = 0;
    while (!fringe.empty()){
        Tuple tuplee = fringe.top();
        Node node = get<1>(tuplee);
        fringe.pop();

        if (std::count_if(explored.begin(), explored.end(), [&node](State* state) {return *state == *node.getState();})) { // if node.getState() is already in explored
            continue;
        }
        numberOfVisitedStates += 1;
        if (problem->isGoalState(node.getState())){
            return retrieveSolution(numberOfVisitedStates, node);
        }
        explored.insert(node.getState());
        vector<Double> successors = problem->getSuccessors(node.getState());
        for (auto & successor : successors){
            State* child = get<0>(successor);
            int cost = get<1>(successor);
            Node newnode(child, node, node.getGn()+cost);
            if (!std::count_if(explored.begin(), explored.end(), [&child](State* state) {return *state == *child;})){
                fringe.emplace(newnode.getGn()+heuristic->heuristicFunction(child),newnode);
            }
        }
    }
    LOG("No path has been found.");
    return {};
}
