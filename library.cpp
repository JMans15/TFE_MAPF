#include "library.h"

#include <iostream>
#include <queue>
#include <tuple>
#include <set>
#include <functional>

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
    int numberOfTimesteps = node.getState().getTimestep();
    int oldT = numberOfTimesteps+1;
    while (currentnode->getParent() != nullptr){
        if (oldT!=currentnode->getState().getTimestep()){
            positionsAtTime.push_back(currentnode->getState().getPositions());
        }
        oldT = currentnode->getState().getTimestep();
        currentnode = currentnode->getParent();
    }
    positionsAtTime.push_back(currentnode->getState().getPositions());
    reverse(positionsAtTime.begin(), positionsAtTime.end());
    return {cost, numberOfVisitedStates, numberOfTimesteps+1, positionsAtTime};
}

Solution aStarSearch(const Problem& problem, TypeOfHeuristic typeOfHeuristic){
    cout << "===== Search ====" << endl;
    Heuristic* heuristic;
    MICheuristic mic = MICheuristic(problem.getTargets(), problem.getGraph().getWidth());
    SICheuristic sic = SICheuristic(problem.getTargets(), problem.getGraph().getWidth());
    if (typeOfHeuristic==MIC){
        cout << "The used heuristic will be the Maximum Individual Cost (Manhattan distance)." << endl;
        heuristic = &mic;
    } else {
        cout << "The used heuristic will be the Sum Of Individual Costs (Manhattan distance)." << endl;
        heuristic = &sic;
    }
    cout << "Beginning the A* search. " << endl;
    State s = problem.getStartState();
    priority_queue<Tuple, vector<Tuple>, CompareF> fringe;
    fringe.emplace(0+heuristic->heuristicFunction(s), Node(s));
    set<State> explored;  // the closed list
    int numberOfVisitedStates = 0;
    while (!fringe.empty()){
        Tuple tuplee = fringe.top();
        Node node = get<1>(tuplee);
        fringe.pop();
        if (explored.count(node.getState())){
            continue;
        }
        numberOfVisitedStates += 1;
        if (problem.isGoalState(node.getState())){
            return retrieveSolution(numberOfVisitedStates, node);
        }
        explored.insert(node.getState());
        vector<Double> successors = problem.getSuccessors(node.getState());
        for (auto & successor : successors){
            State child = get<0>(successor);
            int cost = get<1>(successor);
            Node newnode(child, node, node.getGn()+cost);
            if (explored.count(child)==0){
                fringe.emplace(newnode.getGn()+heuristic->heuristicFunction(child),newnode);
            }
        }
    }
    cout << "No path has been found." << endl;
    return {};
}
