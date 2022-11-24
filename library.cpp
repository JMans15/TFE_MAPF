#include "library.h"
#include "Node.h"

#include <iostream>
#include <queue>
#include <tuple>
#include <set>
#include <functional>

typedef tuple<int, Node> Tuple;
typedef tuple<State, string, int> Triple;

struct CompareF
{
    bool operator()(const Tuple& lhs, const Tuple& rhs) const
    {
        return get<0>(lhs) > get<0>(rhs);
    }
};

Solution aStarSearch(const Problem& problem){
    cout << "===== Search ====" << endl;
    function<int(State, Problem)> heuristic;
    if (problem.getObjFunction()=="SumOfCosts" or problem.getObjFunction()=="Fuel"){
        cout << "The used heuristic will be the Sum Of Individual Costs." << endl;
        heuristic = Problem::SICheuristic;
    } else { // problem.getObjFunction()=="Makespan"
        cout << "The used heuristic will be the Maximum Individual Cost." << endl;
        heuristic = Problem::MICheuristic;
    }
    cout << "Beginning the A* search. " << endl;
    State s = problem.getStartState();
    priority_queue<Tuple, vector<Tuple>, CompareF> fringe;
    fringe.emplace(0+heuristic(s, problem), Node(s));
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
            return problem.retrieveSolution(numberOfVisitedStates, node);
        }
        explored.insert(node.getState());
        vector<Triple> successors = problem.getSuccessors(node.getState());
        for (auto & successor : successors){
            State child = get<0>(successor);
            string action = get<1>(successor);
            int cost = get<2>(successor);
            Node newnode(child, node, action, node.getGn()+cost);
            if (explored.count(child)==0){
                fringe.emplace(newnode.getGn()+heuristic(child,problem),newnode);
            }
        }
    }
    cout << "No path has been found." << endl;
    return {};
}
