//
// Created by Arthur Mahy on 27/02/2023.
//

#include "ReverseResumableAStar.h"

#ifdef DEBUG
#define LOG(str) cout << str << endl;
#else
#define LOG(str)
#endif

ReverseResumableAStar::ReverseResumableAStar(SingleAgentProblem *m_problem) : problem(m_problem) {
    LOG("===== Reverse Resumable A* Search ====");
    heuristic = new Manhattanheuristic(problem->getStart(), problem->getGraph().getWidth());
    shared_ptr<State> g = problem->getGoalState();
    shared_ptr<State> s = problem->getStartState();
    fringe.emplace(0+heuristic->heuristicFunction(g), Node(g));
    Resume(s);
}

int ReverseResumableAStar::Resume(const shared_ptr<State>& state) {
    while (!fringe.empty()){
        Tuple tuplee = fringe.top();
        Node node = get<1>(tuplee);
        fringe.pop();
        auto nodestate = node.getState();
        if (explored.count(nodestate)>0) { // if node.getState() is already in explored
            continue;
        }
        explored.insert({nodestate, node.getGn()});
        vector<Double> successors = problem->getSuccessors(node.getState());
        for (auto & successor : successors){
            shared_ptr<State> child = get<0>(successor);
            int cost = get<1>(successor);
            Node newnode(child, node, node.getGn()+cost);
            if (explored.count(child)==0) { // if child is not in explored
                fringe.emplace(newnode.getGn()+heuristic->heuristicFunction(child),newnode);
            }
        }
        if (*nodestate==*state){
            return node.getGn();
        }
    }
    return -1;
}

int ReverseResumableAStar::OptimalDistance(int position) {
    auto state = make_shared<SingleAgentState>(position,0);
    if (explored.count(state)>0) { // if state is already in explored
        LOG("position is already explored")
        return explored.at(state);
    }
    LOG("position is not yet explored, we'll continue the RRA* search to visit position")
    int cost = Resume(state);
    if (cost==-1){
        return INT_MAX;
    }
    return cost;
}

myUMap ReverseResumableAStar::getExplored() {
    return explored;
}
