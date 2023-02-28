//
// Created by Arthur Mahy on 27/02/2023.
//

#include "AStar.h"
#include "MultiAgentProblem.h"

#ifdef DEBUG
#define LOG(str) cout << str << endl;
#else
#define LOG(str)
#endif

typedef tuple<int, Node> Tuple;

struct StatePtrHashh {
    size_t operator()(const shared_ptr<State>& ptr) const {
        return ptr->hash();
    }
};

struct StatePtrEquall {
    size_t operator()(const shared_ptr<State>& ptr, const shared_ptr<State>& other) const {
        return *ptr == *other;
    }
};

typedef unordered_set<shared_ptr<State>, StatePtrHashh, StatePtrEquall> myUSet;

template <typename T>
void addToSet(myUSet & set, shared_ptr<T>* state) {
    auto newstate = dynamic_pointer_cast<State>(*state);
    set.insert(newstate);
}

template <typename T>
bool setContains(myUSet & set, shared_ptr<T>* state) {
    return set.count(*state) > 0;
}

struct CompareFF {
    bool operator()(const Tuple& lhs, const Tuple& rhs) const
    {
        return get<0>(lhs) > get<0>(rhs);
    }
};

Solution retrieveSolution(int numberOfVisitedStates, Node node) {
    vector<vector<int>> positionsAtTime;
    int cost = node.getGn();
    shared_ptr<Node> currentnode = std::make_unique<Node>(node);
    int numberOfTimesteps = node.getState()->getTimestep();
    int oldT = numberOfTimesteps+1;
    while (currentnode->getParent() != nullptr){
        if (oldT!=currentnode->getState()->getTimestep()){
            positionsAtTime.push_back(currentnode->getState()->getPositions());
        }
        oldT = currentnode->getState()->getTimestep();
        currentnode = std::move(currentnode->getParent());
    }
    positionsAtTime.push_back(currentnode->getState()->getPositions());
    reverse(positionsAtTime.begin(), positionsAtTime.end());
    return {cost, numberOfVisitedStates, numberOfTimesteps+1, positionsAtTime};
}

Heuristic* chooseHeuristic(Problem* problem, TypeOfHeuristic typeOfHeuristic){
    if (problem->getNumberOfAgents()!=1){
        auto* prob = dynamic_cast<MultiAgentProblem*>(problem);
        if (typeOfHeuristic==Manhattan){
            if (prob->getObjFunction()==Makespan){
                LOG("The used heuristic will be the Maximum Individual Cost (Manhattan distance).");
                return new MICheuristic(problem->getTargets(), problem->getGraph().getWidth());
            } else { // prob->getObjFunction()==SumOfCosts or prob->getObjFunction()==Fuel
                LOG("The used heuristic will be the Sum Of Individual Costs (Manhattan distance).");
                return new SICheuristic(problem->getTargets(), problem->getGraph().getWidth());
            }
        } else if (typeOfHeuristic==OptimalDistance){
            if (prob->getObjFunction()==Makespan){
                LOG("The used heuristic will be the Maximum Individual Optimal Cost (optimal distance).");
                LOG("A reverse resumable A* search per agent will be run in addition to this search.");
                return new MIOCheuristic(problem->getStarts(), problem->getTargets(), problem->getGraph());
            } else { // prob->getObjFunction()==SumOfCosts or prob->getObjFunction()==Fuel
                LOG("The used heuristic will be the Sum Of Individual Optimal Costs (optimal distance).");
                LOG("A reverse resumable A* search per agent will be run in addition to this search.");
                return new SIOCheuristic(problem->getStarts(), problem->getTargets(), problem->getGraph());
            }
        } else {
            LOG("Not a valid heuristic.");
            return nullptr;
        }
    } else { // problem->getNumberOfAgents()==1
        if (typeOfHeuristic==Manhattan){
            LOG("The used heuristic will be the Manhattan distance.");
            return new Manhattanheuristic(problem->getTargets()[0], problem->getGraph().getWidth());
        } else if (typeOfHeuristic==OptimalDistance){
            LOG("The used heuristic will be the optimal distance.");
            LOG("This heuristic is only interesting for Space Time A*.");
            LOG("A reverse resumable A* search will be run in addition to this search.");
            return new OptimalDistanceheuristic(problem->getStarts()[0], problem->getTargets()[0], problem->getGraph());
        } else {
            LOG("Not a valid heuristic.");
            return nullptr;
        }
    }
}

AStar::AStar(Problem *problem, TypeOfHeuristic typeOfHeuristic) {
    LOG("===== A* Search ====");
    Heuristic* heuristic = chooseHeuristic(problem, typeOfHeuristic);
    if (heuristic == nullptr){
        solution = {};
        return;
    }
    LOG("Beginning the A* search. ");
    shared_ptr<State> s = problem->getStartState();
    priority_queue<Tuple, vector<Tuple>, CompareFF> fringe;
    fringe.emplace(0+heuristic->heuristicFunction(s), Node(s));
    myUSet explored; // the closed list
    int numberOfVisitedStates = 0;
    while (!fringe.empty()){
        Tuple tuplee = fringe.top();
        Node node = get<1>(tuplee);
        fringe.pop();
        auto nodestate = node.getState();
        if (setContains(explored, &nodestate)) { // if node.getState() is already in explored
            continue;
        }
        numberOfVisitedStates += 1;
        addToSet(explored, &nodestate);
        if (problem->isGoalState(node.getState())){
            delete heuristic;
            solution = retrieveSolution(numberOfVisitedStates, node);
            return;
        }
        vector<Double> successors = problem->getSuccessors(node.getState());
        for (auto & successor : successors){
            shared_ptr<State> child = get<0>(successor);
            int cost = get<1>(successor);
            Node newnode(child, node, node.getGn()+cost);
            if (!setContains(explored, &child)) { // if child is not in explored
                fringe.emplace(newnode.getGn()+heuristic->heuristicFunction(child),newnode);
            }
        }

    }
    LOG("No path has been found.");
    delete heuristic;
    solution = {};
}

Solution AStar::getSolution() {
    return solution;
}
