#include "library.h"

#include <iostream>
#include <queue>
#include <tuple>
#include <set>
#include <algorithm>
#include <unordered_set>
#include <memory>

//#define DEBUG

#ifdef DEBUG
#define LOG(str) cout << str << endl;
#else
#define LOG(str)
#endif

typedef tuple<int, Node> Tuple;
typedef tuple<int, int> PositionTimeConstraint;

using namespace std;

struct StatePtrHash {
    size_t operator()(const State* ptr) const {
        return ptr->hash();
    }
};

struct StatePtrEqual {
    size_t operator()(const State* ptr, const State* other) const {
        return *ptr == *other;
    }
};

typedef unordered_set<State*, StatePtrHash, StatePtrEqual> myUSet;

template <typename T>
void addToSet(myUSet & set, T** state) {
    auto* newstate = dynamic_cast<State*>(*state);
    set.insert(newstate);
}

template <typename T>
bool setContains(myUSet & set, T** state) {
    return set.count(*state) > 0;
}

struct CompareF {
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

Solution aStarSearch(Problem* problem, TypeOfHeuristic typeOfHeuristic){
    LOG("===== Search ====");
    Heuristic* heuristic;
    if (typeOfHeuristic==MIC){
        LOG("The used heuristic will be the Maximum Individual Cost (Manhattan distance).");
        heuristic = new MICheuristic(problem->getTargets(), problem->getGraph().getWidth());
    } else if (typeOfHeuristic==SIC){
        LOG("The used heuristic will be the Sum Of Individual Costs (Manhattan distance).");
        heuristic = new SICheuristic(problem->getTargets(), problem->getGraph().getWidth());
    } else if (typeOfHeuristic==SIOC){
        LOG("The used heuristic will be the Sum Of Individual Optimal Costs (optimal distance, single agent A*).");
        heuristic = new SIOCheuristic(problem->getTargets(), problem->getGraph());
    } else if (typeOfHeuristic==MIOC){
        LOG("The used heuristic will be the Maximum Individual Optimal Cost (optimal distance, single agent A*).");
        heuristic = new MIOCheuristic(problem->getTargets(), problem->getGraph());
    } else { // typeOfHeuristic==Manhattan
        if (problem->getNumberOfAgents()!=1){
            LOG("We cannot use this heuristic with a multi agent problem.");
            return {};
        }
        LOG("The used heuristic will be the Manhattan distance.");
        heuristic = new Manhattanheuristic(problem->getTargets()[0], problem->getGraph().getWidth());
    }
    LOG("Beginning the A* search. ");
    State* s = problem->getStartState();
    priority_queue<Tuple, vector<Tuple>, CompareF> fringe;
    fringe.emplace(0+heuristic->heuristicFunction(s), Node(s));
    myUSet explored;
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
        if (problem->isGoalState(node.getState())){
            return retrieveSolution(numberOfVisitedStates, node);
        }
        addToSet(explored, &nodestate);
        vector<Double> successors = problem->getSuccessors(node.getState());
        for (auto & successor : successors){
            State* child = get<0>(successor);
            int cost = get<1>(successor);
            Node newnode(child, node, node.getGn()+cost);
            if (!setContains(explored, &child)) { // if child is not in explored
                fringe.emplace(newnode.getGn()+heuristic->heuristicFunction(child),newnode);
            }
        }
    }
    LOG("No path has been found.");
    return {};
}

Solution cooperativeAStarSearch(MultiAgentProblem* problem, int verbose){
    LOG("===== Cooperative A* Search ====");
    set<PositionTimeConstraint> reservationTable;
    int cost = 0;
    int numberOfVisitedStates = 0;
    int numberOfTimesteps = 0;
    ObjectiveFunction objectiveFunction;
    if (problem->getObjFunction()==Makespan or problem->getObjFunction()==SumOfCosts){
        objectiveFunction = Makespan;
    } else {
        objectiveFunction = Fuel;
    }
    vector<vector<int>> positions;
    LOG("Beginning the (numberOfAgents) single agent A* searches. ");
    for (int a = 0; a < problem->getNumberOfAgents(); a++){

        // Single agent A* search for agent a
        SingleAgentSpaceTimeProblem singleagentproblem = SingleAgentSpaceTimeProblem(problem->getGraph(), problem->getStarts()[a], problem->getTargets()[a], objectiveFunction, reservationTable, a);
        Solution solution = aStarSearch(&singleagentproblem, Manhattan);

        if (solution.getFoundPath()){
            vector<int> pathofagent = solution.getPathOfAgent(0);
            for (int t = 0; t < pathofagent.size(); t++){
                reservationTable.insert(PositionTimeConstraint(pathofagent[t], t));
            }
            if (problem->getObjFunction()==SumOfCosts){
                cost += solution.getSumOfCostsCost();
            } else if (problem->getObjFunction()==Fuel){
                cost += solution.getFuelCost();
            } else {
                cost = max(cost, solution.getMakespanCost());
            }
            numberOfVisitedStates += solution.getNumberOfVisitedStates();
            numberOfTimesteps = max(numberOfTimesteps, solution.getNumberOfTimesteps());
            positions.emplace_back(pathofagent);
        } else {
            LOG("No path has been found for agent " << a);
            LOG("The solution is not valid");
            return {};
        }
    }
    // We put the paths in the right format
    vector<vector<int>> positionsAtTime;
    for (int t = 0; t < numberOfTimesteps; t++){
        vector<int> pos;
        for (int agent = 0; agent < problem->getNumberOfAgents(); agent++){
            if (t >= positions[agent].size()){
                pos.emplace_back(positions[agent][positions[agent].size()-1]);
            } else {
                pos.emplace_back(positions[agent][t]);
            }
        }
        positionsAtTime.emplace_back(pos);
    }
    return {cost, numberOfVisitedStates, numberOfTimesteps, positionsAtTime};
}
