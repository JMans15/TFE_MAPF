#ifndef TFE_MAPF_HEURISTICOPTIMAL_H
#define TFE_MAPF_HEURISTICOPTIMAL_H

#include "HeuristicManhattan.h"
#include "ReverseResumableAStar.h"

// Optimal distance heuristic
// - for single agent problem
// - only interesting for Space Time A*
// A reverse resumable A* search will be run in addition to the search which this heuristic is used for.
template<typename S, typename std::enable_if<std::is_base_of<SingleAgentState, S>::value or std::is_base_of<SingleAgentSpaceTimeState, S>::value>::type* = nullptr>
class OptimalDistanceHeuristic : public Heuristic<S> {
public:
    OptimalDistanceHeuristic(int start, int target, std::shared_ptr<Graph> graph)
        : rraStarSearch(std::make_shared<ReverseResumableAStar>(std::make_shared<SingleAgentProblem>(graph, start, target))) {}

    int heuristicFunction(std::shared_ptr<S> state) override {
        return rraStarSearch->optimalDistance(state->getPosition());
    }
private:
    std::shared_ptr<ReverseResumableAStar> rraStarSearch;
};

// Sum of Individual Optimal Costs heuristic
// where cost is the optimal distance (ignoring other agents)
// // A reverse resumable A* search per agent will be run in addition to the search which this heuristic is used for.
// - for SumOfCosts and Fuel objective functions
// - for multi agent problem
template<typename S, typename std::enable_if<std::is_base_of<MultiAgentState, S>::value>::type* = nullptr>
class SIOCheuristic : public Heuristic<S> {
public:
    SIOCheuristic(std::vector<int> starts, std::vector<int> targets, std::shared_ptr<Graph> graph) {
        for (int i = 0; i < starts.size(); i++){
            auto prob = std::make_shared<SingleAgentProblem>(graph, starts[i], targets[i]);
            rraStarSearches.push_back(std::make_shared<ReverseResumableAStar>(prob));
        }
    }

    int heuristicFunction(std::shared_ptr<S> state) {
        int sum = 0;
        auto positions = state->getPositions();
        for (int i = 0; i < positions.size(); i++){
            sum += rraStarSearches[i]->optimalDistance(positions[i]);
        }
        return sum;
    }
private:
    std::vector<std::shared_ptr<ReverseResumableAStar>> rraStarSearches;
};

// Maximum Individual Optimal Cost heuristic
// where cost is the optimal distance (ignoring other agents)
// A reverse resumable A* search per agent will be run in addition to the search which this heuristic is used for.
// - for Makespan objective function
// - for multi agent problem
template<typename S, typename std::enable_if<std::is_base_of<MultiAgentState, S>::value>::type* = nullptr>
class MIOCheuristic : public Heuristic<S> {
public:
    MIOCheuristic(std::vector<int> starts, std::vector<int> targets, std::shared_ptr<Graph> graph) {
        for (int i = 0; i < starts.size(); i++){
            auto prob = std::make_shared<SingleAgentProblem>(graph, starts[i], targets[i]);
            rraStarSearches.push_back(std::make_shared<ReverseResumableAStar>(prob));
        }
    }

    int heuristicFunction(std::shared_ptr<S> state) {
        int maxDistance = 0;
        auto positions = state->getPositions();
        for (int i = 0; i < state->getAgentToAssign(); i++){
            maxDistance = std::max(maxDistance, rraStarSearches[i]->optimalDistance(positions[i]));
        }
        for (int i = state->getAgentToAssign(); i < positions.size(); i++){
            maxDistance = std::max(maxDistance, rraStarSearches[i]->optimalDistance(positions[i]) - 1);
        }
        return maxDistance;
    }
private:
    std::vector<std::shared_ptr<ReverseResumableAStar>> rraStarSearches;
};

template <class P, class S,
    typename std::enable_if<std::is_base_of<Problem<S>, P>::value>::type* = nullptr,
    typename std::enable_if<std::is_base_of<SingleAgentState, S>::value>::type* = nullptr
>
std::shared_ptr<Heuristic<S>> getHeuristic(std::shared_ptr<P> problem, std::shared_ptr<TypeOfHeuristic> type) {
    auto pb = std::dynamic_pointer_cast<SingleAgentProblem>(problem);
    if (*type == Manhattan){
        LOG("The used heuristic will be the Manhattan distance.");
        return std::make_shared<ManhattanHeuristic<S>>(pb->getTarget(), pb->getGraph()->getWidth());
    } else if (*type == OptimalDistance){
        LOG("The used heuristic will be the optimal distance.");
        LOG("This heuristic is only interesting for Space Time A*.");
        LOG("A reverse resumable A* search will be run in addition to this search.");
        return std::make_shared<OptimalDistanceHeuristic<S>>(pb->getStart(), pb->getTarget(), pb->getGraph());
    } else {
        LOG("Not a valid heuristic.");
        return nullptr;
    }
}

template <class P, class S,
        typename std::enable_if<std::is_base_of<Problem<S>, P>::value>::type* = nullptr,
        typename std::enable_if<std::is_base_of<SingleAgentSpaceTimeState, S>::value>::type* = nullptr
>
std::shared_ptr<Heuristic<S>> getHeuristic(std::shared_ptr<P> problem, std::shared_ptr<TypeOfHeuristic> type) {
    auto pb = std::dynamic_pointer_cast<SingleAgentSpaceTimeProblem>(problem);
    if (*type == Manhattan){
        LOG("The used heuristic will be the Manhattan distance.");
        return std::make_shared<ManhattanHeuristic<S>>(pb->getTarget(), pb->getGraph()->getWidth());
    } else if (*type == OptimalDistance){
        LOG("The used heuristic will be the optimal distance.");
        LOG("This heuristic is only interesting for Space Time A*.");
        LOG("A reverse resumable A* search will be run in addition to this search.");
        return std::make_shared<OptimalDistanceHeuristic<S>>(pb->getStart(), pb->getTarget(), pb->getGraph());
    } else {
        LOG("Not a valid heuristic.");
        return nullptr;
    }
}

template <class P, class S,
    typename std::enable_if<std::is_base_of<Problem<S>, P>::value>::type* = nullptr,
    typename std::enable_if<std::is_base_of<MultiAgentState, S>::value>::type* = nullptr
>
std::shared_ptr<Heuristic<S>> getHeuristic(std::shared_ptr<P> problem, std::shared_ptr<TypeOfHeuristic> type) {
    auto pb = std::dynamic_pointer_cast<MultiAgentProblem>(problem);
    if (*type == Manhattan){
        if (pb->getObjFunction() == Makespan){
            LOG("The used heuristic will be the Maximum Individual Cost (Manhattan distance).");
            return std::make_shared<MICheuristic<S>>(pb->getTargets(), pb->getGraph()->getWidth());
        } else {
            LOG("The used heuristic will be the Sum Of Individual Costs (Manhattan distance).");
            return std::make_shared<SICheuristic<S>>(pb->getTargets(), pb->getGraph()->getWidth());
        }
    } else if (*type == OptimalDistance){
        if (pb->getObjFunction() == Makespan){
            LOG("The used heuristic will be the Maximum Individual Optimal Cost (optimal distance).");
            LOG("A reverse resumable A* search per agent will be run in addition to this search.");
            return std::make_shared<MIOCheuristic<S>>(pb->getStarts(), pb->getTargets(), pb->getGraph());
        } else {
            LOG("The used heuristic will be the Sum Of Individual Optimal Costs (optimal distance).");
            LOG("A reverse resumable A* search per agent will be run in addition to this search.");
            return std::make_shared<SIOCheuristic<S>>(pb->getStarts(), pb->getTargets(), pb->getGraph());
        }
    } else {
        LOG("Not a valid heuristic.");
        return nullptr;
    }
}

#endif//TFE_MAPF_HEURISTICOPTIMAL_H