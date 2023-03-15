//
// Created by Arthur Mahy on 28/02/2023.
//

#ifndef TFE_MAPF_INDEPENDENTDETECTION_H
#define TFE_MAPF_INDEPENDENTDETECTION_H

#include "MultiAgentProblem.h"
#include "AStar.h"

#include <boost/functional/hash.hpp>

class Group {
public:
    Group(){}
    Group(std::set<int> agents) : agents(agents){}

    std::set<int> getAgents() {
        return agents;
    }

    void putSolution(std::shared_ptr<Solution> m_solution){
        solution = m_solution;
    }

    std::shared_ptr<Solution> getSolution() {
        return solution;
    }
private:
    std::set<int> agents;
    std::shared_ptr<Solution> solution;
};

struct GroupHasher {
    std::size_t operator()(std::shared_ptr<Group> group) const {
        size_t result = 0;
        for (int i : group->getAgents()){
            boost::hash_combine(result, i);
        }
        return result;
    }
};

struct GroupEquality {
    bool operator()(std::shared_ptr<Group> a, std::shared_ptr<Group> b) const {
        return a->getAgents()==b->getAgents();
    }
};

// Independent Detection search
// Only for multi agent problem
//
// typeOfHeuristic is the heuristic for the A* searches
//
// We don't take into account the setOfConstraints attribute of problem
class IndependentDetection {
public:
    IndependentDetection(std::shared_ptr<MultiAgentProblem> problem, TypeOfHeuristic typeOfHeuristic);
    std::shared_ptr<Solution> solve();
private:
    std::shared_ptr<MultiAgentProblem> problem;
    TypeOfHeuristic typeOfHeuristic;
    std::unordered_set<std::shared_ptr<Group>, GroupHasher, GroupEquality> groups;

    // Plans a path for each singleton group
    // Returns true if it found a solution for each agent (false otherwise)
    bool planSingletonGroups();

    // Returns (true, a, b) if there's a conflict between the paths of group a and group b
    // and (false, _, _) otherwise
    std::tuple<bool, std::shared_ptr<Group>, std::shared_ptr<Group>> findAConflict();

    // Merges groupA and groupB and plan a path for the new group
    // Returns true if it found a valid solution for the new group (false otherwise)
    bool mergeGroupsAndPlanNewGroup(std::shared_ptr<Group> groupA, std::shared_ptr<Group> groupB);

    std::shared_ptr<Solution> combineSolutions();
};


#endif //TFE_MAPF_INDEPENDENTDETECTION_H
