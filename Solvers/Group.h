//
// Created by Arthur Mahy on 27/03/2023.
//

#ifndef TFE_MAPF_GROUP_H
#define TFE_MAPF_GROUP_H

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


#endif //TFE_MAPF_GROUP_H
