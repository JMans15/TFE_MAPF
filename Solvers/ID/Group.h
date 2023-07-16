//
// Created by Arthur Mahy on 27/03/2023.
//

#ifndef TFE_MAPF_GROUP_H
#define TFE_MAPF_GROUP_H

#include "../../Solution/Solution.h"
#include <boost/functional/hash.hpp>
#include <utility>

class Group {
public:
  Group() = default;
  explicit Group(std::set<int> agents) : agents(std::move(agents)) {}

  std::set<int> getAgents() { return agents; }

  void putSolution(std::shared_ptr<Solution> m_solution) {
    solution = std::move(m_solution);
  }

  std::shared_ptr<Solution> getSolution() { return solution; }

  bool operator!=(Group other) const { return agents != other.getAgents(); }

  bool operator==(Group other) const { return agents == other.getAgents(); }

  std::size_t getHash() const {
    std::size_t hash_value = agents.size();
    for (int agent : agents) {
      hash_value ^= std::hash<int>()(agent) + 0x9e3779b9 + (hash_value << 6) +
                    (hash_value >> 2);
    }
    return hash_value;
  }

private:
  std::set<int> agents;
  std::shared_ptr<Solution> solution;
};

struct GroupHasher {
  std::size_t operator()(const std::shared_ptr<Group> &group) const {
    return group->getHash();
  }
};

struct GroupEquality {
  bool operator()(const std::shared_ptr<Group> &a,
                  const std::shared_ptr<Group> &b) const {
    return *a == *b;
  }
};

#endif // TFE_MAPF_GROUP_H
