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
  Group() = default; /**< Default constructor */
  explicit Group(std::set<int> agents)
      : agents(std::move(agents)) {
  } /**< Constructor taking agents as argument */

  std::set<int> getAgents() {
    /**
     * Getter for the set of agents the group contains (as a set of their IDs)
     */
    return agents;
  }

  void putSolution(std::shared_ptr<Solution> m_solution) {
    /**
     * Setter for the current solution of the travel group
     */
    solution = std::move(m_solution);
  }

  std::shared_ptr<Solution> getSolution() {
    /**
     * Getter for the current solution of the travel group
     */
    return solution;
  }

  bool operator!=(Group other) const {
    /**
     * Defines the inequality between groups as the inequality between the set
     * of agents they contain
     */
    return agents != other.getAgents();
  }

  bool operator==(Group other) const {
    /**
     * Defines the equality between groups as the equality between the set
     * of agents they contain
     */
    return agents == other.getAgents();
  }

  std::size_t getHash() const {
    /**
     * Returns a hash of the groups computed from the agents and not the
     * solution.
     */
    std::size_t hash_value = agents.size();
    for (int agent : agents) {
      hash_value ^= std::hash<int>()(agent) + 0x9e3779b9 + (hash_value << 6) +
                    (hash_value >> 2);
    }
    return hash_value;
  }

private:
  std::set<int> agents; /*!< The set of agents contained in the group */
  std::shared_ptr<Solution> solution; /*!< The current solution for the group */
};

#ifndef DOXYGEN_SKIP
struct GroupHasher {
  std::size_t operator()(const std::shared_ptr<Group> &group) const {
    return group->getHash();
  }
};

struct GroupEquality {
  /*
   *  Defines equality between smart pointers to groups as the equality
   * between the pointed groups
   */
  bool operator()(const std::shared_ptr<Group> &a,
                  const std::shared_ptr<Group> &b) const {
    return *a == *b;
  }
};
#endif

#endif // TFE_MAPF_GROUP_H
