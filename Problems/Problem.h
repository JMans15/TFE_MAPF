//! A generic problem class

#ifndef TFE_MAPF_PROBLEM_H
#define TFE_MAPF_PROBLEM_H

#include "../Constraints/ConstraintsSet.h"
#include "../GraphParser/Graph.h"

#include <set>
#include <unordered_map>

// #define DEBUG

#ifdef DEBUG
#define LOG(str) std::cout << str << std::endl;
#else
#define LOG(str)
#endif

//! The different types of objective functions
enum ObjectiveFunction {
  Fuel,
  Makespan,
  SumOfCosts,
};

class Problem {
public:
  virtual bool
  hasExternalConstraints() const = 0; /**< @return Whether the problem has
                                         external constraints or not. */
  virtual int getNumberOfAgents()
      const = 0; /**< @return the number of agents in the problem */
  virtual bool isImpossible()
      const = 0; /**< @return Whether the problem is impossible or not */
};

#endif // TFE_MAPF_PROBLEM_H
