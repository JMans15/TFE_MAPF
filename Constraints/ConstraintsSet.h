//
// Created by Arthur Mahy on 05/06/2023.
//

#ifndef TFE_MAPF_CONSTRAINTSSET_H
#define TFE_MAPF_CONSTRAINTSSET_H

#include "EdgeConstraint.h"
#include "VertexConstraint.h"

#include "iostream"
#include <boost/functional/hash.hpp>
#include <unordered_set>

struct SoftVertexConstraintHash {
  std::size_t operator()(const VertexConstraint &constraint) const {
    size_t result = 0;
    boost::hash_combine(result, constraint.getPosition());
    boost::hash_combine(result, constraint.getTime());
    return result;
  }
};

struct SoftVertexConstraintEqual {
  bool operator()(const VertexConstraint &lhs,
                  const VertexConstraint &rhs) const {
    return lhs.getPosition() == rhs.getPosition() &&
           lhs.getTime() == rhs.getTime();
  }
};

struct SoftEdgeConstraintHash {
  std::size_t operator()(const EdgeConstraint &constraint) const {
    size_t result = 0;
    boost::hash_combine(result, constraint.getPosition1());
    boost::hash_combine(result, constraint.getPosition2());
    boost::hash_combine(result, constraint.getTime());
    return result;
  }
};

struct SoftEdgeConstraintEqual {
  bool operator()(const EdgeConstraint &lhs, const EdgeConstraint &rhs) const {
    return lhs.getPosition1() == rhs.getPosition1() &&
           lhs.getPosition2() == rhs.getPosition2() &&
           lhs.getTime() == rhs.getTime();
  }
};

using SoftVertexConstraintsMultiSet =
    std::unordered_multiset<VertexConstraint, SoftVertexConstraintHash,
                            SoftVertexConstraintEqual>;
using SoftEdgeConstraintsMultiSet =
    std::unordered_multiset<EdgeConstraint, SoftEdgeConstraintHash,
                            SoftEdgeConstraintEqual>;

struct HardVertexConstraintHash {
  std::size_t operator()(const VertexConstraint &constraint) const {
    size_t result = 0;
    boost::hash_combine(result, constraint.getAgent());
    boost::hash_combine(result, constraint.getPosition());
    boost::hash_combine(result, constraint.getTime());
    return result;
  }
};

struct HardVertexConstraintEqual {
  bool operator()(const VertexConstraint &lhs,
                  const VertexConstraint &rhs) const {
    return lhs.getAgent() == rhs.getAgent() &&
           lhs.getPosition() == rhs.getPosition() &&
           lhs.getTime() == rhs.getTime();
  }
};

struct HardEdgeConstraintHash {
  std::size_t operator()(const EdgeConstraint &constraint) const {
    size_t result = 0;
    boost::hash_combine(result, constraint.getAgent());
    boost::hash_combine(result, constraint.getPosition1());
    boost::hash_combine(result, constraint.getPosition2());
    boost::hash_combine(result, constraint.getTime());
    return result;
  }
};

struct HardEdgeConstraintEqual {
  bool operator()(const EdgeConstraint &lhs, const EdgeConstraint &rhs) const {
    return lhs.getAgent() == rhs.getAgent() &&
           lhs.getPosition1() == rhs.getPosition1() &&
           lhs.getPosition2() == rhs.getPosition2() &&
           lhs.getTime() == rhs.getTime();
  }
};

using HardVertexConstraintsSet =
    std::unordered_set<VertexConstraint, HardVertexConstraintHash,
                       HardVertexConstraintEqual>;
using HardEdgeConstraintsSet =
    std::unordered_set<EdgeConstraint, HardEdgeConstraintHash,
                       HardEdgeConstraintEqual>;

#endif // TFE_MAPF_CONSTRAINTSSET_H
