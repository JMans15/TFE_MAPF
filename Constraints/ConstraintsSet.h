//
// Created by Arthur Mahy on 05/06/2023.
//

#ifndef TFE_MAPF_CONSTRAINTSSET_H
#define TFE_MAPF_CONSTRAINTSSET_H

#include "VertexConstraint.h"
#include "EdgeConstraint.h"

#include <boost/functional/hash.hpp>
#include "iostream"
#include <unordered_set>

struct SoftVertexConstraintHash {
    std::size_t operator()(const VertexConstraint& constraint) const {
        size_t result = 0;
        boost::hash_combine(result, constraint.getPosition());
        boost::hash_combine(result, constraint.getTime());
        return result;
    }
};

struct SoftVertexConstraintEqual {
    bool operator()(const VertexConstraint& lhs, const VertexConstraint& rhs) const {
        return lhs.getPosition() == rhs.getPosition() && lhs.getTime() == rhs.getTime();
    }
};

struct SoftEdgeConstraintHash {
    std::size_t operator()(const EdgeConstraint& constraint) const {
        size_t result = 0;
        boost::hash_combine(result, constraint.getPosition1());
        boost::hash_combine(result, constraint.getPosition2());
        boost::hash_combine(result, constraint.getTime());
        return result;
    }
};

struct SoftEdgeConstraintEqual {
    bool operator()(const EdgeConstraint& lhs, const EdgeConstraint& rhs) const {
        return lhs.getPosition1() == rhs.getPosition1() && lhs.getPosition2() == rhs.getPosition2() && lhs.getTime() == rhs.getTime();
    }
};

using SoftVertexConstraintsMultiSet = std::unordered_multiset<VertexConstraint, SoftVertexConstraintHash, SoftVertexConstraintEqual>;
using SoftEdgeConstraintsMultiSet = std::unordered_multiset<EdgeConstraint, SoftEdgeConstraintHash, SoftEdgeConstraintEqual>;

#endif //TFE_MAPF_CONSTRAINTSSET_H
