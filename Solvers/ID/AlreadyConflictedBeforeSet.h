//
// Created by Arthur Mahy on 16/05/2023.
//

#ifndef TFE_MAPF_ALREADYCONFLICTEDBEFORESET_H
#define TFE_MAPF_ALREADYCONFLICTEDBEFORESET_H

#include <iostream>
#include "Group.h"
#include "set"

struct PointerGroupHasher {
    std::size_t operator()(const std::shared_ptr<Group>& obj) const {
        return obj->getHash();
    }
};
struct PointerGroupEquality {
    bool operator()(const std::shared_ptr<Group>& lhs, const std::shared_ptr<Group>& rhs) const {
        return *lhs == *rhs;
    }
};
struct SetOfPointersHasher {
    std::size_t operator()(const std::set<std::shared_ptr<Group>, PointerGroupEquality>& set) const {
        std::size_t hash_value = 0;
        for (const auto& obj : set) {
            hash_value ^= PointerGroupHasher()(obj) + 0x9e3779b9 + (hash_value << 6) + (hash_value >> 2);
        }
        return hash_value;
    }
};
struct SetOfPointersEquality {
    bool operator()(const std::set<std::shared_ptr<Group>, PointerGroupEquality>& lhs, const std::set<std::shared_ptr<Group>, PointerGroupEquality>& rhs) const {
        return lhs == rhs;
    }
};


#endif //TFE_MAPF_ALREADYCONFLICTEDBEFORESET_H
