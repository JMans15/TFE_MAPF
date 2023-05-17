#include <utility>

//
// Created by Arthur Mahy on 14/05/2023.
//

#ifndef TFE_MAPF_GROUPCONFLICT_H
#define TFE_MAPF_GROUPCONFLICT_H


class GroupConflict {
public:
    // conflict between group A and group B at time
    GroupConflict(std::shared_ptr<Group> groupA, std::shared_ptr<Group> groupB, int time) : groupA(std::move(groupA)), groupB(std::move(groupB)), time(time) {}
    ~GroupConflict() = default;

    std::shared_ptr<Group> getGroupA() const {
        return groupA;
    }
    std::shared_ptr<Group> getGroupB() const {
        return groupB;
    }
    int getTime() const {
        return time;
    }

    bool operator<(const GroupConflict &other) const {
        if (time == other.time) {
            return true;
        }
        return time < other.time;
    }

private:
    std::shared_ptr<Group> groupA;
    std::shared_ptr<Group> groupB;
    int time;
};


#endif //TFE_MAPF_GROUPCONFLICT_H
