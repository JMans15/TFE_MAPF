#include "Group.h"
#include <memory>
#include <utility>

//
// Created by Arthur Mahy on 14/05/2023.
//

#ifndef TFE_MAPF_GROUPCONFLICT_H
#define TFE_MAPF_GROUPCONFLICT_H

class GroupConflict {
public:
  // conflict between group A and group B at time
  GroupConflict(std::shared_ptr<Group>
                    groupA, /**< First group involved in the conflict */
                std::shared_ptr<Group>
                    groupB, /**< Second group involved in the conflict */
                int time    /**< Time at which the conflict occurs */
                )
      : groupA(std::move(groupA)), groupB(std::move(groupB)), time(time) {
    /** Creates a Groupconflict for groups "groupA" and
     *  "groupB" at time "time"
     */
  }
  ~GroupConflict() = default;

  std::shared_ptr<Group> getGroupA() const {
    /** Getter for the frist group involved in the conflict */
    return groupA;
  }
  std::shared_ptr<Group> getGroupB() const {
    /** Getter for the second group involved in the conflict */
    return groupB;
  }
  int getTime() const {
    /** Getter for the time at which the conflict occured */
    return time;
  }

  bool operator<(const GroupConflict &other) const {
    /** Comparator function for two conflicts defined as the comparison between
     * the time at which they occur */
    if (time == other.time) {
      return true;
    }
    return time < other.time;
  }

private:
  std::shared_ptr<Group> groupA; /**< First group involved in the conflict */
  std::shared_ptr<Group> groupB; /**< Second group involved in the conflict */
  int time;                      /**< Time at which the conflict occurs */
};

#endif // TFE_MAPF_GROUPCONFLICT_H
