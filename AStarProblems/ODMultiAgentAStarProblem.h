//
// Created by Arthur Mahy on 11/06/2023.
//

#ifndef TFE_MAPF_ODMULTIAGENTASTARPROBLEM_H
#define TFE_MAPF_ODMULTIAGENTASTARPROBLEM_H

#include "AStarProblem.h"
#include "../States/ODMultiAgentState.h"
#include "../Problems/MultiAgentProblem.h"

// Multi Agent Problem solved with Operator Decomposition A* (>< Standard A*)
class ODMultiAgentAStarProblem : AStarProblem<ODMultiAgentState>{
public:
    ODMultiAgentAStarProblem(std::shared_ptr<MultiAgentProblem>  problem);

    std::shared_ptr<ODMultiAgentState> getStartState() const override;
    bool isGoalState(std::shared_ptr<ODMultiAgentState> state) const override;
    std::vector<std::tuple<std::shared_ptr<ODMultiAgentState>, int, int>> getSuccessors(std::shared_ptr<ODMultiAgentState> state) const override;
    std::unordered_map<int, std::vector<int>> getPositions(std::vector<std::shared_ptr<ODMultiAgentState>> states) const override;
    int getMaxCost() const override;
    int getStartTime() const override;

    std::shared_ptr<MultiAgentProblem> getProblem();

private:
    std::shared_ptr<MultiAgentProblem> problem;

    // Returns true if position is not already occupied by assigned agents
    bool notAlreadyOccupiedPosition(int position, std::vector<int> &positions, int agentToAssign) const;

    // Returns true if the edge (position, positions[agentToAssign]) is not already occupied by assigned agents
    bool notAlreadyOccupiedEdge(int position, const std::vector<int> &positions, int agentToAssign, const std::vector<int> &prePositions) const;

};


#endif //TFE_MAPF_ODMULTIAGENTASTARPROBLEM_H
