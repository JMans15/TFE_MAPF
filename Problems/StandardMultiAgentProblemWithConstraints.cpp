//
// Created by Arthur Mahy on 23/05/2023.
//

#include "StandardMultiAgentProblemWithConstraints.h"

#include <algorithm>

StandardMultiAgentProblemWithConstraints::StandardMultiAgentProblemWithConstraints(const std::shared_ptr<Graph>& graph, std::vector<int> starts, std::vector<int> targets,
                                                                   ObjectiveFunction objective, const std::vector<int>& m_agentIds,
                                                                   const HardVertexConstraintsSet &setOfHardVertexConstraints,
                                                                   const HardEdgeConstraintsSet &setOfHardEdgeConstraints, int maxCost,
                                                                   const SoftVertexConstraintsMultiSet& setOfSoftVertexConstraints,
                                                                   const SoftEdgeConstraintsMultiSet& setOfSoftEdgeConstraints,
                                                                   int startTime)
        : Problem(graph, (int)starts.size(), maxCost, startTime)
        , starts(starts)
        , targets(targets)
        , agentIds(m_agentIds)
        , objective(objective)
        , setOfHardVertexConstraints(setOfHardVertexConstraints)
        , setOfHardEdgeConstraints(setOfHardEdgeConstraints)
        , setOfSoftVertexConstraints(setOfSoftVertexConstraints)
        , setOfSoftEdgeConstraints(setOfSoftEdgeConstraints)
{
    LOG("==== Multi Agent Problem With Constraints (for a Standard A*) ====");
    LOG("Number of agents : " << numberOfAgents)

    if (objective != SumOfCosts && objective != Makespan && objective != Fuel) {
        LOG("The input for the objective function is not correct.");
        LOG("So, the default objective function will be applied.");
        objective = Fuel;
    }
    if (objective == SumOfCosts){
        LOG("Objective function : SumOfCosts");
    } else if (objective == Makespan){
        LOG("Objective function : Makespan");
    } else {
        LOG("Objective function : Fuel");
    }
    impossible = false;
    if ((not m_agentIds.empty()) and ((int)m_agentIds.size() != numberOfAgents)){
        impossible = true;
    }
    if (numberOfAgents != (int)targets.size()){
        impossible = true;
    }
    if (m_agentIds.empty()){
        for (int a = 0; a < numberOfAgents; a++){
            agentIds.emplace_back(a);
        }
    }
    for (int i = 0; i < numberOfAgents; i++){
        idToIndex[agentIds[i]] = i;
    }
    for (int agentId : agentIds){
        if (std::count(agentIds.begin(), agentIds.end(), agentId)>1){
            LOG("2 or more agents have the same id.");
            impossible = true;
            break;
        }
    }
    LOG("Start position of each agent :");
    for (int i = 0; i < numberOfAgents; i++) {
        LOG(" - Agent " << agentIds[i] << " : " << starts[i]);
        if (graph->getNeighbors(starts[i]).empty()) {
            LOG("   The start position of agent "<< agentIds[i] << " is unreachable.");
            impossible = true;
        }
    }
    LOG("Target position of each agent :");
    for (int i = 0; i < numberOfAgents; i++) {
        LOG(" - Agent " << agentIds[i] << " : " << targets[i]);
        if (graph->getNeighbors(targets[i]).empty()) {
            LOG("   The target position of agent "<< agentIds[i] << " is unreachable.");
            impossible = true;
        }
    }
    for (int start : starts){
        if (std::count(starts.begin(), starts.end(), start)>1){
            LOG("   2 or more agents have the same start position.");
            impossible = true;
            break;
        }
    }
    for (int target : targets){
        if (std::count(targets.begin(), targets.end(), target)>1){
            LOG("   2 or more agents have the same target position.");
            impossible = true;
            break;
        }
    }
    if (!setOfHardVertexConstraints.empty()){
        LOG("The problem has the following hard vertex constraints :");
        for (const auto& constraint : setOfHardVertexConstraints){
            LOG("   Agent " << constraint.getAgent() << " cannot be at position " << constraint.getPosition() << " at time " << constraint.getTime() << ".");
        }
    }
    if (!setOfHardEdgeConstraints.empty()){
        LOG("The problem has the following hard edge constraints :");
        for (const auto& constraint : setOfHardEdgeConstraints){
            LOG("   Agent " << constraint.getAgent() << " cannot go from position " << constraint.getPosition1() << " to position " << constraint.getPosition2() << " between " << constraint.getTime()-1 << " and time "<< constraint.getTime() << ".");
        }
    }
    if (!setOfSoftVertexConstraints.empty()){
        LOG("The problem has the following soft vertex constraints :");
        for (const auto& constraint : setOfSoftVertexConstraints){
            LOG("   Agent " << constraint.getAgent() << " is at position " << constraint.getPosition() << " at time " << constraint.getTime() << ".");
        }
    }
    if (!setOfSoftEdgeConstraints.empty()){
        LOG("The problem has the following soft edge constraints :");
        for (const auto& constraint : setOfSoftEdgeConstraints){
            LOG("   Agent " << constraint.getAgent() << " is occupying the edge (" << constraint.getPosition1() << ", " << constraint.getPosition2() << ") between time " << constraint.getTime()-1 << " and time "<< constraint.getTime() << ".");
        }
    }
    if (maxCost!=INT_MAX){
        LOG("The solution of this problem must have a cost inferior or equal to " << maxCost);
    }
    if (startTime!=0){
        LOG("The start time of the problem is " << startTime);
    }
    LOG(" ");
}

std::shared_ptr<StandardMultiAgentState> StandardMultiAgentProblemWithConstraints::getStartState() const {
    return std::make_shared<StandardMultiAgentState>(starts, startTime);
}

bool StandardMultiAgentProblemWithConstraints::isGoalState(std::shared_ptr<StandardMultiAgentState> state) const {
    auto positions = state->getPositions();
    for (int i = 0; i < numberOfAgents; i++) {
        if (positions[i] != targets[i]) {
            return false;
        }
    }
    return true;
}

// Returns true if position is not already occupied by assigned agents
bool StandardMultiAgentProblemWithConstraints::notAlreadyOccupiedPosition(int position, std::vector<int> &positions, int agentToAssign) const {
    for (int i = 0; i < agentToAssign; i++){
        if (positions[i] == position){
            return false;
        }
    }
    return true;
}

// Returns true if the edge (position, positions[agentToAssign]) is not already occupied by assigned agents
bool StandardMultiAgentProblemWithConstraints::notAlreadyOccupiedEdge(int position, const std::vector<int> &positions, int agentToAssign, const std::vector<int> &prePositions) const {
    for (int i = 0; i < agentToAssign; i++) {
        if (prePositions[i] == position && positions[i] == positions[agentToAssign]){
            return false;
        }
    }
    return true;
}

bool StandardMultiAgentProblemWithConstraints::okForConstraints(int agent, int position, int newPosition, int time) const {
    if (setOfHardVertexConstraints.find({agentIds[agent], newPosition, time}) == setOfHardVertexConstraints.end()){
        return setOfHardEdgeConstraints.find({agentIds[agent], position, newPosition, time}) == setOfHardEdgeConstraints.end();
    }
    return false;
}

bool StandardMultiAgentProblemWithConstraints::okForConstraints(int agent, int newPosition, int time) const {
    return setOfHardVertexConstraints.find({agentIds[agent], newPosition, time}) == setOfHardVertexConstraints.end();
}

int StandardMultiAgentProblemWithConstraints::numberOfViolations(int agent, int position, int newPosition, int time) const {
    int count = 0;
    auto range = setOfSoftVertexConstraints.equal_range({0, newPosition, time});
    for (auto it = range.first; it != range.second; ++it) {
        if (it->getAgent()!=agentIds[agent]){
            count += 1;
        }
    }
    auto range2 = setOfSoftEdgeConstraints.equal_range({0, position, newPosition, time});
    for (auto it = range2.first; it != range2.second; ++it) {
        if (it->getAgent()!=agentIds[agent]){
            count += 1;
        }
    }
    return count;
}

int StandardMultiAgentProblemWithConstraints::numberOfViolations(int agent, int newPosition, int time) const {
    int count = 0;
    auto range = setOfSoftVertexConstraints.equal_range({0, newPosition, time});
    for (auto it = range.first; it != range.second; ++it) {
        if (it->getAgent()!=agentIds[agent]){
            count += 1;
        }
    }
    return count;
}

void StandardMultiAgentProblemWithConstraints::recursiveAssignAMoveToAnAgent(int agentToAssign, std::vector<std::tuple<std::shared_ptr<StandardMultiAgentState>, int, int>>* successors, int cost, std::vector<int> positions, const std::vector<int>& prePositions, int t, int violations, std::vector<int> cannotMove) const {
    int costMovement;
    int costWait;
    if (objective == Fuel){
        costMovement = 1;
        costWait = 0;
    } else if (objective == Makespan) {
        if (agentToAssign == 0){
            // we are assigning a position to the first agent,
            // we know that we will need another timestep
            costMovement = 1;
            costWait = 1;
        } else {
            costMovement = 0;
            costWait = 0;
        }
    } else { // obj_function=="SumOfCosts"
        costMovement = 1;
    }
    if (objective != SumOfCosts){
        if (agentToAssign==numberOfAgents-1){
            // Move
            for (int j : graph->getNeighbors(positions[agentToAssign])) {
                vector<int> newpositions(positions);
                newpositions[agentToAssign] = j;
                if (notAlreadyOccupiedPosition(j, positions, agentToAssign) && notAlreadyOccupiedEdge(j, positions, agentToAssign, prePositions) && okForConstraints(agentToAssign, positions[agentToAssign], j, t+1)) {
                    auto successor = std::make_shared<StandardMultiAgentState>(newpositions, t+1);
                    successors->emplace_back(successor, cost+costMovement, violations+numberOfViolations(agentToAssign, positions[agentToAssign], j, t+1));
                }
            }
            // Wait
            if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && okForConstraints(agentToAssign, positions[agentToAssign], t+1)) {
                auto successor = std::make_shared<StandardMultiAgentState>(positions, t+1);
                successors->emplace_back(successor, cost+costWait, violations+numberOfViolations(agentToAssign, positions[agentToAssign], t+1));
            }
        } else {
            // Move
            for (int j : graph->getNeighbors(positions[agentToAssign])) {
                vector<int> newpositions(positions);
                newpositions[agentToAssign] = j;
                if (notAlreadyOccupiedPosition(j, positions, agentToAssign) && notAlreadyOccupiedEdge(j, positions, agentToAssign, prePositions) && okForConstraints(agentToAssign, positions[agentToAssign], j, t+1)) {
                    recursiveAssignAMoveToAnAgent(agentToAssign+1, successors, cost+costMovement, newpositions, prePositions, t, violations+numberOfViolations(agentToAssign, positions[agentToAssign], j, t+1));
                }
            }
            // Wait
            if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && okForConstraints(agentToAssign, positions[agentToAssign], t+1)) {
                recursiveAssignAMoveToAnAgent(agentToAssign+1, successors, cost+costWait, positions, prePositions, t, violations+numberOfViolations(agentToAssign, positions[agentToAssign], t+1));
            }
        }
    } else { // obj_function=="SumOfCosts"
        if (agentToAssign==numberOfAgents-1){
            if (not (std::find(cannotMove.begin(), cannotMove.end(), agentToAssign) != cannotMove.end())){ // agentToAssign is allowed to move

                // Move
                for (int j : graph->getNeighbors(positions[agentToAssign])) {
                    vector<int> newpositions(positions);
                    newpositions[agentToAssign] = j;
                    if (notAlreadyOccupiedPosition(j, positions, agentToAssign) && notAlreadyOccupiedEdge(j, positions, agentToAssign, prePositions) && okForConstraints(agentToAssign, positions[agentToAssign], j, t+1)) {
                        auto successor = std::make_shared<StandardMultiAgentState>(newpositions, t+1, cannotMove);
                        successors->emplace_back(successor, cost+costMovement, violations+numberOfViolations(agentToAssign, positions[agentToAssign], j, t+1));
                    }
                }

                // Wait
                if (positions[agentToAssign]!=targets[agentToAssign]) { // agentToAssign not at his target position
                    costWait = 1;
                    if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && okForConstraints(agentToAssign, positions[agentToAssign], t+1)) {
                        auto successor = std::make_shared<StandardMultiAgentState>(positions, t+1, cannotMove);
                        successors->emplace_back(successor, cost+costWait, violations+numberOfViolations(agentToAssign, positions[agentToAssign], t+1));
                    }
                } else { // agentToAssign is at his target position
                    // agentToAssign can still move in the future
                    costWait = 1;
                    if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && okForConstraints(agentToAssign, positions[agentToAssign], t+1)) {
                        auto successor = std::make_shared<StandardMultiAgentState>(positions, t+1, cannotMove);
                        successors->emplace_back(successor, cost+costWait, violations+numberOfViolations(agentToAssign, positions[agentToAssign], t+1));
                    }

                    // we are forcing agentToAssign to not move in the future
                    costWait = 0;
                    cannotMove.push_back(agentToAssign);
                    if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && okForConstraints(agentToAssign, positions[agentToAssign], t+1)) {
                        auto successor = std::make_shared<StandardMultiAgentState>(positions, t+1, cannotMove);
                        successors->emplace_back(successor, cost+costWait, violations+numberOfViolations(agentToAssign, positions[agentToAssign], t+1));
                    }
                }


            } else { // agentToAssign is not allowed to move
                costWait = 0;
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && okForConstraints(agentToAssign, positions[agentToAssign], t+1)) {
                    auto successor = std::make_shared<StandardMultiAgentState>(positions, t+1, cannotMove);
                    successors->emplace_back(successor, cost+costWait, violations+numberOfViolations(agentToAssign, positions[agentToAssign], t+1));
                }
            }
        } else {
            if (not (std::find(cannotMove.begin(), cannotMove.end(), agentToAssign) != cannotMove.end())){ // agentToAssign is allowed to move

                // Move
                for (int j : graph->getNeighbors(positions[agentToAssign])) {
                    vector<int> newpositions(positions);
                    newpositions[agentToAssign] = j;
                    if (notAlreadyOccupiedPosition(j, positions, agentToAssign) && notAlreadyOccupiedEdge(j, positions, agentToAssign, prePositions) && okForConstraints(agentToAssign, positions[agentToAssign], j, t+1)) {
                        recursiveAssignAMoveToAnAgent(agentToAssign+1, successors, cost+costMovement, newpositions, prePositions, t, violations+numberOfViolations(agentToAssign, positions[agentToAssign], j, t+1), cannotMove);
                    }
                }

                // Wait
                if (positions[agentToAssign]!=targets[agentToAssign]) { // agentToAssign not at his target position
                    costWait = 1;
                    if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && okForConstraints(agentToAssign, positions[agentToAssign], t+1)) {
                        recursiveAssignAMoveToAnAgent(agentToAssign+1, successors, cost+costWait, positions, prePositions, t, violations+numberOfViolations(agentToAssign, positions[agentToAssign], t+1), cannotMove);
                    }
                } else { // agentToAssign is at his target position
                    // agentToAssign can still move in the future
                    costWait = 1;
                    if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && okForConstraints(agentToAssign, positions[agentToAssign], t+1)) {
                        recursiveAssignAMoveToAnAgent(agentToAssign+1, successors, cost+costWait, positions, prePositions, t, violations+numberOfViolations(agentToAssign, positions[agentToAssign], t+1), cannotMove);
                    }

                    // we are forcing agentToAssign to not move in the future
                    costWait = 0;
                    cannotMove.push_back(agentToAssign);
                    if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && okForConstraints(agentToAssign, positions[agentToAssign], t+1)) {
                        recursiveAssignAMoveToAnAgent(agentToAssign+1, successors, cost+costWait, positions, prePositions, t, violations+numberOfViolations(agentToAssign, positions[agentToAssign], t+1), cannotMove);
                    }
                }


            } else { // agentToAssign is not allowed to move
                costWait = 0;
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && okForConstraints(agentToAssign, positions[agentToAssign], t+1)) {
                    recursiveAssignAMoveToAnAgent(agentToAssign+1, successors, cost+costWait, positions, prePositions, t, violations+numberOfViolations(agentToAssign, positions[agentToAssign], t+1), cannotMove);
                }
            }
        }
    }
}

std::vector<std::tuple<std::shared_ptr<StandardMultiAgentState>, int, int>> StandardMultiAgentProblemWithConstraints::getSuccessors(std::shared_ptr<StandardMultiAgentState> state) const {
    std::vector<std::tuple<std::shared_ptr<StandardMultiAgentState>, int, int>> successors;
    auto positions = state->getPositions();
    int t = state->getTimestep();

    recursiveAssignAMoveToAnAgent(0, &successors, 0, positions, positions, t, 0, state->getCannotMove());

    return successors;
}

std::unordered_map<int, std::vector<int>> StandardMultiAgentProblemWithConstraints::getPositions(std::vector<std::shared_ptr<StandardMultiAgentState>> states) const {
    std::unordered_map<int, std::vector<int>> positions;

    for (const auto& state : states) {
        for (int agent = 0; agent < numberOfAgents; agent++){
            positions[agentIds[agent]].push_back(state->getPositions()[agent]);
        }
    }

    return positions;
}


const std::vector<int>& StandardMultiAgentProblemWithConstraints::getStarts() const {
    return starts;
}

const std::vector<int>& StandardMultiAgentProblemWithConstraints::getTargets() const {
    return targets;
}

ObjectiveFunction StandardMultiAgentProblemWithConstraints::getObjFunction() {
    return objective;
}

HardVertexConstraintsSet StandardMultiAgentProblemWithConstraints::getSetOfHardVertexConstraints() const {
    return setOfHardVertexConstraints;
}

HardEdgeConstraintsSet StandardMultiAgentProblemWithConstraints::getSetOfHardEdgeConstraints() const {
    return setOfHardEdgeConstraints;
}

std::vector<int> StandardMultiAgentProblemWithConstraints::getAgentIds() const {
    return agentIds;
}

int StandardMultiAgentProblemWithConstraints::getStartOf(int id) {
    return starts[idToIndex[id]];
}

int StandardMultiAgentProblemWithConstraints::getTargetOf(int id) {
    return targets[idToIndex[id]];
}

bool StandardMultiAgentProblemWithConstraints::isImpossible() const {
    return impossible;
}

SoftVertexConstraintsMultiSet StandardMultiAgentProblemWithConstraints::getSetOfSoftVertexConstraints() const {
    return setOfSoftVertexConstraints;
}

SoftEdgeConstraintsMultiSet StandardMultiAgentProblemWithConstraints::getSetOfSoftEdgeConstraints() const {
    return setOfSoftEdgeConstraints;
}
