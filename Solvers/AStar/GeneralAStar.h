//
// Created by Arthur Mahy on 10/06/2023.
//

#ifndef TFE_MAPF_GENERALASTAR_H
#define TFE_MAPF_GENERALASTAR_H

#include "AStar.h"

#include <utility>
#include "../../Problems/Problem.h"

// Basic A* search
// Can be applied for multi agent and single agent problems
//
// Only takes in account negative constraints of problem
class GeneralAStar {
public:

    GeneralAStar(TypeOfHeuristic typeOfHeuristic, bool spaceTimeSearch, bool operatorDecomposition = true)
            : typeOfHeuristic(typeOfHeuristic)
            , fixedParameters(true)
            , spaceTimeSearch(spaceTimeSearch)
            , operatorDecomposition(operatorDecomposition)
    {}

    GeneralAStar(TypeOfHeuristic typeOfHeuristic)
            : typeOfHeuristic(typeOfHeuristic)
            , fixedParameters(false)
    {
        operatorDecomposition = true;
    }

    std::shared_ptr<Solution> solve(const std::shared_ptr<MultiAgentProblem>& problem){
        if (not fixedParameters){
            if (problem->hasExternalConstraints()){
                spaceTimeSearch = true;
            } else {
                spaceTimeSearch = false;
            }
        }
        if (problem->isImpossible()){
            return std::make_shared<Solution>();
        }
        if (spaceTimeSearch){
            if (operatorDecomposition){
                return AStar<ODMultiAgentAStarProblemWithConstraints, ODMultiAgentSpaceTimeState>(std::make_shared<ODMultiAgentAStarProblemWithConstraints>(problem), typeOfHeuristic).solve();
            } else {
                return AStar<StandardMultiAgentAStarProblemWithConstraints, StandardMultiAgentSpaceTimeState>(std::make_shared<StandardMultiAgentAStarProblemWithConstraints>(problem), typeOfHeuristic).solve();
            }
        } else {
            if (operatorDecomposition){
                return AStar<ODMultiAgentAStarProblem, ODMultiAgentState>(std::make_shared<ODMultiAgentAStarProblem>(problem), typeOfHeuristic).solve();
            } else {
                return AStar<StandardMultiAgentAStarProblem, StandardMultiAgentState>(std::make_shared<StandardMultiAgentAStarProblem>(problem), typeOfHeuristic).solve();
            }
        }
    }

    std::shared_ptr<Solution> solve(const std::shared_ptr<SingleAgentProblem>& problem){
        if (not fixedParameters){
            if (problem->hasExternalConstraints()){
                spaceTimeSearch = true;
            } else {
                spaceTimeSearch = false;
            }
        }
        if (problem->isImpossible()){
            return std::make_shared<Solution>();
        }
        if (spaceTimeSearch){
            return AStar<SingleAgentAStarProblemWithConstraints, SingleAgentSpaceTimeState>(std::make_shared<SingleAgentAStarProblemWithConstraints>(problem), typeOfHeuristic).solve();
        } else {
            return AStar<SingleAgentAStarProblem, SingleAgentState>(std::make_shared<SingleAgentAStarProblem>(problem), typeOfHeuristic).solve();
        }
    }

private:
    TypeOfHeuristic typeOfHeuristic;
    bool fixedParameters;
    bool spaceTimeSearch; // SpaceTimeSearch or SpaceSearch (for single and multi)
    bool operatorDecomposition; // OperatorDecomposition or Standard (for multi)
};

#endif //TFE_MAPF_GENERALASTAR_H
