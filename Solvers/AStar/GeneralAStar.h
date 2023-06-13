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
class GeneralAStar {
public:
    GeneralAStar(std::shared_ptr<Problem> problem, TypeOfHeuristic typeOfHeuristic, bool spaceTimeSearch, bool operatorDecomposition = true)
            : problem(std::move(problem))
            , typeOfHeuristic(typeOfHeuristic)
            , spaceTimeSearch(spaceTimeSearch)
            , operatorDecomposition(operatorDecomposition)
            , fixedParameters(true)
    {}

    GeneralAStar(std::shared_ptr<Problem> m_problem, TypeOfHeuristic typeOfHeuristic)
            : problem(std::move(m_problem))
            , typeOfHeuristic(typeOfHeuristic)
            , fixedParameters(false)
    {
        operatorDecomposition = true;
        if (problem->hasTimeConstraints()){
            spaceTimeSearch = true;
        } else {
            spaceTimeSearch = false;
        }
    }

    GeneralAStar(TypeOfHeuristic typeOfHeuristic, bool spaceTimeSearch, bool operatorDecomposition = true)
            : typeOfHeuristic(typeOfHeuristic)
            , spaceTimeSearch(spaceTimeSearch)
            , operatorDecomposition(operatorDecomposition)
            , fixedParameters(true)
    {}

    GeneralAStar(TypeOfHeuristic typeOfHeuristic)
            : typeOfHeuristic(typeOfHeuristic)
            , fixedParameters(false)
    {
        operatorDecomposition = true;
    }

    std::shared_ptr<Solution> solve(){
        if (problem->isImpossible()){
            return std::make_shared<Solution>();
        }
        if (problem->isMultiAgentProblem()){
            auto prob = std::dynamic_pointer_cast<MultiAgentProblem>(problem);
            if (spaceTimeSearch){
                if (operatorDecomposition){
                    return AStar<ODMultiAgentAStarProblemWithConstraints, MultiAgentSpaceTimeState>(std::make_shared<ODMultiAgentAStarProblemWithConstraints>(prob), typeOfHeuristic).solve();
                } else {
                    return AStar<StandardMultiAgentAStarProblemWithConstraints, StandardMultiAgentSpaceTimeState>(std::make_shared<StandardMultiAgentAStarProblemWithConstraints>(prob), typeOfHeuristic).solve();
                }
            } else {
                if (operatorDecomposition){
                    return AStar<ODMultiAgentAStarProblem, MultiAgentState>(std::make_shared<ODMultiAgentAStarProblem>(prob), typeOfHeuristic).solve();
                } else {
                    return AStar<StandardMultiAgentAStarProblem, StandardMultiAgentState>(std::make_shared<StandardMultiAgentAStarProblem>(prob), typeOfHeuristic).solve();
                }
            }
        } else {
            auto prob = std::dynamic_pointer_cast<SingleAgentProblem>(problem);
            if (spaceTimeSearch){
                return AStar<SingleAgentAStarProblemWithConstraints, SingleAgentSpaceTimeState>(std::make_shared<SingleAgentAStarProblemWithConstraints>(prob), typeOfHeuristic).solve();
            } else {
                return AStar<SingleAgentAStarProblem, SingleAgentState>(std::make_shared<SingleAgentAStarProblem>(prob), typeOfHeuristic).solve();
            }
        }
    }

    std::shared_ptr<Solution> solve(const std::shared_ptr<MultiAgentProblem>& m_problem){
        if (not fixedParameters){
            if (m_problem->hasTimeConstraints()){
                spaceTimeSearch = true;
            } else {
                spaceTimeSearch = false;
            }
        }
        problem = m_problem;
        return solve();
    }

private:
    std::shared_ptr<Problem> problem;
    TypeOfHeuristic typeOfHeuristic;
    bool fixedParameters;
    bool spaceTimeSearch; // SpaceTimeSearch or SpaceSearch (for single and multi)
    bool operatorDecomposition; // OperatorDecomposition or Standard (for multi)
};

#endif //TFE_MAPF_GENERALASTAR_H
