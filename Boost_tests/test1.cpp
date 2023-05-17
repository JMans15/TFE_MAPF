//
// Created by mansj on 15/03/23.
//

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN  // in only one cpp file
#include <boost/test/unit_test.hpp>

#include "../Solvers/AStar/AStar.h"
#include "../Solvers/CooperativeAStar.h"
#include "../Problems/MultiAgentProblemWithConstraints.h"
#include "../GraphParser/Parser.h"
#include "../Solvers/AStar/ReverseResumableAStar.h"
#include "../Problems/SingleAgentProblem.h"
#include "../Problems/SingleAgentProblemWithConstraints.h"
#include "../Solvers/ID/IndependenceDetection.h"
#include "../Solution/Solution.h"

#include <chrono>
#include <iostream>
#include <vector>

BOOST_AUTO_TEST_SUITE(globalTests)

    BOOST_AUTO_TEST_CASE(one_agent_simple_map) {
        auto g = Parser::parse("../../mapf-map/AssignmentIACourse.map");
        int start = 7;
        int target = 17;
        auto problem = std::make_shared<SingleAgentProblem>(g, start, target);
        auto solution = AStar<SingleAgentProblem, SingleAgentState>(problem, Manhattan).solve();
        BOOST_REQUIRE_MESSAGE(solution->getFoundPath(), "Found a path");
        BOOST_CHECK_MESSAGE(solution->getNumberOfVisitedNodes() == 25, "NumberOfVisitedStates = " << solution->getNumberOfVisitedNodes() << " tested against 25");
        BOOST_CHECK_MESSAGE(solution->getCost() == 18, "Cost = " << solution->getCost() << " tested against 18");
    }

    BOOST_AUTO_TEST_CASE(one_agent_bigger_map) {
        auto g = Parser::parse("../../mapf-map/Paris/Paris_1_256.map");
        int start = 1;
        int target = 256*200-100;
        auto problem = std::make_shared<SingleAgentProblem>(g, start, target);
        auto solution = AStar<SingleAgentProblem, SingleAgentState>(problem, Manhattan).solve();
        BOOST_REQUIRE_MESSAGE(solution->getFoundPath(), "Found a path");
        BOOST_CHECK_MESSAGE(solution->getCost() == 354, "Cost = " << solution->getCost() << " tested against 354");
    }

    BOOST_AUTO_TEST_CASE(two_agents_vertex_constraint) {
        auto g = Parser::parse("../../mapf-map/empty-4-4.map");
        vector<int> starts;
        starts.push_back(4);
        starts.push_back(9);
        vector<int> targets;
        targets.push_back(6);
        targets.push_back(1);

        auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, Makespan);
        auto solution = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, Manhattan).solve();
        BOOST_REQUIRE_MESSAGE(solution->getFoundPath(), "Found a path");
        BOOST_CHECK_MESSAGE(solution->getMakespanCost() == 3, "MakespanCost = " << solution->getMakespanCost() << " tested against 3");

        problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, Fuel);
        solution = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, Manhattan).solve();
        BOOST_REQUIRE_MESSAGE(solution->getFoundPath(), "Found a path");
        BOOST_CHECK_MESSAGE(solution->getFuelCost() == 4, "FuelCost = " << solution->getFuelCost() << " tested against 4");

        problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, SumOfCosts);
        solution = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, Manhattan).solve();
        BOOST_REQUIRE_MESSAGE(solution->getFoundPath(), "Found a path");
        BOOST_CHECK_MESSAGE(solution->getSumOfCostsCost() == 5, "SumOfCostsCost = " << solution->getSumOfCostsCost() << " tested against 5");
    }

    BOOST_AUTO_TEST_CASE(two_agents_edge_constraint) {
        auto g = Parser::parse("../../mapf-map/empty-4-4.map");
        vector<int> starts;
        starts.push_back(0);
        starts.push_back(3);
        vector<int> targets;
        targets.push_back(3);
        targets.push_back(0);
        auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, Makespan);
        auto solution = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, Manhattan).solve();
        BOOST_REQUIRE_MESSAGE(solution->getFoundPath(), "Found a path");
        BOOST_CHECK_MESSAGE(solution->getMakespanCost() == 5, "MakespanCost = " << solution->getMakespanCost() << " tested against 5");

        problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, Fuel);
        solution = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, Manhattan).solve();
        BOOST_REQUIRE_MESSAGE(solution->getFoundPath(), "Found a path");
        BOOST_CHECK_MESSAGE(solution->getFuelCost() == 8, "FuelCost = " << solution->getFuelCost() << " tested against 8");

        problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, SumOfCosts);
        solution = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, Manhattan).solve();
        BOOST_REQUIRE_MESSAGE(solution->getFoundPath(), "Found a path");
        BOOST_CHECK_MESSAGE(solution->getSumOfCostsCost() == 8, "SumOfCostsCost = " << solution->getSumOfCostsCost() << " tested against 8");
    }

    BOOST_AUTO_TEST_CASE(two_agents) {
        auto g = Parser::parse("../../mapf-map/AssignmentIACourse.map");
        vector<int> starts;
        starts.push_back(48);
        starts.push_back(17);
        vector<int> targets;
        targets.push_back(17);
        targets.push_back(20);
        auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, Makespan);
        auto solution = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, Manhattan).solve();
        BOOST_REQUIRE_MESSAGE(solution->getFoundPath(), "Found a path");
        BOOST_CHECK_MESSAGE(solution->getMakespanCost() == 7, "MakespanCost = " << solution->getMakespanCost() << " tested against 7");

        problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, Fuel);
        solution = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, Manhattan).solve();
        BOOST_REQUIRE_MESSAGE(solution->getFoundPath(), "Found a path");
        BOOST_CHECK_MESSAGE(solution->getFuelCost() == 10, "FuelCost = " << solution->getFuelCost() << " tested against 10");

        problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, SumOfCosts);
        solution = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, Manhattan).solve();
        BOOST_REQUIRE_MESSAGE(solution->getFoundPath(), "Found a path");
        BOOST_CHECK_MESSAGE(solution->getSumOfCostsCost() == 10, "SumOfCostsCost = " << solution->getSumOfCostsCost() << " tested against 10");
    }

    BOOST_AUTO_TEST_CASE(one_constrained_agent) {
        auto g = Parser::parse("../../mapf-map/empty-4-4.map");
        int start = 4;
        int target = 6;
        auto problem = std::make_shared<SingleAgentProblemWithConstraints>(g, start, target, Makespan, 0, std::set<VertexConstraint>{VertexConstraint{0, 5, 1}});
        auto solution = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(problem, Manhattan).solve();
        BOOST_REQUIRE_MESSAGE(solution->getFoundPath(), "Found a path");
        BOOST_CHECK_MESSAGE(solution->getMakespanCost() == 3, "MakespanCost = " << solution->getMakespanCost() << " tested against 3");

        problem = std::make_shared<SingleAgentProblemWithConstraints>(g, start, target, Fuel, 0, std::set<VertexConstraint>{VertexConstraint{0, 5, 1}});
        solution = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(problem, Manhattan).solve();
        BOOST_REQUIRE_MESSAGE(solution->getFoundPath(), "Found a path");
        BOOST_CHECK_MESSAGE(solution->getFuelCost() == 2, "FuelCost = " << solution->getFuelCost() << " tested against 2");
    }

    BOOST_AUTO_TEST_CASE(reverse_resumable) {
        auto g = Parser::parse("../../mapf-map/ReverseResumableAStarExample.map");
        int start = 33;
        int target = 2;
        auto problem = std::make_shared<SingleAgentProblem>(g, start, target);
        ReverseResumableAStar search = ReverseResumableAStar(problem);
        BOOST_CHECK_MESSAGE(search.getDistance().size() == 1, "Distance size = " << search.getDistance().size() << " tested against 1");
        auto optDist = search.optimalDistance(0);
        BOOST_CHECK_MESSAGE(optDist == 2, "Optimal distance after resume = " << optDist << " tested against 2");
        BOOST_CHECK_MESSAGE(search.getDistance().size() == 24, "Distance size = " << search.getDistance().size() << " tested against 24");
        optDist = search.optimalDistance(5);
        BOOST_CHECK_MESSAGE(optDist == 9, "Optimal distance after resume = " << optDist << " tested against 9");
        BOOST_CHECK_MESSAGE(search.getDistance().size() == 27, "Distance size = " << search.getDistance().size() << " tested against 27");
        optDist = search.optimalDistance(6);
        BOOST_CHECK_MESSAGE(optDist == 3, "Optimal distance after resume = " << optDist << " tested against 3");
        BOOST_CHECK_MESSAGE(search.getDistance().size() == 27, "Distance size = " << search.getDistance().size() << " tested against 27");
        int p[27] = {11, 12, 17, 24, 6, 35, 0, 29, 23, 28, 5, 34, 33, 2, 15, 30, 1, 8, 21, 7, 20, 9, 22, 19, 32, 26, 25};
        int v[27] = {8, 4, 7, 8, 3, 8, 2, 7, 6, 6, 9, 7, 8, 0, 3, 9, 1, 1, 4, 2, 5, 2, 5, 6, 7, 6, 7};
        std::unordered_map<int, int> expected;
        for (int i = 0; i < 27; i++){
            expected[p[i]]=v[i];
        }
        for(const auto& key_value: search.getDistance()) {
            std::shared_ptr<SingleAgentState> key = key_value.first;
            int value = key_value.second;
            BOOST_REQUIRE_MESSAGE(expected[key->getPosition()] == value, "Value = " << value << " tested against " << expected[key->getPosition()]);
        }
    }

    BOOST_AUTO_TEST_CASE(single_spacetime) {
        auto g = Parser::parse("../../mapf-map/AssignmentIACourse.map");
        int start = 7;
        int target = 17;

        auto problem = std::make_shared<SingleAgentProblemWithConstraints>(g, start, target, Fuel);
        auto solution1 = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(problem, OptimalDistance).solve();
        BOOST_CHECK_MESSAGE(solution1->getFuelCost() == 18, "FuelCost = " << solution1->getFuelCost() << " tested against 18");

        problem = std::make_shared<SingleAgentProblemWithConstraints>(g, start, target, Makespan);
        solution1 = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(problem, Manhattan).solve();
        auto solution2 = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(problem, OptimalDistance).solve();
        BOOST_CHECK_MESSAGE(solution1->getMakespanCost() == 18, "MakespanCost = " << solution1->getMakespanCost() << " tested against 18");

        BOOST_CHECK_MESSAGE(solution1->getNumberOfVisitedNodes() == 177, "NumberOfVisitedStates = " << solution1->getNumberOfVisitedNodes() << " for Manhattan, tested against 177");
        BOOST_CHECK_MESSAGE(solution2->getNumberOfVisitedNodes() == 19, "NumberOfVisitedStates = " << solution2->getNumberOfVisitedNodes() << " for OptimalDistance, tested against 19");
    }

    BOOST_AUTO_TEST_CASE(cooperative) {
        auto g = Parser::parse("../../mapf-map/AssignmentIACourse.map");
        vector<int> starts;
        starts.push_back(17);
        starts.push_back(22);
        vector<int> targets;
        targets.push_back(7);
        targets.push_back(6);
        auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, Makespan);
        auto solution1 = CooperativeAStar(problem, Manhattan).solve();
        auto solution2 = CooperativeAStar(problem, OptimalDistance).solve();
        BOOST_REQUIRE_MESSAGE(solution1->getFoundPath(), "Found a path with Manhattan");
        BOOST_REQUIRE_MESSAGE(solution2->getFoundPath(), "Found a path with OptimalDistance");
    }

    BOOST_AUTO_TEST_CASE(independence_detection_simple) {
        auto g = Parser::parse("../../mapf-map/AssignmentIACourse.map");
        vector<int> starts;
        starts.push_back(7);
        starts.push_back(3);
        starts.push_back(11);
        vector<int> targets;
        targets.push_back(21);
        targets.push_back(5);
        targets.push_back(4);
        auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, SumOfCosts, vector<int>{3, 4, 5});
        auto solution = IndependenceDetection(problem, OptimalDistance).solve();
        solution->print();
        BOOST_REQUIRE_MESSAGE(solution->getFoundPath(), "Found a path");
    }

    BOOST_AUTO_TEST_CASE(independence_detection_full) {
        auto g = Parser::parse("../../mapf-map/Paris/Paris_1_256.map");
        std::vector<int> starts;
        starts.push_back(1);
        starts.push_back(150);
        std::vector<int> targets;
        targets.push_back(150);
        targets.push_back(1);
        auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, SumOfCosts, std::vector<int>{5, 10});
        auto solution = IndependenceDetection(problem, OptimalDistance).solve();
        solution->print();
        BOOST_REQUIRE_MESSAGE(solution->getFoundPath(), "Found a path");
    }

BOOST_AUTO_TEST_SUITE_END()