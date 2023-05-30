//
// Created by mansj on 10/11/22.
//

#include "Solvers/AStar/AStar.h"
#include "Solvers/CooperativeAStar.h"
#include "Problems/MultiAgentProblemWithConstraints.h"
#include "Problems/StandardMultiAgentProblemWithConstraints.h"
#include "GraphParser/Parser.h"
#include "Solvers/AStar/ReverseResumableAStar.h"
#include "Problems/SingleAgentProblem.h"
#include "Problems/SingleAgentProblemWithConstraints.h"
#include "Solvers/ID/OldSimpleIndependenceDetection.h"
#include "Solvers/ID/SimpleIndependenceDetection.h"
#include "Solvers/ID/OldIndependenceDetection.h"
#include "Solvers/ID/IndependenceDetection.h"
#include "Solvers/CBS/ConflictBasedSearch.h"

#include <chrono>
#include <iostream>
#include <vector>

template <typename T>
class Clock {
    std::chrono::high_resolution_clock clock;
    std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> start;
    std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> stop;
public:
    void tick() {
        this->start = this->clock.now();
    }
    void tack () {
        this->stop = this->clock.now();
    };
    void print () {
        std::cerr << std::chrono::duration_cast<T>(this->stop-this->start) << std::endl;
    };
};

Clock<std::chrono::milliseconds> timer;

int main() {

    // TEST 1 : 1 agent
    /*auto g = Parser::parse("../mapf-map/AssignmentIACourse.map");
    int start = 7;
    int target = 17;
    auto problem = std::make_shared<SingleAgentProblem>(g, start, target);
    auto solution = AStar<SingleAgentProblem, SingleAgentState>(problem, Manhattan).solve();
    // cost = 18, numberOfVisitedStates = 25
    solution->print();*/

    // TEST 2 : 1 agent
    /*auto g = Parser::parse("../mapf-map/Paris/Paris_1_256.map");
    int start = 1;
    int target = 256*200-100;
    timer.tick();
    auto problem = std::make_shared<SingleAgentProblem>(g, start, target);
    auto solution = AStar<SingleAgentProblem, SingleAgentState>(problem, Manhattan).solve();
    timer.tack();
    // cost = 354
    solution->print();
    timer.print();*/

    // TEST 3 : 2 agents can't be at the same vertex at the same time
    /*auto g = Parser::parse("../mapf-map/empty-4-4.map");
    vector<int> starts;
    starts.push_back(4);
    starts.push_back(9);
    vector<int> targets;
    targets.push_back(6);
    targets.push_back(1);
    auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, Makespan);
    auto solution = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, Manhattan).solve();
    // makespan cost = 3, fuel cost = 4,  sumofcosts cost = 5
    solution->print();*/

    // TEST 4 : 2 agents can't traverse the same edge between successive time steps
    /*auto g = Parser::parse("../mapf-map/empty-4-4.map");
    vector<int> starts;
    starts.push_back(0);
    starts.push_back(3);
    vector<int> targets;
    targets.push_back(3);
    targets.push_back(0);
    auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, SumOfCosts);
    auto solution = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, Manhattan).solve();
    // makespan cost = 5, fuel cost = 8,  sumofcosts cost = 8
    solution->print();*/

    // TEST 5 : 2 agents
    /*auto g = Parser::parse("../mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(48);
    starts.push_back(17);
    vector<int> targets;
    targets.push_back(17);
    targets.push_back(20);
    auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, Makespan);
    auto solution = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, Manhattan).solve();
    // makespan cost = 7, fuel cost = 10,  sumofcosts cost = 10
    solution->print();*/

    // TEST 6 : 1 agent and a hard vertex constraint
    /*auto g = Parser::parse("../mapf-map/empty-4-4.map");
    int start = 4;
    int target = 6;
    auto problem = std::make_shared<SingleAgentProblemWithConstraints>(g, start, target, Makespan, 0, std::set<VertexConstraint>{{0,5,1}});
    auto solution = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(problem, Manhattan).solve();
    solution->print();*/

    // TEST 6 BIS : 1 agent and a hard edge constraint
    /*auto g = Parser::parse("../mapf-map/empty-4-4.map");
    int start = 4;
    int target = 6;
    auto problem = std::make_shared<SingleAgentProblemWithConstraints>(g, start, target, Makespan, 0, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{{0, 4, 5, 1}});
    auto solution = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(problem, Manhattan).solve();
    solution->print();*/

    // TEST 7 : 1 agent and a soft vertex constraint
    /*auto g = Parser::parse("../mapf-map/empty-4-4.map");
    int start = 1;
    int target = 6;
    auto problem = std::make_shared<SingleAgentProblemWithConstraints>(g, start, target, Makespan, 0, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, INT_MAX, std::set<VertexConstraint>{{0,5,1}});
    auto solution = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(problem, Manhattan).solve();
    solution->print();*/

    // TEST 7 BIS : 1 agent and a soft edge constraint
    /*auto g = Parser::parse("../mapf-map/empty-4-4.map");
    int start = 1;
    int target = 6;
    auto problem = std::make_shared<SingleAgentProblemWithConstraints>(g, start, target, Makespan, 0, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, INT_MAX, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{{0,1,5,1}});
    auto solution = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(problem, Manhattan).solve();
    solution->print();*/

    // TEST 8 : Reverse Resumable A*
    /*auto g = Parser::parse("../mapf-map/ReverseResumableAStarExample.map");
    int start = 33;
    int target = 2;
    auto problem = std::make_shared<SingleAgentProblem>(g, start, target);
    ReverseResumableAStar search = ReverseResumableAStar(problem);
    std::cout << search.getDistance().size() << std::endl; // = 1
    std::cout << search.optimalDistance(0) << std::endl; // = 2
    std::cout << search.getDistance().size() << std::endl; // = 24
    std::cout << search.optimalDistance(5) << std::endl; // = 9
    std::cout << search.getDistance().size() << std::endl; // = 27
    std::cout << search.optimalDistance(6) << std::endl; // = 3
    std::cout << search.getDistance().size() << std::endl; // = 27
    for(const auto& key_value: search.getDistance()) {
        std::shared_ptr<SingleAgentState> key = key_value.first;
        int value = key_value.second;
        std::cout << key->getPosition() << " - " << value << std::endl;
    }*/

    // TEST 9 : Single agent space time search, comparaison between Manhattan distance and Optimal distance (RRA*)
    /*auto g = Parser::parse("../mapf-map/AssignmentIACourse.map");
    int start = 7;
    int target = 17;
    auto problem = std::make_shared<SingleAgentProblemWithConstraints>(g, start, target, Makespan);
    auto solution1 = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(problem, Manhattan).solve();
    auto solution2 = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(problem, OptimalDistance).solve();
    // makespan cost = 18, fuel cost = 18
    solution1->print(); // numberOfVisitedStates = 177
    solution2->print(); // numberOfVisitedStates = 19*/

    // TEST 10 : Comparaison between cooperative A* (Manhattan for the single agent A*) and hierarchical cooperative A* (Optimal distance RRA* for the single agent A*)
    /*auto g = Parser::parse("../mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(17);
    starts.push_back(22);
    vector<int> targets;
    targets.push_back(7);
    targets.push_back(6);
    auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, Makespan);
    auto solution1 = CooperativeAStar(problem, Manhattan).solve();
    auto solution2 = CooperativeAStar(problem, OptimalDistance).solve();
    solution1->print(); // numberOfVisitedStates = 389
    solution2->print(); // numberOfVisitedStates = 142*/

    // TEST 11 : 2 agents and comparaison between Manhattan and OptimalDistance
    /*auto g = Parser::parse("../mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(48);
    starts.push_back(17);
    vector<int> targets;
    targets.push_back(17);
    targets.push_back(20);
    auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, SumOfCosts);
    auto solution1 = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, Manhattan).solve();
    auto solution2 = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, OptimalDistance).solve();
    solution1->print(); // numberOfVisitedStates = 17
    solution2->print(); // numberOfVisitedStates = 17*/

    // TEST 12 : 2 agents and comparaison between Manhattan and OptimalDistance
    /*auto g = Parser::parse("../mapf-map/Paris/Paris_1_256.map");
    std::vector<int> starts;
    starts.push_back(1);
    starts.push_back(150);
    std::vector<int> targets;
    targets.push_back(150);
    targets.push_back(1);
    auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, SumOfCosts);
    auto solution1 = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, Manhattan).solve();
    solution1->print(); // numberOfVisitedStates = 690412
    auto solution2 = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, OptimalDistance).solve();
    solution2->print(); // numberOfVisitedStates = 331*/

    // TEST 13 : Small simple independence detection test
    /*auto g = Parser::parse("../mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(7);
    starts.push_back(3);
    starts.push_back(11);
    vector<int> targets;
    targets.push_back(21);
    targets.push_back(5);
    targets.push_back(4);
    auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, SumOfCosts, vector<int>{1,3,4});
    auto solution = SimpleIndependenceDetection(problem, OptimalDistance).solve();
    auto solution2 = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, OptimalDistance).solve();
    solution->print();
    solution2->print();*/

    // TEST 14 : Simple independence detection
    /*auto g = Parser::parse("../mapf-map/Paris/Paris_1_256.map");
    std::vector<int> starts;
    starts.push_back(1);
    starts.push_back(150);
    std::vector<int> targets;
    targets.push_back(150);
    targets.push_back(1);
    auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, Makespan, std::vector<int>{5,10});
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    auto solution1 = SimpleIndependenceDetection(problem, OptimalDistance).solve();
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << elapsed_seconds.count() << std::endl;
    start = std::chrono::system_clock::now();
    auto solution2 = AStar<MultiAgentProblemWithConstraints,MultiAgentState>(problem, OptimalDistance).solve();
    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;
    std::cout << elapsed_seconds.count() << std::endl;
    std::cout << solution1->getMakespanCost() << std::endl;
    std::cout << solution2->getMakespanCost() << std::endl;*/

    // TEST 15 : maxCost in an A* search
    /*auto g = Parser::parse("../mapf-map/Paris/Paris_1_256.map");
    int start = 1;
    int target = 256*200-100;
    auto problem1 = std::make_shared<SingleAgentProblem>(g, start, target, 0, 354);
    auto solution1 = AStar<SingleAgentProblem, SingleAgentState>(problem1, Manhattan).solve();
    solution1->print(); // foundPath = true
    auto problem2 = std::make_shared<SingleAgentProblem>(g, start, target, 0, 353);
    auto solution2 = AStar<SingleAgentProblem, SingleAgentState>(problem2, Manhattan).solve();
    solution2->print(); // foundPath = false*/

    // TEST 16 : Small independence detection test
    /*auto g = Parser::parse("../mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(7);
    starts.push_back(5);
    starts.push_back(11);
    vector<int> targets;
    targets.push_back(21);
    targets.push_back(11);
    targets.push_back(5);
    auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, SumOfCosts, vector<int>{3,4,5});
    auto solution = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, OptimalDistance).solve();
    solution->print();
    auto solution2 = IndependenceDetection(problem, OptimalDistance).solve();
    solution2->print();*/

    // TEST 15 : Conflict Based Search
    /*auto g = Parser::parse("../mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(7);
    starts.push_back(5);
    starts.push_back(11);
    vector<int> targets;
    targets.push_back(21);
    targets.push_back(11);
    targets.push_back(5);
    auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, SumOfCosts, vector<int>{3,4,5});
    auto solution1 = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, OptimalDistance).solve();
    solution1->print();
    auto solution2 = ConflictBasedSearch(problem, Manhattan).solve();
    solution2->print();*/

    // TEST 16 : Conflict Based Search
    /*auto g = Parser::parse("../mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(17);
    starts.push_back(22);
    vector<int> targets;
    targets.push_back(7);
    targets.push_back(6);
    auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, SumOfCosts);
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    auto solution1 = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, Manhattan).solve();
    solution1->print();
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds1 = end - start;
    start = std::chrono::system_clock::now();
    auto solution2 = IndependenceDetection(problem, Manhattan).solve();
    solution2->print();
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds2 = end - start;
    start = std::chrono::system_clock::now();
    auto solution3 = ConflictBasedSearch(problem, Manhattan).solve();
    solution3->print();
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds3 = end - start;
    std::cout << elapsed_seconds1.count() << std::endl;
    std::cout << elapsed_seconds2.count() << std::endl;
    std::cout << elapsed_seconds3.count() << std::endl;*/

    /*auto g = Parser::parse("../mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(37);
    starts.push_back(43);
    vector<int> targets;
    targets.push_back(26);
    targets.push_back(20);
    auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, SumOfCosts);
    auto solution1 = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, OptimalDistance).solve();
    auto solution2 = ConflictBasedSearch(problem, OptimalDistance).solve();
    solution1->print();
    solution2->print();*/

    /*auto g = Parser::parse("../mapf-map/Paris/Paris_1_256.map");
    vector<int> starts;
    int w = 256;
    starts.push_back(124*w+186);
    starts.push_back(69*w+168);
    starts.push_back(94*w+72);
    starts.push_back(179*w+204);
    starts.push_back(125*w+38);
    vector<int> targets;
    targets.push_back(190*w+109);
    targets.push_back(34*w+199);
    targets.push_back(95*w+174);
    targets.push_back(27*w+56);
    targets.push_back(36*w+236);
    auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, SumOfCosts);
    auto solution1 = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, OptimalDistance).solve();
    auto solution2 = ConflictBasedSearch(problem, OptimalDistance).solve();
    solution1->print();
    solution2->print();*/

    // TEST 15 : maxCost in a Cooperative A*
    /*auto g = Parser::parse("../mapf-map/Paris/Paris_1_256.map");
    auto problem1 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, Makespan, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 354);
    auto solution1 = CooperativeAStar(problem1, Manhattan).solve();
    std::cout << solution1->getFoundPath() << std::endl; // foundPath = true
    auto problem2 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, Makespan, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 353);
    auto solution2 = CooperativeAStar(problem2, Manhattan).solve();
    std::cout << solution2->getFoundPath() << std::endl; // foundPath = false
    auto problem3 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, SumOfCosts, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 355);
    auto solution3 = CooperativeAStar(problem3, Manhattan).solve();
    std::cout << solution3->getFoundPath() << std::endl; // foundPath = true
    auto problem4 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, SumOfCosts, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 354);
    auto solution4 = CooperativeAStar(problem4, Manhattan).solve();
    std::cout << solution4->getFoundPath() << std::endl; // foundPath = false*/

    // TEST 15 : maxCost in SID
    /*auto g = Parser::parse("../mapf-map/Paris/Paris_1_256.map");
    auto problem1 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, Makespan, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 354);
    auto solution1 = SimpleIndependenceDetection(problem1, Manhattan).solve();
    std::cout << solution1->getFoundPath() << std::endl; // foundPath = true
    auto problem2 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, Makespan, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 353);
    auto solution2 = SimpleIndependenceDetection(problem2, Manhattan).solve();
    std::cout << solution2->getFoundPath() << std::endl; // foundPath = false
    auto problem3 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, SumOfCosts, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 355);
    auto solution3 = SimpleIndependenceDetection(problem3, Manhattan).solve();
    std::cout << solution3->getFoundPath() << std::endl; // foundPath = true
    auto problem4 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, SumOfCosts, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 354);
    auto solution4 = SimpleIndependenceDetection(problem4, Manhattan).solve();
    std::cout << solution4->getFoundPath() << std::endl; // foundPath = false*/

    // TEST 15 : maxCost in OldSID
    /*auto g = Parser::parse("../mapf-map/Paris/Paris_1_256.map");
    auto problem1 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, Makespan, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 354);
    auto solution1 = OldSimpleIndependenceDetection(problem1, Manhattan).solve();
    std::cout << solution1->getFoundPath() << std::endl; // foundPath = true
    auto problem2 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, Makespan, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 353);
    auto solution2 = OldSimpleIndependenceDetection(problem2, Manhattan).solve();
    std::cout << solution2->getFoundPath() << std::endl; // foundPath = false
    auto problem3 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, SumOfCosts, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 355);
    auto solution3 = OldSimpleIndependenceDetection(problem3, Manhattan).solve();
    std::cout << solution3->getFoundPath() << std::endl; // foundPath = true
    auto problem4 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, SumOfCosts, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 354);
    auto solution4 = OldSimpleIndependenceDetection(problem4, Manhattan).solve();
    std::cout << solution4->getFoundPath() << std::endl; // foundPath = false*/

    // TEST 15 : maxCost in ID
    /*auto g = Parser::parse("../mapf-map/Paris/Paris_1_256.map");
    auto problem1 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, Makespan, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 354);
    auto solution1 = IndependenceDetection(problem1, Manhattan).solve();
    std::cout << solution1->getFoundPath() << std::endl; // foundPath = true
    auto problem2 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, Makespan, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 353);
    auto solution2 = IndependenceDetection(problem2, Manhattan).solve();
    std::cout << solution2->getFoundPath() << std::endl; // foundPath = false
    auto problem3 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, SumOfCosts, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 355);
    auto solution3 = IndependenceDetection(problem3, Manhattan).solve();
    std::cout << solution3->getFoundPath() << std::endl; // foundPath = true
    auto problem4 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, SumOfCosts, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 354);
    auto solution4 = IndependenceDetection(problem4, Manhattan).solve();
    std::cout << solution4->getFoundPath() << std::endl; // foundPath = false*/

    // TEST 15 : maxCost in OldID
    /*auto g = Parser::parse("../mapf-map/Paris/Paris_1_256.map");
    auto problem1 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, Makespan, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 354);
    auto solution1 = OldIndependenceDetection(problem1, Manhattan).solve();
    std::cout << solution1->getFoundPath() << std::endl; // foundPath = true
    auto problem2 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, Makespan, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 353);
    auto solution2 = OldIndependenceDetection(problem2, Manhattan).solve();
    std::cout << solution2->getFoundPath() << std::endl; // foundPath = false
    auto problem3 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, SumOfCosts, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 355);
    auto solution3 = OldIndependenceDetection(problem3, Manhattan).solve();
    std::cout << solution3->getFoundPath() << std::endl; // foundPath = true
    auto problem4 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, SumOfCosts, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 354);
    auto solution4 = OldIndependenceDetection(problem4, Manhattan).solve();
    std::cout << solution4->getFoundPath() << std::endl; // foundPath = false*/

    // TEST 15 : maxCost in CBS
    /*auto g = Parser::parse("../mapf-map/Paris/Paris_1_256.map");
    auto problem1 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, Makespan, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 354);
    auto solution1 = ConflictBasedSearch(problem1, Manhattan).solve();
    std::cout << solution1->getFoundPath() << std::endl; // foundPath = true
    auto problem2 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, Makespan, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 353);
    auto solution2 = ConflictBasedSearch(problem2, Manhattan).solve();
    std::cout << solution2->getFoundPath() << std::endl; // foundPath = false
    auto problem3 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, SumOfCosts, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 355);
    auto solution3 = ConflictBasedSearch(problem3, Manhattan).solve();
    std::cout << solution3->getFoundPath() << std::endl; // foundPath = true
    auto problem4 = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{1,2}, vector<int>{256*200-100,1}, SumOfCosts, vector<int>{0,1}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, 354);
    auto solution4 = ConflictBasedSearch(problem4, Manhattan).solve();
    std::cout << solution4->getFoundPath() << std::endl; // foundPath = false*/

    // TEST X : StandardMultiAgent : 2 agents can't be at the same vertex at the same time
    /*auto g = Parser::parse("../mapf-map/empty-4-4.map");
    vector<int> starts;
    starts.push_back(4);
    starts.push_back(9);
    vector<int> targets;
    targets.push_back(6);
    targets.push_back(1);
    auto problem = std::make_shared<StandardMultiAgentProblemWithConstraints>(g, starts, targets, SumOfCosts);
    auto solution = AStar<StandardMultiAgentProblemWithConstraints, StandardMultiAgentState>(problem, Manhattan).solve();
    // makespan cost = 3, fuel cost = 4,  sumofcosts cost = 5
    solution->print();*/

    // TEST X : StandardMultiAgent : 2 agents can't traverse the same edge between successive time steps
    /*auto g = Parser::parse("../mapf-map/empty-4-4.map");
    vector<int> starts;
    starts.push_back(0);
    starts.push_back(3);
    vector<int> targets;
    targets.push_back(3);
    targets.push_back(0);
    auto problem = std::make_shared<StandardMultiAgentProblemWithConstraints>(g, starts, targets, SumOfCosts);
    auto solution = AStar<StandardMultiAgentProblemWithConstraints, StandardMultiAgentState>(problem, Manhattan).solve();
    // makespan cost = 5, fuel cost = 8,  sumofcosts cost = 8
    solution->print();*/

    // TEST X : StandardMultiAgent : 2 agents
    /*auto g = Parser::parse("../mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(48);
    starts.push_back(17);
    vector<int> targets;
    targets.push_back(17);
    targets.push_back(20);
    auto problem = std::make_shared<StandardMultiAgentProblemWithConstraints>(g, starts, targets, Makespan);
    auto solution = AStar<StandardMultiAgentProblemWithConstraints, StandardMultiAgentState>(problem, Manhattan).solve();
    // makespan cost = 7, fuel cost = 10,  sumofcosts cost = 10
    solution->print();*/

    // TEST X : StandardMultiAgent and a hard vertex constraint
    /*auto g = Parser::parse("../mapf-map/empty-4-4.map");
    auto problem = std::make_shared<StandardMultiAgentProblemWithConstraints>(g, vector<int>{4}, vector<int>{6}, Fuel, vector<int>{0}, std::set<VertexConstraint>{{0,5,1}});
    auto solution = AStar<StandardMultiAgentProblemWithConstraints, StandardMultiAgentState>(problem, Manhattan).solve();
    solution->print();*/

    // TEST X : StandardMultiAgent and a hard edge constraint
    /*auto g = Parser::parse("../mapf-map/empty-4-4.map");
    auto problem = std::make_shared<StandardMultiAgentProblemWithConstraints>(g, vector<int>{4}, vector<int>{6}, Fuel, vector<int>{0}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{{0, 4, 5, 1}});
    auto solution = AStar<StandardMultiAgentProblemWithConstraints, StandardMultiAgentState>(problem, Manhattan).solve();
    solution->print();*/

    // TEST X : StandardMultiAgent and a soft vertex constraint
    /*auto g = Parser::parse("../mapf-map/empty-4-4.map");
    auto problem = std::make_shared<StandardMultiAgentProblemWithConstraints>(g, vector<int>{1}, vector<int>{6}, Fuel, vector<int>{0}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, INT_MAX, std::set<VertexConstraint>{{0,5,1}});
    auto solution = AStar<StandardMultiAgentProblemWithConstraints, StandardMultiAgentState>(problem, Manhattan).solve();
    solution->print();*/

    // TEST X : StandardMultiAgent and a soft edge constraint
    /*auto g = Parser::parse("../mapf-map/empty-4-4.map");
    auto problem = std::make_shared<StandardMultiAgentProblemWithConstraints>(g, vector<int>{1}, vector<int>{6}, Fuel, vector<int>{0}, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{}, INT_MAX, std::set<VertexConstraint>{}, std::set<EdgeConstraint>{{1,1,5,1}});
    auto solution = AStar<StandardMultiAgentProblemWithConstraints, StandardMultiAgentState>(problem, Manhattan).solve();
    solution->print();*/

    // TEST X : Disjoint splitting conflict based search
    /*auto g = Parser::parse("../mapf-map/empty-5-5.map");
    auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, vector<int>{5, 1}, vector<int>{9, 13}, SumOfCosts);
    auto solution = ConflictBasedSearch(problem, Manhattan, false, true).solve();
    solution->print();*/

    /*auto g = Parser::parse("../mapf-map/Paris/Paris_1_256.map");
    vector<int> starts;
    int w = 256;
    starts.push_back(31+w*131);
    starts.push_back(169+w*31);
    starts.push_back(192+w*252);
    starts.push_back(140+w*135);
    starts.push_back(162+w*225);
    starts.push_back(153+w*255);
    starts.push_back(252+w*42);
    starts.push_back(64+w*70);
    starts.push_back(216+w*229);
    starts.push_back(110+w*235);
    starts.push_back(209+w*138);
    starts.push_back(140+w*205);
    starts.push_back(31+w*7);
    starts.push_back(215+w*25);
    starts.push_back(3+w*36);
    starts.push_back(14+w*97);
    starts.push_back(104+w*204);
    starts.push_back(227+w*30);
    vector<int> targets;
    targets.push_back(158+w*235);
    targets.push_back(187+w*112);
    targets.push_back(63+w*138);
    targets.push_back(129+w*77);
    targets.push_back(222+w*102);
    targets.push_back(32+w*162);
    targets.push_back(53+w*193);
    targets.push_back(180+w*185);
    targets.push_back(101+w*187);
    targets.push_back(131+w*198);
    targets.push_back(150+w*81);
    targets.push_back(152+w*110);
    targets.push_back(131+w*194);
    targets.push_back(124+w*139);
    targets.push_back(188+w*24);
    targets.push_back(174+w*155);
    targets.push_back(199+w*252);
    targets.push_back(177+w*180);
    auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, Makespan);
    auto solution2 = ConflictBasedSearch(problem, OptimalDistance, false, true).solve();
    solution2->print();*/
}
