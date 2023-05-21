//
// Created by mansj on 31/01/23.
//

// What do we need to control?
// -> algorithm (multi only)
//    -> A*
//    -> CBS
//    -> Coop
//    -> ID
//    -> SID
// -> scenario
// -> number of agents
// -> MAP
// -> Heuristics
// -> Objective functions

#include "Solvers/AStar/AStar.h"
#include "Solvers/ID/SimpleIndependenceDetection.h"
#include "Solvers/ID/IndependenceDetection.h"
#include "Solvers/CBS/ConflictBasedSearch.h"
#include "Solvers/CooperativeAStar.h"
#include "Problems/MultiAgentProblemWithConstraints.h"
#include "GraphParser/Parser.h"
#include "Problems/SingleAgentProblem.h"
#include "Solution/Solution.h"

#include "external-headers/cxxopts.hpp"

#include <filesystem>
#include <fstream>

int main(int argc, const char** argv) {
    cxxopts::Options options("TFE_MAPF_visu", "Runner program for visualizing our MAPF results");

    options.add_options()
            ("map", "Path to file containing the map info", cxxopts::value<std::string>())
            ("scen", "Path to file containing the scenario info", cxxopts::value<std::string>())
            ("v, verbose", "Verbose output", cxxopts::value<bool>()->default_value("false"))
            ("h, help", "Print help")
            ("heuristic", "Heuristic to use [Optimal, Manhattan]", cxxopts::value<string>()->default_value("Optimal"))
            ("o, objective", "Objective function to use [SumOfCosts, Fuel, Makespan]", cxxopts::value<string>()->default_value("SumOfCosts"))
            ("a, algo", "Algorithm to use [AStar, CBS, Coop, ID, SID]", cxxopts::value<string>()->default_value("AStar"))
            ("n, agents", "Number of agents", cxxopts::value<int>())
            ("outfile", "Output file that will be filled with result data", cxxopts::value<std::string>())
            ;

    auto result = options.parse(argc, argv);

    enum Algo {
        ASTAR,
        CBS,
        COOP,
        ID,
        SID,
    };

    std::map<string, Algo> mAlgo({
        {"AStar", ASTAR},
        {"CBS", CBS},
        {"Coop", COOP},
        {"ID", ID},
        {"SID", SID}
    });

    if (!mAlgo.contains(result["algo"].as<string>())) {
        printf("Given algo is invalid, use -h or --help");
        exit(0);
    }
    Algo algo = mAlgo[result["algo"].as<string>()];

    std::map<string, TypeOfHeuristic> mHeuristic({
        {"Optimal", OptimalDistance},
        {"Manhattan", Manhattan}
    });

    if (!mHeuristic.contains(result["heuristic"].as<string>())) {
        printf("Given heuristic is invalid, use -h or --help");
        exit(0);
    }

    TypeOfHeuristic heuristic = mHeuristic[result["heuristic"].as<string>()];

    std::map<string, ObjectiveFunction> mObjective({
        {"Fuel", Fuel},
        {"Makespan", Makespan},
        {"SumOfCosts", SumOfCosts}
    });

    if (!mObjective.contains(result["objective"].as<string>())) {
        printf("Given objective is invalid, use -h or --help");
        exit(0);
    }

    ObjectiveFunction objective = mObjective[result["objective"].as<string>()];

    // Getting filename and checking map file extension
    auto file = std::filesystem::path(result["map"].as<std::string>());
    if (file.extension() != ".map") {
        std::cout << "Not a .map file" << std::endl;
        exit(0);
    }

    string map_filename = file.filename();

    // Parsing the graph from the map file
    auto g = Parser::parse(file.c_str());
    std::shared_ptr<Solution> solution;
    int w;

    auto scenfile = std::filesystem::path(result["scen"].as<std::string>());
    if (scenfile.extension() != ".scen") {
        std::cout << "Not a .scen file" << std::endl;
        exit(0);
    }

    int agents = INT_MAX;
    if (result.count("n")) {
        agents = result["n"].as<int>();
    }

    std::ifstream infile;
    infile.open(scenfile.c_str());
    string line;
    getline(infile, line);
    std::vector<int> starts, targets;
    int n, h, sx, sy, tx, ty;
    double d;
    std::string map;
    int count = 0;
    while (getline(infile, line)) {
        //region extract data from line
        std::stringstream ss;
        ss.str(line);
        std::string item;
        getline(ss, item, '\t');
        n = stoi(item);
        getline(ss, item, '\t');
        if (map_filename != item) {
            printf("The map file doesn't correspond to the scen file");
            exit(0);
        }
        getline(ss, item, '\t');
        w = stoi(item);
        getline(ss, item, '\t');
        h = stoi(item);
        getline(ss, item, '\t');
        sx = stoi(item);
        getline(ss, item, '\t');
        sy = stoi(item);
        getline(ss, item, '\t');
        tx = stoi(item);
        getline(ss, item, '\t');
        ty = stoi(item);
        //endregion

        int start = sy*w+sx;
        starts.emplace_back(start);
        int target = ty*w+tx;
        targets.emplace_back(target);

        count += 1;
        if (count == agents) {
            break;
        }
    }

    auto problem = std::make_shared<MultiAgentProblemWithConstraints>(g, starts, targets, objective);

    switch (algo) {
        case ASTAR:
            solution = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(problem, heuristic).solve();
            break;
        case CBS:
            solution = ConflictBasedSearch(problem, heuristic).solve();
            break;
        case COOP:
            solution = CooperativeAStar(problem, heuristic).solve();
            break;
        case ID:
            solution = IndependenceDetection(problem, heuristic).solve();
            break;
        case SID:
            solution = SimpleIndependenceDetection(problem, heuristic).solve();
            break;
    }

    if (result.count("outfile")) {
        solution->write(result["outfile"].as<std::string>(), w);
    }
}