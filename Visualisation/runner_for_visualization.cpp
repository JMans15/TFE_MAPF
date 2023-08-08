//
// Created by mansj on 31/01/23.
//

#include "../Solvers/AStar/AStar.h"
#include "../Solvers/ID/SimpleIndependenceDetection.h"
#include "../Solvers/ID/IndependenceDetection.h"
#include "../Problems/MultiAgentProblem.h"
#include "../GraphParser/Parser.h"
#include "../Problems/SingleAgentProblem.h"
#include "../Solution/Solution.h"

#include "../external-headers/cxxopts.hpp"

#include <fstream>

int main(int argc, const char** argv) {
    cxxopts::Options options("TFE_MAPF_visu", "Runner program for visualizing our MAPF results");

    options.add_options()
        ("map", "Path to file containing the map info", cxxopts::value<std::string>())
        ("scen", "Path to file containing the scenario info", cxxopts::value<std::string>())
        ("s, start", "Index of the starting position, only for single agent", cxxopts::value<int>())
        ("t, target", "Index of the target position, only for single agent", cxxopts::value<int>())
        ("v, verbose", "Verbose output", cxxopts::value<bool>()->default_value("false"))
        ("h, help", "Print help")
        ("m, multi", "Multi agent version")
        ("sid", "Simple Independence Detection")
        ("id", "Independence Detection")
        ("a, agents", "Number of agents", cxxopts::value<int>())
        ("outfile", "Output file that will be filled with result data", cxxopts::value<std::string>())
        ;

    auto result = options.parse(argc, argv);

    enum mode {
        SINGLE,
        MULTI
    };

    mode Mode;

    // region arguments logic
    if (result.count("m")) {
        Mode = MULTI;
        if (result.count("s") || result.count("t")) {
            std::cout << "Cant't use s or t in multi agent mode" << std::endl;
            exit(0);
        }
        if (!result.count("scen")) {
            std::cout << "Arg scen is mandatory in multi agent mode" << std::endl;
            exit(0);
        }
    }
    else {
        Mode = SINGLE;
        if (result.count("h")) {
            std::cout << options.help({""}) << std::endl;
            exit(0);
        }

        if (!result.count("s")) {
            std::cout << "Arg start is mandatory in single agent mode" << std::endl;
            exit(0);
        }

        if (!result.count("t")) {
            std::cout << "Arg target is mandatory in single agent mode" << std::endl;
            exit(0);
        }

        if (result.count("scen")) {
            std::cout << "Can't use arg scen in single agent mode" << std::endl;
            exit(0);
        }
    }
    if (!result.count("map")) {
        std::cout << "Arg map is mandatory" << std::endl;
        exit(0);
    }
    // endregion

    // Getting filename and checking map file extension
    auto file = result["map"].as<std::string>();
    if (file.substr(file.find_last_of('.') + 1) != "map") {
        std::cout << "Not a map file" << std::endl;
        exit(0);
    }

    // Parsing the graph from the map file
    auto g = Parser::parse(file);
    std::shared_ptr<Solution> solution;
    int w;

    if (Mode == MULTI) {
        auto scenfile = result["scen"].as<std::string>();
        if (scenfile.substr(scenfile.find_last_of('.') + 1) != "scen") {
            std::cout << "Not a scen file" << std::endl;
            exit(0);
        }

        int agents = INT_MAX;
        if (result.count("a")) {
            agents = result["a"].as<int>();
        }
        
        std::ifstream infile;
        infile.open(scenfile);
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

        auto problem = std::make_shared<MultiAgentProblem>(g, starts, targets, SumOfCosts);
        if (result.count("sid")) {
            auto solver = SimpleIndependenceDetection(problem, OptimalDistance);
            solution = solver.solve();
        }
        else if (result.count("id")) {
            auto solver = IndependenceDetection(problem, OptimalDistance);
            solution = solver.solve();
        }
        else {
            auto solver = GeneralAStar(problem, OptimalDistance);
            solution = solver.solve();
        }
    } else {
        // Getting start and target
        auto start = result["s"].as<int>();
        auto target = result["t"].as<int>();

        // Solving and printing problem
        auto problem = std::make_shared<SingleAgentProblem>(g, start, target);
        auto aStar = GeneralAStar(problem, Manhattan);
        solution = aStar.solve();
    }

    if (result.count("outfile")) {
        solution->write(result["outfile"].as<std::string>(), w);
    }
    //solution.print();
}