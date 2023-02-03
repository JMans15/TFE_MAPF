//
// Created by mansj on 31/01/23.
//

#include "parser.h"
#include "library.h"
#include "SingleAgentProblem.h"
#include "external-headers/cxxopts.hpp"

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
            cout << "Cant't use s or t in multi agent mode" << endl;
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

    auto scenfile = result["scen"].as<std::string>();
    if (file.substr(file.find_last_of('.') + 1) != "scen") {
        std::cout << "Not a scen file" << std::endl;
        exit(0);
    }

    // Parsing the graph from the map file
    Graph g = parser::parse(file);

    if (Mode == MULTI) {
        cout << "Multi agent currently not supported, WIP" << endl;
    }

    // Getting start and target
    auto start = result["s"].as<int>();
    auto target = result["t"].as<int>();

    // Solving and printing problem
    SingleAgentProblem problem = SingleAgentProblem(g, start, target);
    Solution solution = aStarSearch(&problem, Manhattan);
    solution.print();
}
