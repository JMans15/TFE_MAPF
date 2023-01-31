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
        ("f, file", "Path to file containing the map info", cxxopts::value<std::string>())
        ("s, start", "Index of the starting position", cxxopts::value<int>())
        ("t, target", "Index of the target position", cxxopts::value<int>())
        ("v, verbose", "Verbose output", cxxopts::value<bool>()->default_value("false"))
        ("h, help", "Print help")
        ;

    auto result = options.parse(argc, argv);

    // region Checking for mandatory arguments
    if (result.count("h")) {
        std::cout << options.help({""}) << std::endl;
        exit(0);
    }

    if (!result.count("f")) {
        std::cout << "Arg file is mandatory" << std::endl;
        exit(0);
    }

    if (!result.count("s")) {
        std::cout << "Arg start is mandatory" << std::endl;
        exit(0);
    }

    if (!result.count("t")) {
        std::cout << "Arg target is mandatory" << std::endl;
        exit(0);
    }
    // endregion

    // Getting filename and checking map file extension
    auto file = result["f"].as<std::string>();
    if (file.substr(file.find_last_of('.') + 1) != "map") {
        std::cout << "Not a map file" << std::endl;
        exit(0);
    }

    // Parsing the graph from the map file
    Graph g = parser::parse(file);

    // Getting start and target
    auto start = result["s"].as<int>();
    auto target = result["t"].as<int>();

    // Solving and printing problem
    SingleAgentProblem problem = SingleAgentProblem(g, start, target);
    Solution solution = aStarSearch(&problem, Manhattan);
    solution.print();
}
