//
// Created by mansj on 10/11/22.
//

#ifndef MAPF_REBORN_PARSER_H
#define MAPF_REBORN_PARSER_H

#include "Graph.h"

#include <fstream>

class Parser {
public:
    static std::shared_ptr<Graph> parse(std::string filename);

private:
    static int tocellno(int l, int c, int w);
};

#endif //MAPF_REBORN_PARSER_H
