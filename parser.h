//
// Created by mansj on 10/11/22.
//

#ifndef MAPF_REBORN_PARSER_H
#define MAPF_REBORN_PARSER_H
#include <fstream>
#include "Graph.h"

using std::string;

class parser {
public:
    static Graph parse(string filename);
private:
    static int tocellno(int l, int c, int w);
};


#endif //MAPF_REBORN_PARSER_H
