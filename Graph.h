//
// Created by mansj on 10/11/22.
//

#ifndef MAPF_REBORN_GRAPH_H
#define MAPF_REBORN_GRAPH_H

#include <vector>
#include <fstream>

using std::string;
using std::vector;

class Graph {
private:
    vector<int> *adjlists;
    int N, E, width;

public:
    Graph(int N, int Width);
    int getN() const;
    int getWidth() const;
    void add_edge(int from, int to);
    vector<int> getneighbors(int index) const;
    void print();
    void print(int num);
};


#endif //MAPF_REBORN_GRAPH_H
