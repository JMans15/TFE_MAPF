//
// Created by mansj on 10/11/22.
//

#ifndef MAPF_REBORN_GRAPH_H
#define MAPF_REBORN_GRAPH_H

#include <vector>
#include <fstream>
#include <memory>

using std::string;
using std::vector;

class Graph {
private:
    vector<int> *adjlists;
    int N, E, width;

public:
    Graph(int N, int Width);
    ~Graph();
    int getN() const;
    int getWidth() const;
    void addEdge(int from, int to);
    vector<int> getNeighbors(int index) const;
    void print();
    void print(int num);
};


#endif //MAPF_REBORN_GRAPH_H
