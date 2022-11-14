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
    int N, E;
public:
    int getN() const;

public:
    explicit Graph(int N);
    void add_edge(int from, int to);
    vector<int> getneighbors(int index);
    void print();
    void print(int num);
};


#endif //MAPF_REBORN_GRAPH_H
