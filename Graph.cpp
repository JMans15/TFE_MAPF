//
// Created by mansj on 10/11/22.
//

#include <sstream>
#include <iostream>
#include "Graph.h"

using std::stringstream;
using std::endl;
using std::cout;
using std::min;

Graph::Graph(int N) : E(0), N(N) {
    adjlists = new vector<int>[N];
}

void Graph::add_edge(int from, int to) {
    adjlists[from].push_back(to);
    adjlists[to].push_back(from);
    E++;
}

void Graph::print() {
    print(N);
}

void Graph::print(int num) {
    stringstream res;
    res << "Graph has " << N << " vertices and " << E << " edges." << endl;
    if (num < N) res << "Printing first " << num << " vertices." << endl;
    for (int n = 0; n < min(num, N); n++) {
        if (adjlists[n].empty()) continue;
        res << "Vertex " << n << ": [";
        for (int i : adjlists[n]) {
            res << i << ", ";
        }
        res .seekp(-2, stringstream::cur);
        res << "]" << endl;
    }
    cout << res.str();
}

vector<int> Graph::getneighbors(int index) {
    if (index > N) return {};
    return adjlists[index];
}

int Graph::getN() const {
    return N;
}
