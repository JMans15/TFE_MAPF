//
// Created by mansj on 10/11/22.
//

#include "Graph.h"

#include <iostream>
#include <sstream>

Graph::Graph(int nVertices, int width)
    : nVertices(nVertices), nEdges(0), width(width) {
  neighbors.resize(nVertices);
}

Graph::~Graph() {}

void Graph::addEdge(int from, int to) {
  neighbors[from].push_back(to);
  neighbors[to].push_back(from);
  nEdges++;
}

void Graph::print() { print(nVertices); }

void Graph::print(int num) {
  std::stringstream res;
  res << "Graph has " << nVertices << " vertices and " << nEdges << " edges."
      << std::endl;
  if (num < nVertices)
    res << "Printing first " << num << " vertices." << std::endl;
  for (int n = 0; n < std::min(num, nVertices); n++) {
    if (neighbors[n].empty())
      continue;
    res << "Vertex " << n << ": [";
    for (int i : neighbors[n]) {
      res << i << ", ";
    }
    res.seekp(-2, std::stringstream::cur);
    res << "]" << std::endl;
  }
  std::cout << res.str();
}

const std::vector<int> &Graph::getNeighbors(int index) const {
  if (index > nVertices)
    return {};
  return neighbors[index];
}

int Graph::getNumberOfVertices() const { return nVertices; }

int Graph::getWidth() const { return width; }
