//
// Created by mansj on 10/11/22.
//

#ifndef MAPF_REBORN_GRAPH_H
#define MAPF_REBORN_GRAPH_H

#include <fstream>
#include <memory>
#include <vector>

using std::string;
using std::vector;

class Graph {
public:
  Graph(int nVertices, int width);
  ~Graph();

  int getNumberOfVertices() const;
  int getWidth() const;
  void addEdge(int from, int to);
  const std::vector<int> &getNeighbors(int index) const;
  void print();
  void print(int num);

private:
  std::vector<std::vector<int>> neighbors;
  int nVertices, nEdges, width;
};

#endif // MAPF_REBORN_GRAPH_H
