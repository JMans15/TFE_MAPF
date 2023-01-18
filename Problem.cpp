//
// Created by Arthur Mahy on 09/11/2022.
//

#include "Problem.h"

// #include <bits/stdc++.h>
#include <utility>

Problem::Problem(Graph m_graph) : graph(m_graph) {}

Graph Problem::getGraph() const {
    return graph;
}

int Problem::getNumberOfAgents() const {
    return numberOfAgents;
}
