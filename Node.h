//
// Created by Arthur Mahy on 09/11/2022.
//

#ifndef TFE_MAPF_NODE_H
#define TFE_MAPF_NODE_H

#include "State.h"
#include <vector>
#include <string>
using namespace std;


class Node {
public:

    explicit Node(State m_state);
    Node(State m_state, const Node& m_parent, string m_action, int m_gn); //Constructeur surchargé
    ~Node(); //Destructeur
    vector<string> getPath();
    State getState();
    int getGn() const;
    // 2 nodes are equal if they have the same state
    bool operator== (Node other) const
    {
        return other.getState()==state;
    }

private:
    State state; // state of the node
    string action; // action to get to this node
    Node* parent; // parent node
    int gn; // path cost g(n)
};


#endif //TFE_MAPF_NODE_H
