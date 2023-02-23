//
// Created by Arthur Mahy on 09/11/2022.
//

#ifndef TFE_MAPF_NODE_H
#define TFE_MAPF_NODE_H

#include "State.h"
#include <vector>
#include <string>
#include <memory>
using namespace std;


class Node {
public:

    explicit Node(shared_ptr<State> m_state);
    Node(shared_ptr<State> m_state, const Node& m_parent, int m_gn); //Constructeur surchargé
    ~Node(); //Destructeur
    shared_ptr<State> getState();
    shared_ptr<Node> getParent() const;
    int getGn() const;
    /*
    // 2 nodes are equal if they have the same state
    bool operator== (Node other) const
    {
        return *other.getState()==*state;
    }*/

private:
    shared_ptr<State> state; // state of the node
    shared_ptr<Node> parent; // parent node
    int gn; // path cost g(n)
};


#endif //TFE_MAPF_NODE_H
