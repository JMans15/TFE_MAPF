//
// Created by Arthur Mahy on 09/11/2022.
//

#include "Node.h"

#include <utility>

Node::Node(State* m_state) {
    state = m_state;
    parent = nullptr;
    gn = 0;
}

Node::Node(State* m_state, const Node& m_parent, int m_gn) {
    state = m_state;
    parent = new Node(m_parent);
    gn = m_gn;
}

Node::~Node() {
    //if (parent != nullptr){
        // delete parent;
    //}
}

State* Node::getState() {
    return state;
}

Node *Node::getParent() const {
    return parent;
}

int Node::getGn() const {
    return gn;
}
