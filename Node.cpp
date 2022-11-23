//
// Created by Arthur Mahy on 09/11/2022.
//

#include "Node.h"

#include <utility>

Node::Node(State m_state) : state(std::move(m_state)) {
    parent = nullptr;
    action = "";
    gn = 0;
}

Node::Node(State m_state, const Node& m_parent, string m_action, int m_gn) : state(std::move(m_state)) {
    parent = new Node(m_parent);
    action = std::move(m_action);
    gn = m_gn;
}

Node::~Node() {
    //if (parent != nullptr){
        // delete parent;
    //}
}

vector<string> Node::getPath() {
    vector<string> path;
    Node* node = this;
    while (node->parent != nullptr){
        path.push_back(node->action);
        node = node->parent;
    }
    reverse(path.begin(), path.end());
    return path;
}

State Node::getState() {
    return state;
}

string Node::getAction() const {
    return action;
}

Node *Node::getParent() const {
    return parent;
}

int Node::getGn() const {
    return gn;
}
