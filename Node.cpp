//
// Created by Arthur Mahy on 09/11/2022.
//

#include "Node.h"

#include <utility>

Node::Node(shared_ptr<State> m_state) {
    state = std::move(m_state);
    parent = nullptr;
    gn = 0;
}

Node::Node(shared_ptr<State> m_state, const Node& m_parent, int m_gn) {
    state = std::move(m_state);
    parent = make_shared<Node>(m_parent);
    gn = m_gn;
}

Node::~Node() = default;

shared_ptr<State> Node::getState() {
    return state;
}

shared_ptr<Node> Node::getParent() const {
    return parent;
}

int Node::getGn() const {
    return gn;
}
