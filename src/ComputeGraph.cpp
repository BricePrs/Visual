//
// Created by brice on 10/6/25.
//

#include "ComputeGraph.h"

ComputeNode::ComputeNode(Texture texture) {
}

ComputeNode::~ComputeNode() {
}

void ComputeNode::Apply(int &budget) {
    for (auto inputNode: _inputNodes) {
        if (!inputNode.second.HasFinished()) {
            inputNode.second.Apply(budget);
        }
    }
    while (budget > 0 && !HasFinished()) {

        // Dispatch here

        budget--;
    }
}

bool ComputeNode::HasFinished() {
    return true;
}

Texture ComputeNode::GetTexture() {
    return _nodeTexture;
}

void ComputeNode::AddInput(ComputeNode node, std::string name) {
    _inputNodes[name] = node;
}

void ComputeNode::BindTextures() {
}
