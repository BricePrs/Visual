//
// Created by brice on 10/6/25.
//

#ifndef VISUAL_COMPUTEGRAPH_H
#define VISUAL_COMPUTEGRAPH_H
#include <unordered_map>
#include <vector>

#include "Texture.h"

class ComputeNode {
public:

    explicit ComputeNode(Texture texture); // How to handle pingpong ? Cycle allowed then Return true to HasFinished ? Class inheriting ComputeNode containing sub graphs ?
    ~ComputeNode();

    virtual void Apply(int &budget);
    virtual bool HasFinished();

    Texture GetTexture();

    void AddInput(ComputeNode node, std::string name);

private:

    void BindTextures();

    int frameApplyBudget = 1;
    std::unordered_map<std::string, ComputeNode> _inputNodes;

    Texture _nodeTexture;
};


#endif //VISUAL_COMPUTEGRAPH_H