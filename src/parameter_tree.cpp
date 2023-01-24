/**
 * @file   parameter_tree.cpp
 * @author Dominik Authaler
 * @date   24.01.2022
 *
 * Utility class for organizing parameters as a tree according to their prefix.
 */

#include "parameter_tree.hpp"

ParameterTree::ParameterTree() : root(std::make_shared<ParameterGroup>()) {

}

void ParameterTree::add(const ROSParameter &parameter) {
    add(root, parameter);
}
void ParameterTree::clear() {
    root = std::make_shared<ParameterGroup>();
}

void ParameterTree::add(const std::shared_ptr<ParameterGroup> &curNode, const ROSParameter &parameter) {
    auto prefixStart = parameter.name.find('/');
    if (prefixStart == std::string::npos) {
        curNode->parameters.emplace_back(parameter);
        return;
    }

    // extract first prefix and find corresponding node
    auto prefix = parameter.name.substr(0, prefixStart);
    auto remainingName = parameter.name.substr(prefixStart + 1);

    std::shared_ptr<ParameterGroup> nextNode = nullptr;
    for (const auto &subgroup : curNode->subgroups) {
        if (subgroup->prefix == prefix) {
            nextNode = subgroup;
            break;
        }
    }

    if (nextNode == nullptr) {
        nextNode = std::make_shared<ParameterGroup>(prefix);
        curNode->subgroups.emplace_back(nextNode);
    }

    add(nextNode, ROSParameter(remainingName, parameter.value));
}

std::shared_ptr<ParameterGroup> ParameterTree::getRoot() {
    return root;
}