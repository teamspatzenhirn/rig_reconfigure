/**
 * @file   parameter_tree.cpp
 * @author Dominik Authaler
 * @date   24.01.2022
 *
 * Utility class for organizing parameters as a tree according to their prefix.
 */

#include "parameter_tree.hpp"

#include <algorithm>

constexpr auto SEPARATORS = "/.\\";

// declaration of utility functions
bool recursivelyRemoveEmptySubgroups(const std::shared_ptr<ParameterGroup> &curNode);
std::size_t findCaseInsensitive(const std::string &string, const std::string &pattern);

ParameterTree::ParameterTree() : root(std::make_shared<ParameterGroup>()) {

}

void ParameterTree::add(const ROSParameter &parameter) {
    add(root, TreeElement(parameter, parameter.name));
}
void ParameterTree::clear() {
    root = std::make_shared<ParameterGroup>();
}

void ParameterTree::add(const std::shared_ptr<ParameterGroup> &curNode, const TreeElement &parameter) {
    auto prefixStart = parameter.name.find_first_of(SEPARATORS);
    if (prefixStart == std::string::npos) {
        curNode->parameters.emplace_back(parameter);
        maxParamNameLength = std::max(maxParamNameLength, parameter.name.length());
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

    add(nextNode, TreeElement(remainingName, parameter.fullPath, parameter.value));
}

std::shared_ptr<ParameterGroup> ParameterTree::getRoot() {
    return root;
}

std::size_t ParameterTree::getMaxParamNameLength() const {
    return maxParamNameLength;
}

ParameterTree ParameterTree::filter(const std::string &filterString) const {
    ParameterTree filteredTree;

    filteredTree.maxParamNameLength = maxParamNameLength;

    // first pass: filter all parameters
    filter(filteredTree.getRoot(), root, filterString);

    // second pass: remove empty subgroups
    filteredTree.removeEmptySubgroups();

    return filteredTree;
}

void ParameterTree::filter(const std::shared_ptr<ParameterGroup> &destinationNode,
                           const std::shared_ptr<ParameterGroup> &sourceNode,
                           const std::string &filterString) const {
    if (destinationNode == nullptr || sourceNode == nullptr) {
        return;
    }

    // filter parameters
    for (const auto &parameter : sourceNode->parameters) {

        const auto pos = findCaseInsensitive(parameter.fullPath, filterString);
        if (pos != std::string::npos) {
            // we need to realign the position of the pattern because start and end positions of the pattern
            // are defined as within the parameter name (not the full path with prefixes)
            auto searchPatternPos = findCaseInsensitive(parameter.name, filterString);

            if (searchPatternPos != std::string::npos && !filterString.empty()) {
                destinationNode->parameters.push_back(parameter);
                destinationNode->parameters.back().searchPatternStart = searchPatternPos;
                destinationNode->parameters.back().searchPatternEnd = searchPatternPos + filterString.length();
            } else {
                // search pattern was found in the prefix --> nothing to highlight within the parameter name
                destinationNode->parameters.push_back(parameter);
            }
        }
    }

    // filter subgroups
    for (const auto &subgroup : sourceNode->subgroups) {
        destinationNode->subgroups.push_back(std::make_shared<ParameterGroup>(subgroup->prefix));

        auto searchPatternPos = findCaseInsensitive(subgroup->prefix, filterString);
        if (searchPatternPos != std::string::npos && !filterString.empty()) {
            destinationNode->subgroups.back()->prefixSearchPatternStart = searchPatternPos;
            destinationNode->subgroups.back()->prefixSearchPatternEnd = searchPatternPos + filterString.length();
        }

        filter(destinationNode->subgroups.back(), subgroup, filterString);
    }
}

void ParameterTree::removeEmptySubgroups() {
    recursivelyRemoveEmptySubgroups(root);
}

bool recursivelyRemoveEmptySubgroups(const std::shared_ptr<ParameterGroup> &curNode) {
    bool empty = true;

    if (curNode == nullptr) {
        return true;
    }

    if (!curNode->parameters.empty()) {
        empty = false;
    }

    std::erase_if(curNode->subgroups, [](const std::shared_ptr<ParameterGroup> &group) {
        return recursivelyRemoveEmptySubgroups(group);
    });

    if (!curNode->subgroups.empty()) {
        empty = false;
    }

    return empty;
}

// based on the following stack overflow answer: https://stackoverflow.com/a/19839371
std::size_t findCaseInsensitive(const std::string &string, const std::string &pattern) {
    auto it = std::search(string.begin(), string.end(), pattern.begin(), pattern.end(),
                          [](unsigned char ch1, unsigned char ch2) { return std::toupper(ch1) == std::toupper(ch2); }
    );

    return (it != string.end()) ? std::distance(string.begin(), it) : std::string::npos;
}