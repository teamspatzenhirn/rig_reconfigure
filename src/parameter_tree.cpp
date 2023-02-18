/**
 * @file   parameter_tree.cpp
 * @author Dominik Authaler
 * @date   24.01.2022
 *
 * Utility class for organizing parameters as a tree according to their prefix.
 */

#include "parameter_tree.hpp"

#include <cstring>

// declaration of utility functions
bool recursivelyRemoveEmptySubgroups(const std::shared_ptr<ParameterGroup> &curNode);
std::string toUpperCase(const std::string &string);

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

    add(nextNode, ROSParameter(remainingName, parameter.value));
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

    // convert search string to upper case letters in order to search case-insensitive
    auto upperCaseSearchString = toUpperCase(filterString);

    // first pass: filter all parameters
    filter(filteredTree.getRoot(), root, upperCaseSearchString, "");

    // second pass: remove empty subgroups (multiple passes since our tree has no parent pointer)
    filteredTree.removeEmptySubgroups();


    return filteredTree;
}

void ParameterTree::filter(const std::shared_ptr<ParameterGroup> &destinationNode,
                           const std::shared_ptr<ParameterGroup> &sourceNode,
                           const std::string &filterString,
                           const std::string &prefix) const {
    if (destinationNode == nullptr || sourceNode == nullptr) {
        return;
    }

    // filter parameters
    for (const auto &parameter : sourceNode->parameters) {
        auto fullParameterName = prefix + '/' + toUpperCase(parameter.name);
        const auto pos = fullParameterName.find(filterString);
        if (pos != std::string::npos) {
            // we need to realign the position of the pattern because the parameter stores only the name (without
            // the prefix) + the pattern could be contained in the prefix and the parameter
            auto searchPatternPos = toUpperCase(parameter.name).find(filterString);

            if (searchPatternPos != std::string::npos) {
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
        auto newPrefix = prefix + '/' + toUpperCase(subgroup->prefix);
        destinationNode->subgroups.push_back(std::make_shared<ParameterGroup>(subgroup->prefix));

        auto searchPatternPos = toUpperCase(subgroup->prefix).find(filterString);
        if (searchPatternPos != std::string::npos) {
            destinationNode->subgroups.back()->prefixSearchPatternStart = searchPatternPos;
            destinationNode->subgroups.back()->prefixSearchPatternEnd = searchPatternPos + filterString.length();
        }

        filter(destinationNode->subgroups.back(), subgroup, filterString, newPrefix);
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

std::string toUpperCase(const std::string &string) {
    std::string upper;
    upper.reserve(string.size());

    for (const auto c : string) {
        upper.push_back(toupper(c));
    }

    return upper;
}