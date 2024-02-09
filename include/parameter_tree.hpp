/**
 * @file   parameter_tree.hpp
 * @author Dominik Authaler
 * @date   24.01.2023
 *
 * Utility class for organizing parameters as a tree according to their prefix.
 */

#ifndef RIG_RECONFIGURE_PARAMETER_TREE_HPP
#define RIG_RECONFIGURE_PARAMETER_TREE_HPP

#include <memory>
#include <optional>
#include <utility>
#include "responses.hpp"

/**
 * Extension of the ROS parameter struct, adding information which is relevant
 * for the parameter tree.
 */
struct TreeElement {
    TreeElement(const ROSParameter &parameter_, std::string fullParameterPath_,
                std::optional<std::size_t> patternStart_ = std::nullopt,
                std::optional<std::size_t> patternEnd_ = std::nullopt) :
        name(parameter_.name), fullPath(std::move(fullParameterPath_)), value(parameter_.value),
        searchPatternStart(patternStart_), searchPatternEnd(patternEnd_) {};

    TreeElement(std::string name_, std::string fullParameterPath_, ROSParameterVariant value_,
                std::optional<std::size_t> patternStart_ = std::nullopt,
                std::optional<std::size_t> patternEnd_ = std::nullopt) :
        name(std::move(name_)), fullPath(std::move(fullParameterPath_)), value(std::move(value_)),
        searchPatternStart(patternStart_), searchPatternEnd(patternEnd_) {};

    std::string name; // parameter name without prefixes

    // in addition to the name we store the full path of the parameter in the leaf nodes of the tree in order to be
    // able to support the mixing of different separators
    std::string fullPath;

    ROSParameterVariant value;

    // in case this parameter is part of a filtered parameter tree the following two members store the intermediate
    // information where in the name (not the fullPath!) the applied search pattern is located
    std::optional<std::size_t> searchPatternStart;
    std::optional<std::size_t> searchPatternEnd;
};

struct ParameterGroup {
    explicit ParameterGroup(std::string prefix_ = "") : prefix(std::move(prefix_)) {};

    std::string prefix;
    std::optional<std::size_t> prefixSearchPatternStart;
    std::optional<std::size_t> prefixSearchPatternEnd;

    std::vector<TreeElement> parameters;
    std::vector<std::shared_ptr<ParameterGroup>> subgroups;
};

class ParameterTree {
  public:
    ParameterTree();

    void add(const ROSParameter &parameter);
    void clear();
    [[nodiscard]] std::shared_ptr<ParameterGroup> getRoot();
    [[nodiscard]] std::size_t getMaxParamNameLength() const;
    [[nodiscard]] std::string getAppliedFilter() const;

    [[nodiscard]] ParameterTree filter(const std::string &filterString) const;

    void removeEmptySubgroups();


  private:
    void add(const std::shared_ptr<ParameterGroup> &curNode, const TreeElement &parameter);

    void filter(const std::shared_ptr<ParameterGroup> &destinationNode,
                const std::shared_ptr<ParameterGroup> &sourceNode,
                const std::string &filterString) const;

    std::shared_ptr<ParameterGroup> root = nullptr;

    // bookkeeping for a nicer visualization
    std::size_t maxParamNameLength = 0;

    std::string appliedFilter;
};

#endif // RIG_RECONFIGURE_PARAMETER_TREE_HPP
