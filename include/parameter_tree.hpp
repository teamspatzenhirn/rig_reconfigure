/**
 * @file   parameter_tree.hpp
 * @author Dominik Authaler
 * @date   24.01.2022
 *
 * Utility class for organizing parameters as a tree according to their prefix.
 */

#ifndef RIG_RECONFIGURE_PARAMETER_TREE_HPP
#define RIG_RECONFIGURE_PARAMETER_TREE_HPP

#include <memory>
#include "responses.hpp"

struct ParameterGroup {
    explicit ParameterGroup(std::string prefix = "") : prefix(std::move(prefix)) {};

    std::string prefix;

    std::vector<ROSParameter> parameters;
    std::vector<std::shared_ptr<ParameterGroup>> subgroups;
};

class ParameterTree {
  public:
    ParameterTree();

    void add(const ROSParameter &parameter);
    void clear();
    std::shared_ptr<ParameterGroup> getRoot();
    std::size_t getMaxParamNameLength() const;


  private:
    void add(const std::shared_ptr<ParameterGroup>& curNode, const ROSParameter& parameter);

    std::shared_ptr<ParameterGroup> root = nullptr;

    // bookkeeping for a nicer visualization
    std::size_t maxParamNameLength = 0;
};

#endif // RIG_RECONFIGURE_PARAMETER_TREE_HPP
