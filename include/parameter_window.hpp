/**
 * @file   parameter_window.hpp
 * @author Dominik Authaler
 * @date   13.01.2024
 *
 * Code related to the parameter window within the graphical user interface.
 */

#ifndef RIG_RECONFIGURE_PARAMETER_WINDOW_HPP
#define RIG_RECONFIGURE_PARAMETER_WINDOW_HPP

#include <imgui.h>
#include <string>

#include "parameter_tree.hpp"
#include "service_wrapper.hpp"
#include "utils.hpp"

/**
 * Renders the window for the parameter modification.
 * @param[in] windowName                 Window name.
 * @param[in] curSelectedNode            Name of the currently selected node.
 * @param[in, out] serviceWrapper        Service wrapper for issuing ROS requests.
 * @param[in, out] filteredParameterTree Filtered parameter tree.
 * @param[in, out] filter                Filter string input.
 * @param[in, out] status                Status for displaying errors.
 */
void renderParameterWindow(const char *windowName, const std::string &curSelectedNode,
                           ServiceWrapper &serviceWrapper, ParameterTree &filteredParameterTree,
                           std::string &filter, Status &status);

#endif //RIG_RECONFIGURE_PARAMETER_WINDOW_HPP
