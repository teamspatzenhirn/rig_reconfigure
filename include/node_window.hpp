/**
 * @file   node_window.hpp
 * @author Dominik Authaler
 * @date   13.01.2024
 *
 * Code related to the node window within the graphical user interface.
 */

#ifndef RIG_RECONFIGURE_NODE_WINDOW_HPP
#define RIG_RECONFIGURE_NODE_WINDOW_HPP

#include "utils.hpp"
#include "service_wrapper.hpp"

/**
 * Renders the window for the node selection.
 * @param[in] windowName          Window name.
 * @param[in] nodeNames           List with the available nodes.
 * @param[in, out] serviceWrapper Service wrapper for issuing ROS requests.
 * @param[in, out] selectedNode   Full name of the currently selected node.
 * @param[in, out] status         Status.
 */
void renderNodeWindow(const char *windowName, const std::vector<std::string> &nodeNames,
                      ServiceWrapper &serviceWrapper, std::string &selectedNode, Status &status);

#endif //RIG_RECONFIGURE_NODE_WINDOW_HPP
