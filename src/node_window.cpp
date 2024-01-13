/**
 * @file   node_window.cpp
 * @author Dominik Authaler
 * @date   13.01.2024
 *
 * Code related to the node window within the graphical user interface.
 */

#include "node_window.hpp"

#include <imgui.h>
#include <string>

void renderNodeWindow(const char *windowName, const std::vector<std::string> &nodeNames,
                      ServiceWrapper &serviceWrapper, int &nodeNameIndex, Status &status) {
    ImGui::Begin(windowName);

    if (nodeNames.empty()) {
        ImGui::Text("No nodes available!");
    } else {
        ImGui::Text("Available nodes:");

        if (ImGui::BeginListBox("##Nodes", ImVec2(-1, 500))) {
            for (auto i = 0U; i < nodeNames.size(); ++i) {
                const bool isSelected = (nodeNameIndex == i);
                if (ImGui::Selectable(nodeNames[i].c_str(), isSelected)) {
                    nodeNameIndex = i;
                }
            }
            ImGui::EndListBox();
        }

        if (status.type == Status::Type::NO_NODES_AVAILABLE) {
            status.text.clear();
            status.type = Status::Type::NONE;
        }
    }

    if (ImGui::Button("Refresh")) {
        serviceWrapper.pushRequest(std::make_shared<Request>(Request::Type::QUERY_NODE_NAMES));

        if (status.type == Status::Type::SERVICE_TIMEOUT) {
            status.text.clear();
            status.type = Status::Type::NONE;
        }
    }

    ImGui::End();
}
