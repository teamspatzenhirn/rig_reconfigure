/**
 * @file   parameter_window.hpp
 * @author Dominik Authaler
 * @date   13.01.2024
 *
 * Code related to the parameter window within the graphical user interface.
 */

#include "parameter_window.hpp"

#include <imgui_internal.h> // necessary for the tree node manipulation
#include <misc/cpp/imgui_stdlib.h>
#include <set>

#include "utils.hpp"

/// Minimum width specified for text input fields.
constexpr auto MIN_INPUT_TEXT_FIELD_WIDTH = 100;
/// Width of the window reserved for padding (e.g. between parameter name and input field) in case the width of the
/// input field is scaled using the window width.
constexpr auto TEXT_INPUT_FIELD_PADDING = 100;
/// Reduction of the text field width per nesting level (necessary to avoid input field spanning across the window
/// borders)
constexpr auto TEXT_FIELD_WIDTH_REDUCTION_PER_LEVEL = 22;
constexpr auto FILTER_INPUT_TEXT_FIELD_WIDTH = 250;
constexpr auto FILTER_HIGHLIGHTING_COLOR = ImVec4(1, 0, 0, 1);
constexpr auto TEXT_INPUT_EDITING_END_CHARACTERS = "\n";

static std::set<ImGuiID> visualizeParameters(ServiceWrapper &serviceWrapper,
                                             const std::shared_ptr<ParameterGroup> &parameterNode,
                                             std::size_t maxParamLength,
                                             const std::string &filterString, bool expandAll = false);

void renderParameterWindow(const char *windowName, const std::string &curSelectedNode,
                           ServiceWrapper &serviceWrapper, ParameterTree &filteredParameterTree, std::string &filter,
                           Status &status) {
    // unfortunately DearImGui doesn't provide any option to collapse tree nodes recursively, hence, we need to keep
    // track of the ID of each open tree node (across function calls, since a tree node stays open even if
    // the parent node is closed)
    static std::set<ImGuiID> treeNodeIDs;
    static std::string displayedNodeName;

    if (displayedNodeName != curSelectedNode) {
        displayedNodeName = curSelectedNode;
        treeNodeIDs.clear();
    }

    ImGui::Begin(windowName);

    bool expandAllParameters = false;

    const auto curWindowWidth = static_cast<int>(ImGui::GetWindowSize().x);

    if (!curSelectedNode.empty()) {
        ImGui::Text("Parameters of '%s'", curSelectedNode.c_str());
        ImGui::Dummy(ImVec2(0.0F, 5.0F));

        if (ImGui::Button("Reload parameters")) {
            serviceWrapper.pushRequest(std::make_shared<Request>(Request::Type::QUERY_NODE_PARAMETERS));
        }

        ImGui::SameLine();

        if (ImGui::Button("Expand all")) {
            expandAllParameters = true;
        }

        ImGui::SameLine();

        if (ImGui::Button("Collapse all")) {
            for (const auto id : treeNodeIDs) {
                ImGui::TreeNodeSetOpen(id, false);
            }
            treeNodeIDs.clear();
        }

        ImGui::Dummy(ImVec2(0.0F, 10.0F));

        ImGui::AlignTextToFramePadding();
        ImGui::Text("Filter: ");
        ImGui::SameLine();
        ImGui::PushItemWidth(FILTER_INPUT_TEXT_FIELD_WIDTH);
        ImGui::InputText("##Filter", &filter, ImGuiInputTextFlags_CharsNoBlank);
        ImGui::PopItemWidth();
        ImGui::SameLine();
        if (ImGui::Button("Clear")) {
            filter = "";
        }

        ImGui::Dummy(ImVec2(0.0F, 10.0F));

        const auto maxParamLength = filteredParameterTree.getMaxParamNameLength();
        const auto ids = visualizeParameters(serviceWrapper, filteredParameterTree.getRoot(),
                                             maxParamLength, filteredParameterTree.getAppliedFilter(),
                                             expandAllParameters);
        treeNodeIDs.insert(ids.begin(), ids.end());

        if (status.type == Status::Type::NO_NODES_AVAILABLE) {
            status.text.clear();
            status.type = Status::Type::NONE;
        }
    } else {
        ImGui::Text("Please select a node first!");
    }

    ImGui::End();
}

std::string getFormatStringFromStep(double step, int max_digits = 10) {
    const double epsilon = 1e-12;  // tolerance for rounding error
    int digits = 0;
    double scaled = step;
    while (digits < max_digits && std::abs(scaled - std::round(scaled)) > epsilon) {
        scaled *= 10.0;
        ++digits;
    }
    return "%." + std::to_string(digits) + "f";
}

std::set<ImGuiID> visualizeParameters(ServiceWrapper &serviceWrapper,
                                      const std::shared_ptr<ParameterGroup> &parameterNode,
                                      const std::size_t maxParamLength,
                                      const std::string &filterString,
                                      const bool expandAll) {
    // required to store which of the text input fields is 'dirty' (has changes which have not yet been propagated to
    // the ROS service (because editing has not yet been finished))
    // --> since ImGui only allows a single active input field storing the path of the corresponding parameter is enough
    static std::string dirtyTextInput;

    std::set<ImGuiID> nodeIDs;
    auto *window = ImGui::GetCurrentWindow();

    if (parameterNode == nullptr || (parameterNode->parameters.empty() && parameterNode->subgroups.empty())) {
        if (!filterString.empty()) {
            ImGui::Text("This node doesn't seem to have any parameters\nmatching the filter!");
        } else {
            ImGui::Text("This node doesn't seem to have any parameters!");
        }

        return {};
    }

    for (auto &[name, descriptor, fullPath, value, highlightingStart, highlightingEnd] : parameterNode->parameters) {
        std::string identifier = "##" + name;

        // simple 'space' padding to avoid the need for a more complex layout with columns (the latter is still desired
        // :D)
        std::string padding;
        if (name.length() < maxParamLength) {
            padding = std::string(maxParamLength - name.length(), ' ');
        }

        ImGui::AlignTextToFramePadding();

        if (highlightingStart.has_value() && highlightingEnd.has_value()) {
            highlightedText(name, highlightingStart.value(), highlightingEnd.value(), FILTER_HIGHLIGHTING_COLOR);
        } else {
            ImGui::Text("%s", name.c_str());
        }

        ImGui::SameLine(0, 0);
        ImGui::Text("%s", padding.c_str());

        ImGui::SameLine();
        ImGui::PushItemWidth(-FLT_MIN);

        if (descriptor.read_only) {
            ImGui::BeginDisabled();
        }

        if (std::holds_alternative<double>(value)) {
            if (!descriptor.floating_point_range.empty()) {
                double* min = &descriptor.floating_point_range[0].from_value;
                double* max = &descriptor.floating_point_range[0].to_value;
                double step = std::fabs(descriptor.floating_point_range[0].step);
                std::string format = "%.6g";
                if (step != 0) {
                    format = getFormatStringFromStep(step);
                }
                if(ImGui::SliderScalar(identifier.c_str(), ImGuiDataType_Double, 
                                    &std::get<double>(value), min, max, format.c_str(), 
                                    ImGuiSliderFlags_AlwaysClamp) && step != 0) {
                    std::get<double>(value) = std::round((std::get<double>(value) - *min) / step) * step + *min;
                }
            }
            else {
                ImGui::DragScalar(identifier.c_str(), ImGuiDataType_Double, &std::get<double>(value), 
                                  1.0, nullptr, nullptr, "%.6g");
            }
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                serviceWrapper.pushRequest(
                        std::make_shared<ParameterModificationRequest>(ROSParameter(fullPath, value)));
            }
        } else if (std::holds_alternative<bool>(value)) {
            if (ImGui::Checkbox(identifier.c_str(), &std::get<bool>(value))) {
                serviceWrapper.pushRequest(
                        std::make_shared<ParameterModificationRequest>(ROSParameter(fullPath, value)));
            }
        } else if (std::holds_alternative<int>(value)) {
            if (!descriptor.integer_range.empty()) {
                int min = descriptor.integer_range[0].from_value;
                int max = descriptor.integer_range[0].to_value;
                int step = descriptor.integer_range[0].step;

                if(ImGui::SliderInt(identifier.c_str(), &std::get<int>(value), min, max) && step != 0) {
                    std::get<int>(value) = static_cast<int>(std::round((std::get<int>(value) - min) / step) * step + min);
                }
            }
            else {
                ImGui::DragInt(identifier.c_str(), &std::get<int>(value));
            }
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                serviceWrapper.pushRequest(
                        std::make_shared<ParameterModificationRequest>(ROSParameter(fullPath, value)));
            }
        } else if (std::holds_alternative<std::string>(value)) {
            // Set to true when enter is pressed
            bool flush = false;

            // Note: ImGui provides an option to provide only callbacks on enter, but we additionally need the
            //       information whether the text field is 'dirty', hence, we need to check for 'enter'
            //       by ourselves
            if (ImGui::InputText(identifier.c_str(), &std::get<std::string>(value))) {
                dirtyTextInput = fullPath;

                auto &str = std::get<std::string>(value);

                // check if last character indicates the end of the editing
                if (str.ends_with(TEXT_INPUT_EDITING_END_CHARACTERS)) {
                    flush = true;
                    str.pop_back();
                }
            }

            // Second condition: InputText focus lost
            if (flush || (!ImGui::IsItemActive() && dirtyTextInput == fullPath)) {
                dirtyTextInput.clear();
                serviceWrapper.pushRequest(
                        std::make_shared<ParameterModificationRequest>(ROSParameter(fullPath, value)));
            }
        }

        if (descriptor.read_only) {
            ImGui::EndDisabled();
        }

        ImGui::PopItemWidth();
    }

    if (!parameterNode->subgroups.empty()) {
        for (const auto &subgroup : parameterNode->subgroups) {
            if (expandAll) {
                ImGui::SetNextItemOpen(true);
            }

            const auto label = "##" + subgroup->prefix;

            // this is hacky, we need the ID of the TreeNode in order to access the memory for collapsing it
            const auto nodeID = window->GetID(label.c_str());
            nodeIDs.insert(nodeID);

            bool open = ImGui::TreeNode(label.c_str());

            ImGui::SameLine();
            bool textClicked = false;
            if (subgroup->prefixSearchPatternStart.has_value() && subgroup->prefixSearchPatternEnd.has_value()) {
                textClicked = highlightedSelectableText(subgroup->prefix, subgroup->prefixSearchPatternStart.value(),
                                                        subgroup->prefixSearchPatternEnd.value(),
                                                        FILTER_HIGHLIGHTING_COLOR);
            } else {
                textClicked = ImGui::Selectable((subgroup->prefix).c_str());
            }

            if (textClicked) {
                ImGui::TreeNodeSetOpen(nodeID, !open);
            }

            if (open) {
                auto subIDs = visualizeParameters(serviceWrapper, subgroup, maxParamLength,
                                                  filterString, expandAll);
                nodeIDs.insert(subIDs.begin(), subIDs.end());
                ImGui::TreePop();
            }
        }
    }

    return nodeIDs;
}
