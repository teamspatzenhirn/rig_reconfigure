/**
 * @file   rig_reconfigure.cpp
 * @author Dominik Authaler
 * @date   11.12.2022
 *
 * Dynamic reconfigure GUI via DearImgui.
 */


#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include <cstdio>
#include <vector>
#include <chrono>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_internal.h"
#include "misc/cpp/imgui_stdlib.h"
#include "parameter_tree.hpp"
#include "service_wrapper.hpp"

using namespace std::chrono_literals;

constexpr auto INPUT_TEXT_FIELD_WIDTH = 100;
constexpr auto FILTER_INPUT_TEXT_FIELD_WIDTH = 250;
constexpr auto FILTER_HIGHLIGHTING_COLOR = ImVec4(1, 0, 0, 1);
constexpr auto STATUS_WARNING_COLOR = ImVec4(1, 0, 0, 1);
constexpr auto NODES_AUTO_REFRESH_INTERVAL = 1s; // unit: seconds
constexpr auto DESIRED_FRAME_RATE = 30;
constexpr std::chrono::duration<float> DESIRED_FRAME_DURATION_MS = 1000ms / DESIRED_FRAME_RATE;

enum class StatusTextType { NONE, NO_NODES_AVAILABLE, PARAMETER_CHANGED, SERVICE_TIMEOUT };

static void glfw_error_callback(int error, const char *description) {
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

static std::set<ImGuiID> visualizeParameters(ServiceWrapper &serviceWrapper,
                                             const std::shared_ptr<ParameterGroup> &parameterNode,
                                             std::size_t maxParamLength, const std::string &filterString,
                                             bool expandAll = false);
static void highlightedText(const std::string &text, std::size_t start, std::size_t end,
                            const ImVec4 &highlightColor = FILTER_HIGHLIGHTING_COLOR);
static bool highlightedSelectableText(const std::string &text, std::size_t start, std::size_t end,
                                      const ImVec4 &highlightColor = FILTER_HIGHLIGHTING_COLOR);

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    bool manualFrameLimit = false;

    // really simple argument parsing, intended to start the manual vsync automatically (e.g. use an alias to add
    // the argument in the remote setup)
    if (argc == 2 && std::string(argv[1]) == "--manual_framerate_limit") {
        manualFrameLimit = true;
    }

    ServiceWrapper serviceWrapper;

    // Setup window
    glfwSetErrorCallback(glfw_error_callback);
    if (glfwInit() != GLFW_TRUE) {
        return 1;
    }

    // GL 3.0 + GLSL 130
    const char *glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    // Create window with graphics context
    GLFWwindow *window = glfwCreateWindow(600, 800, "Parameter modification editor", NULL, NULL);
    if (window == NULL)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImGuiWindowFlags window_flags =
            ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_MenuBar;

    int selectedIndex = -1;
    int nodeNameIndex = -1;
    std::vector<std::string> nodeNames;
    std::string curSelectedNode;
    std::string status;
    StatusTextType statusType = StatusTextType::NONE;
    ParameterTree parameterTree;         // tree with all parameters
    ParameterTree filteredParameterTree; // parameter tree after the application of the filter string
    bool reapplyFilter = true;
    std::string filter;              // current filter string of the text input field
    std::string currentFilterString; // currently active filter string
    bool autoRefreshNodes = true;
    auto lastNodeRefreshTime = std::chrono::system_clock::now();
    // unfortunately DearImGui doesn't provide any option to collapse tree nodes recursively, hence, we need to keep
    // track of all the ID of each open tree node
    std::set<ImGuiID> treeNodeIDs;

    // request available nodes on startup
    serviceWrapper.pushRequest(std::make_shared<Request>(Request::Type::QUERY_NODE_NAMES));

    bool shouldResetLayout = true;

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        // these variables are only relevant for a single iteration
        bool expandAllParameters = false;

        const auto frame_start = std::chrono::high_resolution_clock::now();

        // check the response queue
        auto response = serviceWrapper.tryPopResponse();
        serviceWrapper.checkForTimeouts();

        if (response != nullptr) {
            switch (response->type) {
                case Response::Type::NODE_NAMES:
                    nodeNames = std::dynamic_pointer_cast<NodeNameResponse>(response)->nodeNames;

                    if (nodeNames.empty()) {
                        status = "Seems like there are no nodes available!";
                        statusType = StatusTextType::NO_NODES_AVAILABLE;
                    }

                    break;
                case Response::Type::PARAMETERS: {
                    auto parameters = std::dynamic_pointer_cast<ParameterValueResponse>(response)->parameters;

                    // reorganize the parameters as a tree
                    parameterTree.clear();
                    for (const auto &param : parameters) {
                        parameterTree.add(param);
                    }
                    reapplyFilter = true;
                    break;
                }

                case Response::Type::MODIFICATION_RESULT: {
                    auto result = std::dynamic_pointer_cast<ParameterModificationResponse>(response);

                    if (result->success) {
                        status = "Parameter '" + result->parameterName + "' modified successfully!";
                    } else {
                        status = "Parameter '" + result->parameterName + "' couldn't be modified!";
                    }
                    statusType = StatusTextType::PARAMETER_CHANGED;

                    break;
                }

                case Response::Type::SERVICE_TIMEOUT: {
                    auto result = std::dynamic_pointer_cast<ServiceTimeout>(response);

                    status = "Node '" + result->nodeName + "' didn't respond to service call. Maybe the node has died?";
                    statusType = StatusTextType::SERVICE_TIMEOUT;

                    break;
                }
            }
        }

        // handle changes of the selected node + died nodes / newly added nodes during the refresh step
        const auto nodeNameIterator = std::find(nodeNames.begin(), nodeNames.end(), curSelectedNode);
        bool nodeStillAvailable = (nodeNameIterator != nodeNames.end());
        bool nameAtIndexChanged = (selectedIndex < nodeNames.size() && curSelectedNode != nodeNames.at(selectedIndex));

        if (nodeNameIndex == selectedIndex && nameAtIndexChanged && nodeStillAvailable) {
            // node list has changed, e.g. because new nodes have been started
            // -> selected node does still exist, hence, we simply need to update the selected index
            selectedIndex = std::distance(nodeNames.begin(), nodeNameIterator);
            nodeNameIndex = selectedIndex;
        } else if (nodeNameIndex != selectedIndex) {
            // selected node has changed
            selectedIndex = nodeNameIndex;

            treeNodeIDs.clear();

            auto selectedNodeName = nodeNames.at(selectedIndex);

            if (selectedNodeName != curSelectedNode) {
                curSelectedNode = selectedNodeName;

                // query parameters of the node
                serviceWrapper.setNodeOfInterest(curSelectedNode);
                serviceWrapper.pushRequest(std::make_shared<Request>(Request::Type::QUERY_NODE_PARAMETERS));
            }

            // clear warning about died node if one switches to another one
            if (statusType == StatusTextType::SERVICE_TIMEOUT) {
                status.clear();
                statusType = StatusTextType::NONE;
            }
        } else if (!curSelectedNode.empty()
                   && (nodeNames.empty() || nameAtIndexChanged || selectedIndex >= nodeNames.size())) {
            status = "Warning: Node '" + curSelectedNode + "' seems to have died!";
            statusType = StatusTextType::SERVICE_TIMEOUT;
        }

        if (reapplyFilter == true || currentFilterString != filter) {
            reapplyFilter = false;
            currentFilterString = filter;

            filteredParameterTree = parameterTree.filter(currentFilterString);
        }

        // auto refresh the node list periodically
        const auto currentTime = std::chrono::system_clock::now();
        if (currentTime - lastNodeRefreshTime > NODES_AUTO_REFRESH_INTERVAL && autoRefreshNodes) {
            lastNodeRefreshTime = currentTime;
            serviceWrapper.pushRequest(std::make_shared<Request>(Request::Type::QUERY_NODE_NAMES));
        }

        // Poll and handle events (inputs, window resize, etc.)
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiViewport *viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->Pos);
        ImGui::SetNextWindowSize(viewport->Size);
        ImGui::SetNextWindowViewport(viewport->ID);
        ImGui::SetNextWindowBgAlpha(0.0F);

        ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
        window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
                        ImGuiWindowFlags_NoMove;
        window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0F);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0F);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));

        ImGui::Begin("Root window", nullptr, window_flags);
        ImGui::PopStyleVar(3);

        ImGuiID dockspace_id = ImGui::GetID("Root window dockspace");
        ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_PassthruCentralNode;
        ImGui::DockSpace(dockspace_id, ImVec2(0.0F, 0.0F), dockspace_flags);

        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("View")) {
                ImGui::MenuItem("Reset layout", nullptr, &shouldResetLayout);
                ImGui::MenuItem("Manual vsync", nullptr, &manualFrameLimit);
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Nodes")) {
                ImGui::MenuItem("Refresh periodically", nullptr, &autoRefreshNodes);
                ImGui::EndMenu();
            }

            ImGui::EndMainMenuBar();
        }

        if (ImGui::DockBuilderGetNode(dockspace_id) == NULL || shouldResetLayout) {
            shouldResetLayout = false;
            ImGui::DockBuilderRemoveNode(dockspace_id);                            // Clear out existing layout
            ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace); // Add empty node
            ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->Size);

            ImGuiID top = 0;
            ImGuiID bottom = 0;
            ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Up, 0.9, &top, &bottom);

            ImGuiID left = 0;
            ImGuiID right = 0;
            ImGui::DockBuilderSplitNode(top, ImGuiDir_Left, 0.3, &left, &right);

            ImGui::DockBuilderDockWindow("Nodes", left);
            ImGui::DockBuilderDockWindow("Parameters", right);
            ImGui::DockBuilderDockWindow("Status", bottom);
            ImGui::DockBuilderFinish(dockspace_id);
        }

        ImGui::Begin("Nodes");

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

            if (statusType == StatusTextType::NO_NODES_AVAILABLE) {
                status.clear();
                statusType = StatusTextType::NONE;
            }
        }

        if (ImGui::Button("Refresh")) {
            serviceWrapper.pushRequest(std::make_shared<Request>(Request::Type::QUERY_NODE_NAMES));

            if (statusType == StatusTextType::SERVICE_TIMEOUT) {
                status.clear();
                statusType = StatusTextType::NONE;
            }
        }

        ImGui::End();

        ImGui::Begin("Parameters");

        if (!curSelectedNode.empty()) {
            ImGui::Text("Parameters of '%s'", curSelectedNode.c_str());
            ImGui::Dummy(ImVec2(0.0f, 5.0f));

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

            ImGui::Dummy(ImVec2(0.0f, 10.0f));

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

            ImGui::Dummy(ImVec2(0.0f, 10.0f));

            const auto ids = visualizeParameters(serviceWrapper, filteredParameterTree.getRoot(),
                                                 filteredParameterTree.getMaxParamNameLength(), currentFilterString,
                                                 expandAllParameters);
            treeNodeIDs.insert(ids.begin(), ids.end());

            if (statusType == StatusTextType::NO_NODES_AVAILABLE) {
                status.clear();
                statusType = StatusTextType::NONE;
            }
        } else {
            ImGui::Text("Please select a node first!");
        }

        ImGui::End();

        ImGui::Begin("Status");
        if (statusType == StatusTextType::SERVICE_TIMEOUT) {
            ImGui::TextColored(STATUS_WARNING_COLOR, "%s", status.c_str());
        } else {
            ImGui::Text("%s", status.c_str());
        }
        ImGui::End();

        ImGui::End();

        // Rendering
        {
            ImGui::Render();
            int display_h = 0;
            int display_w = 0;
            glfwGetFramebufferSize(window, &display_w, &display_h);
            glViewport(0, 0, display_w, display_h);
            ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
            glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w,
                         clear_color.w);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            glfwSwapBuffers(window);
        }

        // unfortunately vsync via glfw is broken for VNC sessions, hence, we need to emulate it manually
        if (manualFrameLimit) {
            const auto frame_end = std::chrono::high_resolution_clock::now();
            const auto duration = duration_cast<std::chrono::milliseconds>(frame_end - frame_start);
            const auto waitTime = DESIRED_FRAME_DURATION_MS - duration;

            if (waitTime.count() > 0) {
                std::this_thread::sleep_for(waitTime);
            }
        }
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    serviceWrapper.terminate();

    rclcpp::shutdown();
}

std::set<ImGuiID> visualizeParameters(ServiceWrapper &serviceWrapper,
                                         const std::shared_ptr<ParameterGroup> &parameterNode,
                                         std::size_t maxParamLength, const std::string &filterString,
                                         const bool expandAll) {
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

    for (auto &[name, fullPath, value, highlightingStart, highlightingEnd] : parameterNode->parameters) {
        std::string identifier = "##" + name;

        // simple 'space' padding to avoid the need for a more complex layout with columns (the latter is still desired
        // :D)
        std::string padding;
        if (name.length() < maxParamLength) {
            padding = std::string(maxParamLength - name.length(), ' ');
        }

        ImGui::AlignTextToFramePadding();

        if (highlightingStart.has_value() && highlightingEnd.has_value()) {
            highlightedText(name, highlightingStart.value(), highlightingEnd.value());
        } else {
            ImGui::Text("%s", name.c_str());
        }

        ImGui::SameLine(0, 0);
        ImGui::Text("%s", padding.c_str());

        ImGui::SameLine();
        ImGui::PushItemWidth(INPUT_TEXT_FIELD_WIDTH);

        if (std::holds_alternative<double>(value)) {
            if (ImGui::DragScalar(identifier.c_str(), ImGuiDataType_Double, &std::get<double>(value), 1.0F, nullptr,
                                  nullptr, "%.6g")) {
                serviceWrapper.pushRequest(
                        std::make_shared<ParameterModificationRequest>(ROSParameter(fullPath, value)));
            }
        } else if (std::holds_alternative<bool>(value)) {
            if (ImGui::Checkbox(identifier.c_str(), &std::get<bool>(value))) {
                serviceWrapper.pushRequest(
                        std::make_shared<ParameterModificationRequest>(ROSParameter(fullPath, value)));
            }
        } else if (std::holds_alternative<int>(value)) {
            if (ImGui::DragInt(identifier.c_str(), &std::get<int>(value))) {
                serviceWrapper.pushRequest(
                        std::make_shared<ParameterModificationRequest>(ROSParameter(fullPath, value)));
            }
        } else if (std::holds_alternative<std::string>(value)) {
            if (ImGui::InputText(identifier.c_str(), &std::get<std::string>(value))) {
                serviceWrapper.pushRequest(
                        std::make_shared<ParameterModificationRequest>(ROSParameter(fullPath, value)));
            }
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
                                                        subgroup->prefixSearchPatternEnd.value());
            } else {
                textClicked = ImGui::Selectable((subgroup->prefix).c_str());
            }

            if (textClicked) {
                ImGui::TreeNodeSetOpen(nodeID, !open);
            }

            if (open) {
                auto subIDs = visualizeParameters(serviceWrapper, subgroup, maxParamLength, filterString,
                                                  expandAll);
                nodeIDs.insert(subIDs.begin(), subIDs.end());
                ImGui::TreePop();
            }
        }
    }

    return nodeIDs;
}


void highlightedText(const std::string &text, std::size_t start, std::size_t end, const ImVec4 &highlightColor) {
    if (start == std::string::npos) {
        ImGui::Text("%s", text.c_str());
        return;
    }

    if (start > 0) {
        ImGui::Text("%s", text.substr(0, start).c_str());
        ImGui::SameLine(0, 0);
    }

    ImGui::PushStyleColor(ImGuiCol_Text, FILTER_HIGHLIGHTING_COLOR);
    ImGui::Text("%s", text.substr(start, end - start).c_str());
    ImGui::PopStyleColor();

    if (end < text.length() - 1) {
        ImGui::SameLine(0, 0);
        ImGui::Text("%s", text.substr(end).c_str());
    }
}

bool highlightedSelectableText(const std::string &text, std::size_t start, std::size_t end,
                               const ImVec4 &highlightColor) {
    bool selected = false;

    if (start == std::string::npos) {
        selected |= ImGui::Selectable(text.c_str());
        return selected;
    }

    if (start > 0) {
        selected |= ImGui::Selectable( text.substr(0, start).c_str());
        ImGui::SameLine(0, 0);
    }

    ImGui::PushStyleColor(ImGuiCol_Text, FILTER_HIGHLIGHTING_COLOR);
    selected |= ImGui::Selectable(text.substr(start, end - start).c_str());
    ImGui::PopStyleColor();

    if (end < text.length() - 1) {
        ImGui::SameLine(0, 0);
        selected |= ImGui::Selectable(text.substr(end).c_str());
    }

    return selected;
}
