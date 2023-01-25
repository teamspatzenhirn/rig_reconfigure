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

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_internal.h"
#include "service_wrapper.hpp"
#include "parameter_tree.hpp"

static void glfw_error_callback(int error, const char *description) {
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void print_error_and_fail(const std::string &error) {
    std::cerr << error << std::endl;
    std::exit(1);
}

void visualizeParameters(ServiceWrapper &serviceWrapper, const std::shared_ptr<ParameterGroup> &parameterNode, const std::string &prefix = "");

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

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
    glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);

    // Create window with graphics context
    GLFWwindow *window = glfwCreateWindow(1280, 720, "Parameter modification editor", NULL, NULL);
    if (window == NULL)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    int selectedIndex = 0;
    std::vector<std::string> nodeNames;
    std::string curSelectedNode;
    ParameterTree parameterTree;

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        // check the response queue
        auto response = serviceWrapper.tryPopResponse();

        if (response != nullptr) {
            switch (response->type) {
                case Response::Type::NODE_NAMES:
                    nodeNames = std::dynamic_pointer_cast<NodeNameResponse>(response)->nodeNames;
                    break;
                case Response::Type::PARAMETERS: {
                    auto parameters = std::dynamic_pointer_cast<ParameterValueResponse>(response)->parameters;

                    // reorganize the parameters as a tree
                    parameterTree.clear();
                    for (const auto param : parameters) {
                        parameterTree.add(param);
                    }
                    break;
                }

                case Response::Type::MODIFICATION_RESULT:
                    auto result = std::dynamic_pointer_cast<ParameterModificationResponse>(response);

                    std::cout << "Modification successful: " << result->success << std::endl;
                    std::cout << "Reason: " << result->reason << std::endl;
            }
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

        ImGui::Begin("Dynamic reconfigure");

        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Node: ");
        ImGui::SameLine();

        if (!nodeNames.empty()) {
            auto selectedNodeName = nodeNames.at(selectedIndex);

            if (selectedNodeName != curSelectedNode) {
                curSelectedNode = selectedNodeName;

                // query parameters of the node
                serviceWrapper.setNodeOfInterest(curSelectedNode);
                serviceWrapper.pushRequest(std::make_shared<Request>(Request::Type::QUERY_NODE_PARAMETERS));
            }
        }

        if (ImGui::BeginCombo("##", curSelectedNode.c_str())) {
            for (auto i = 0U; i < nodeNames.size(); ++i) {
                const bool isSelected = (selectedIndex == i);
                if (ImGui::Selectable(nodeNames[i].c_str(), isSelected)) {
                    selectedIndex = i;
                }
            }

            ImGui::EndCombo();
        }

        ImGui::SameLine();

        if (ImGui::Button("Refresh")) {
            serviceWrapper.pushRequest(std::make_shared<Request>(Request::Type::QUERY_NODE_NAMES));
        }

        ImGui::Separator();

        if (!curSelectedNode.empty()) {
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted("Parameters");
            ImGui::SameLine();

            if (ImGui::Button("Reload parameters")) {
                serviceWrapper.pushRequest(std::make_shared<Request>(Request::Type::QUERY_NODE_PARAMETERS));
            }

            ImGui::Dummy(ImVec2(0.0f, 10.0f));

            visualizeParameters(serviceWrapper, parameterTree.getRoot());
        }

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

void visualizeParameters(ServiceWrapper &serviceWrapper, const std::shared_ptr<ParameterGroup> &parameterNode, const std::string &prefix) {
    if (parameterNode == nullptr || (parameterNode->parameters.empty() && parameterNode->subgroups.empty())) {
        ImGui::Text("This node doesn't seem to have any parameters!");
        return;
    }

    for (auto &[name, value] : parameterNode->parameters) {
//        std::string identifier = "##" + name;
        std::string identifier = name;

//        ImGui::AlignTextToFramePadding();
//        ImGui::Text("%s", name.c_str());
//        ImGui::SameLine();
        if (std::holds_alternative<double>(value)) {
            if (ImGui::InputDouble(identifier.c_str(), &std::get<double>(value), 0.01, 0.5, "%.2f")) {
                serviceWrapper.pushRequest(std::make_shared<ParameterModificationRequest>(ROSParameter(prefix + '/' + name, value)));
            }
        } else if (std::holds_alternative<bool>(value)) {
            if (ImGui::Checkbox(identifier.c_str(), &std::get<bool>(value))) {
                serviceWrapper.pushRequest(std::make_shared<ParameterModificationRequest>(ROSParameter(prefix + '/' + name, value)));
            }
        } else if (std::holds_alternative<int>(value)) {
            if (ImGui::InputInt(identifier.c_str(), &std::get<int>(value), 1, 10)) {
                serviceWrapper.pushRequest(std::make_shared<ParameterModificationRequest>(ROSParameter(prefix + '/' + name, value)));
            }
        }
    }

    if (!parameterNode->subgroups.empty()) {
        for (const auto &subgroup : parameterNode->subgroups) {
            if (ImGui::TreeNode(subgroup->prefix.c_str())) {
                visualizeParameters(serviceWrapper, subgroup, prefix + '/' +  subgroup->prefix.c_str());
                ImGui::TreePop();
            }
        }
    }
}