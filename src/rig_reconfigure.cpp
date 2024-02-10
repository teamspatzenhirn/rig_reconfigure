/**
 * @file   rig_reconfigure.cpp
 * @author Dominik Authaler
 * @date   11.12.2022
 *
 * Dynamic reconfigure GUI via DearImgui.
 */


#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include <cstdlib>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_internal.h>
#include <vector>

#include <lodepng.h>

#include "service_wrapper.hpp"
#include "utils.hpp"
#include "node_window.hpp"
#include "parameter_window.hpp"

using namespace std::chrono_literals;

constexpr auto STATUS_WARNING_COLOR = ImVec4(1, 0, 0, 1);
constexpr auto NODES_AUTO_REFRESH_INTERVAL = 1s; // unit: seconds
constexpr auto DESIRED_FRAME_RATE = 30;
constexpr std::chrono::duration<float> DESIRED_FRAME_DURATION_MS = 1000ms / DESIRED_FRAME_RATE;
constexpr auto DEFAULT_WINDOW_WIDTH = 600;
constexpr auto DEFAULT_WINDOW_HEIGHT = 800;

// window names
constexpr auto NODE_WINDOW_NAME = "Nodes";
constexpr auto STATUS_WINDOW_NAME = "Status";
constexpr auto PARAMETER_WINDOW_NAME = "Parameters";

static void renderInfoWindow(bool *showInfoWindow, const std::filesystem::path &resourcePath);

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

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    // place the imgui.ini config file within the users home directory (instead of current working directory)
    const std::filesystem::path config_file_dir(std::string(std::getenv("HOME")) + "/.config/rig_reconfigure");
    const std::string config_file_path = config_file_dir.string() + "/imgui.ini";

    if (!std::filesystem::exists(config_file_dir)) {
        std::filesystem::create_directory(config_file_dir);
    }

    ImGui::GetIO().IniFilename = config_file_path.c_str();
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    // load window size if already stored from last iteration
    bool configFileExisting = std::filesystem::exists(config_file_path);
    auto window_width = DEFAULT_WINDOW_WIDTH;
    auto window_height = DEFAULT_WINDOW_HEIGHT;

    if (configFileExisting) {
        ImGui::LoadIniSettingsFromDisk(config_file_path.c_str());
        ImGuiID id = ImHashStr("Root window");
        ImGuiWindowSettings* root_window_settings = ImGui::FindWindowSettingsByID(id);

        window_width = root_window_settings->Size.x;
        window_height = root_window_settings->Size.y;
    }

    // Create window with graphics context
    GLFWwindow *window = glfwCreateWindow(window_width, window_height, "Parameter modification editor",
                                          nullptr, nullptr);
    if (window == nullptr) {
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    const auto resourcePath = findResourcePath(argv[0]);

    loadWindowIcon(window, resourcePath);

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    std::vector<std::string> nodeNames;
    std::string selectedNode;      // name of the node which has been selected in the current iteration
    std::string curDisplayedNode;  // name of the node for which parameters are currently displayed
    Status status;
    ParameterTree parameterTree;         // tree with all parameters
    ParameterTree filteredParameterTree; // parameter tree after the application of the filter string
    bool reapplyFilter = true;
    std::string filter;                  // current filter string of the text input field
    bool autoRefreshNodes = true;
    auto lastNodeRefreshTime = std::chrono::system_clock::now();

    // request available nodes on startup
    serviceWrapper.pushRequest(std::make_shared<Request>(Request::Type::QUERY_NODE_NAMES));

    bool shouldResetLayout = false;
    bool showInfo = false;
    bool ignoreDefaultParameters = true;
    serviceWrapper.setIgnoreDefaultParameters(ignoreDefaultParameters);

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        // these variables are only relevant for a single iteration
        const auto frame_start = std::chrono::high_resolution_clock::now();

        // check the response queue
        auto response = serviceWrapper.tryPopResponse();
        serviceWrapper.checkForTimeouts();

        if (response != nullptr) {
            switch (response->type) {
                case Response::Type::NODE_NAMES:
                    nodeNames = std::dynamic_pointer_cast<NodeNameResponse>(response)->nodeNames;

                    if (nodeNames.empty()) {
                        status.text = "Seems like there are no nodes available!";
                        status.type = Status::Type::NO_NODES_AVAILABLE;
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
                        status.text = "Parameter '" + result->parameterName + "' modified successfully!";
                    } else {
                        status.text = "Parameter '" + result->parameterName + "' couldn't be modified!";
                    }
                    status.type = Status::Type::PARAMETER_CHANGED;

                    break;
                }

                case Response::Type::SERVICE_TIMEOUT: {
                    auto result = std::dynamic_pointer_cast<ServiceTimeout>(response);

                    status.text = "Node '" + result->nodeName + "' didn't respond to service call. Maybe the node has died?";
                    status.type = Status::Type::SERVICE_TIMEOUT;

                    break;
                }
            }
        }

        // handle changes of the selected node + died nodes / newly added nodes during the refresh step
        const auto nodeNameIterator = std::find(nodeNames.begin(), nodeNames.end(), curDisplayedNode);
        bool nodeStillAvailable = (nodeNameIterator != nodeNames.end());

        if (!curDisplayedNode.empty() && !nodeStillAvailable) {
            status.text = "Warning: Node '" + curDisplayedNode + "' seems to have died!";
            status.type = Status::Type::SERVICE_TIMEOUT;
        } else if (!selectedNode.empty() && selectedNode != curDisplayedNode) {
            // selected node has changed
            curDisplayedNode = selectedNode;

            // query parameters of the node
            serviceWrapper.setNodeOfInterest(curDisplayedNode);
            serviceWrapper.pushRequest(std::make_shared<Request>(Request::Type::QUERY_NODE_PARAMETERS));

            // clear warning about died node if one switches to another one
            if (status.type == Status::Type::SERVICE_TIMEOUT) {
                status.text.clear();
                status.type = Status::Type::NONE;
            }
        }

        if (reapplyFilter == true || filteredParameterTree.getAppliedFilter() != filter) {
            reapplyFilter = false;

            filteredParameterTree = parameterTree.filter(filter);
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

            if (ImGui::BeginMenu("Parameters")) {
                bool propertyChanged = ImGui::MenuItem("Hide default parameters", nullptr, &ignoreDefaultParameters);
                serviceWrapper.setIgnoreDefaultParameters(ignoreDefaultParameters);
                if (propertyChanged && !curDisplayedNode.empty()) {
                    // Reload parameters if menu item was toggled
                    serviceWrapper.pushRequest(std::make_shared<Request>(Request::Type::QUERY_NODE_PARAMETERS));
                }
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Info")) {
                ImGui::MenuItem("Show info", nullptr, &showInfo);
                ImGui::EndMenu();
            }

            ImGui::EndMainMenuBar();
        }

        renderInfoWindow(&showInfo, resourcePath);

        if (ImGui::DockBuilderGetNode(dockspace_id) == nullptr || shouldResetLayout || !configFileExisting) {
            shouldResetLayout = false;
            configFileExisting = true;
            ImGui::DockBuilderRemoveNode(dockspace_id);                            // Clear out existing layout
            ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace); // Add empty node
            ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->Size);

            ImGuiID top = 0;
            ImGuiID bottom = 0;
            ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Up, 0.9, &top, &bottom);

            ImGuiID left = 0;
            ImGuiID right = 0;
            ImGui::DockBuilderSplitNode(top, ImGuiDir_Left, 0.3, &left, &right);

            ImGui::DockBuilderDockWindow(NODE_WINDOW_NAME, left);
            ImGui::DockBuilderDockWindow(PARAMETER_WINDOW_NAME, right);
            ImGui::DockBuilderDockWindow(STATUS_WINDOW_NAME, bottom);
            ImGui::DockBuilderFinish(dockspace_id);
        }

        renderNodeWindow(NODE_WINDOW_NAME, nodeNames, serviceWrapper, selectedNode, status);

        // Note: updating the parameter window at least one iteration late is no problem since the parameters
        //       have to be queried anyway before being able to visualize anything meaningful
        renderParameterWindow(PARAMETER_WINDOW_NAME, curDisplayedNode, serviceWrapper, filteredParameterTree,
                              filter, status);

        ImGui::Begin(STATUS_WINDOW_NAME);
        if (status.type == Status::Type::SERVICE_TIMEOUT) {
            ImGui::TextColored(STATUS_WARNING_COLOR, "%s", status.text.c_str());
        } else {
            ImGui::Text("%s", status.text.c_str());
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
            ImVec4 clear_color = ImVec4(0.45F, 0.55F, 0.60F, 1.00F);
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

void renderInfoWindow(bool *showInfoWindow, const std::filesystem::path &resourcePath) {
    // load the logo with text only once
    static GLuint imageTexture;
    static unsigned int width = 0;
    static unsigned int height = 0;
    static bool imageLoaded = false;

    if (!imageLoaded) {
        auto logoPath = resourcePath / "rig_reconfigure_text.png";

        std::vector<unsigned char> imageData;

        unsigned int error = lodepng::decode(imageData, width, height, logoPath.string());

        if (error == 0) {
            glGenTextures(1, &imageTexture);
            glBindTexture(GL_TEXTURE_2D, imageTexture);

            // Setup filtering parameters for display
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

#if defined(GL_UNPACK_ROW_LENGTH) && !defined(__EMSCRIPTEN__)
            glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
#endif
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,static_cast<GLsizei>(width),
                         static_cast<GLsizei>(height), 0, GL_RGBA, GL_UNSIGNED_BYTE, imageData.data());
        }

        imageLoaded = true;
    }

    if (*showInfoWindow) {
        ImGui::Begin("Info", showInfoWindow, ImGuiWindowFlags_AlwaysAutoResize);

        if (width != 0 && height != 0) {
            ImGui::Image((void*)(intptr_t)imageTexture,
                         ImVec2(static_cast<float>(width), static_cast<float>(height)));
        } else {
            ImGui::Text("rig-reconfigure");
        }

        ImGui::Text(" ");
        ImGui::Text("Created in winter 2022 by");
        ImGui::Text(" ");
        ImGui::Text("Dominik Authaler");
        ImGui::Text("Jonas Otto");
        ImGui::Text(" ");
        ImGui::Text("Sources available at https://github.com/teamspatzenhirn/rig_reconfigure");
        ImGui::Text("under MIT license. Pull requests extending the functionality / fixing");
        ImGui::Text("bugs are always welcome!");
        ImGui::Text(" ");
        ImGui::Text("This tool has been created while the authors have been part of Team Spatzenhirn,");
        ImGui::Text("a student team at Ulm University. If you like rig-reconfigure, please consider");
        ImGui::Text("supporting the team (e.g. by joining if you're studying at Ulm University).");

        ImGui::End();
    }
}
