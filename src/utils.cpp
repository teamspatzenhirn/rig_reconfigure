/**
 * @file   utils.cpp
 * @author Dominik Authaler
 * @date   12.01.2024
 *
 * Collection of utility functions.
 */

#include "utils.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <vector>

#include "lodepng.h"

void highlightedText(const std::string &text, std::size_t start, std::size_t end, const ImVec4 &highlightColor) {
    if (start == std::string::npos) {
        ImGui::Text("%s", text.c_str());
        return;
    }

    if (start > 0) {
        ImGui::Text("%s", text.substr(0, start).c_str());
        ImGui::SameLine(0, 0);
    }

    ImGui::PushStyleColor(ImGuiCol_Text, highlightColor);
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
        selected |= ImGui::Selectable(text.substr(0, start).c_str());
        ImGui::SameLine(0, 0);
    }

    ImGui::PushStyleColor(ImGuiCol_Text, highlightColor);
    selected |= ImGui::Selectable(text.substr(start, end - start).c_str());
    ImGui::PopStyleColor();

    if (end < text.length() - 1) {
        ImGui::SameLine(0, 0);
        selected |= ImGui::Selectable(text.substr(end).c_str());
    }

    return selected;
}

std::filesystem::path findResourcePath(const std::string &execPath) {
    auto resourcePath = std::filesystem::path(execPath).parent_path().append("resource");

    try {
        // Try getting package share dir via ament, and use that if it succeeds.
        resourcePath = ament_index_cpp::get_package_share_directory("rig_reconfigure");
        resourcePath.append("resource");
    } catch (ament_index_cpp::PackageNotFoundError &e) {
        std::cerr << "Warning: Error while looking for package share directory: " << e.what()
                  << "\n";
    }

    return resourcePath;
}

void loadWindowIcon(GLFWwindow *windowPtr, const std::filesystem::path &resourcePath) {
    const auto logoPath = resourcePath / "rig_reconfigure.png";

    std::vector<unsigned char> iconData;
    unsigned int width;
    unsigned int height;

    unsigned int error = lodepng::decode(iconData, width, height, logoPath.string());

    if (error == 0) {
        GLFWimage icon;

        icon.width = static_cast<int>(width);
        icon.height = static_cast<int>(height);
        icon.pixels = iconData.data();

        glfwSetWindowIcon(windowPtr, 1, &icon);
    } else {
        std::cerr << "Unable to load window icon (decoder error " << error << " - " << lodepng_error_text(error) << ")"
                  << std::endl;
    }
}

void glfw_error_callback(const int error, const char *description) {
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}
