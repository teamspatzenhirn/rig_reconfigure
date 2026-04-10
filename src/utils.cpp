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
#include <cstdint>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <cmath>
#include <iostream>
#include <limits>
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

std::string getFormatStringFromStep(double step, int max_digits) {
    if (step == 0) {
        return std::string("%.6g");
    }
    const double epsilon = 1e-12;  // tolerance for rounding error
    int digits = 0;
    double scaled = step;
    while (digits < max_digits && std::abs(scaled - std::round(scaled)) > epsilon) {
        scaled *= 10.0;
        ++digits;
    }
    return "%." + std::to_string(digits) + "f";
}

bool hasBoundedRange(const rcl_interfaces::msg::ParameterDescriptor& param) {
    if (param.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
        if (param.floating_point_range.empty()) {
            return false;
        }

        if (!std::isfinite(param.floating_point_range[0].from_value) ||
            !std::isfinite(param.floating_point_range[0].to_value)) {
            return false;
        }

        if (param.floating_point_range[0].from_value <= -std::numeric_limits<float>::max() ||
            param.floating_point_range[0].from_value >=  std::numeric_limits<float>::max()) {
            return false;
        }

        if (param.floating_point_range[0].to_value <= -std::numeric_limits<float>::max() ||
            param.floating_point_range[0].to_value >=  std::numeric_limits<float>::max()) {
            return false;
        }

        return true;
    }
    else if (param.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
        if (param.integer_range.empty()) {
            return false;
        }

        // This is how generate_parameter_library encodes open integer ranges
        if (param.integer_range.at(0).to_value == std::numeric_limits<int64_t>::max() ||
            param.integer_range.at(0).from_value == std::numeric_limits<int64_t>::lowest()) {
            return false;
        }

        return true;
    }
    return false;
}

bool areDoublesEqual(double x, double y, double ulp) {
  return std::abs(x - y) <= std::numeric_limits<double>::epsilon() * std::abs(x + y) * ulp;
}

double snapToDoubleRange(double value, double from_value, double to_value, double step) {
    // this mostly matches the logic for checking if a double parameter value is
    // valid for a defined floating point range: rclcpp/node_interfaces/node_parameters.cpp

    // clamp value within bounds
    value = std::clamp(value, from_value, to_value);

    // handle cases where the step is not defined or would be invalid
    if (step == 0.0 || std::isnan(step) || from_value == std::numeric_limits<double>::lowest()) {
        // continuous range
        return value;
    }

    // return from_value if the value is within 100 ULPs
    if (areDoublesEqual(value, from_value)) {
        return from_value;
    }

    // return to_value if the value is within 100 ULPs
    if (areDoublesEqual(value, to_value)) {
        return to_value;
    }

    // snap to closest step
    step = std::abs(step);
    return from_value + std::round((value - from_value) / step) * step;
}

int64_t snapToIntegerRange(int64_t value, int64_t from_value, int64_t to_value, uint64_t step) {
    // this mostly matches the logic for checking if a double parameter value is
    // valid for a defined integer range: rclcpp/node_interfaces/node_parameters.cpp

    value = std::clamp(value, from_value, to_value);

    if (step == 0 || step == 1) {
        // all integers within range
        return value;
    }

    // the start of the range is always valid
    if (value == from_value) {
        return value;
    }

    // the end of the range is always valid
    if (value == to_value) {
        return value;
    }

    // snap to closest step
    return from_value + static_cast<int64_t>(std::round((value - from_value) / static_cast<double>(step))) * step;
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
