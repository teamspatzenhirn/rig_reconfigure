/**
 * @file   utils.hpp
 * @author Dominik Authaler
 * @date   12.01.2024
 *
 * Collection of utility functions.
 */

#ifndef RIG_RECONFIGURE_UTILS_HPP
#define RIG_RECONFIGURE_UTILS_HPP

#include <string>
#include <imgui.h>
#include <filesystem>

#include <GLFW/glfw3.h> // will drag system OpenGL headers

struct Status {
    enum class Type { NONE, NO_NODES_AVAILABLE, PARAMETER_CHANGED, SERVICE_TIMEOUT };

    Type type = Type::NONE;
    std::string text;
};

/**
 * Utility imgui function for partly highlighted text.
 * @param text           String which should be displayed.
 * @param start          Starting index of the highlighted part.
 * @param end            End index of the highlighted part.
 * @param highlightColor Color used for the highlighted part, the remaining text is displayed using the default text
 *                       color.
 */
void highlightedText(const std::string &text, std::size_t start, std::size_t end,
                     const ImVec4 &highlightColor);

/**
 * Utility imgui function for partly highlighted text which can be selected.
 * @param text           String which should be displayed.
 * @param start          Starting index of the highlighted part.
 * @param end            End index of the highlighted part.
 * @param highlightColor Color used for the highlighted part, the remaining text is displayed using the default text
 *                       color.
 */
bool highlightedSelectableText(const std::string &text, std::size_t start, std::size_t end,
                               const ImVec4 &highlightColor);

/**
 * Searches for the resource directory.
 * @param execPath Executable path.
 * @return Path to the resource directory.
 */
std::filesystem::path findResourcePath(const std::string &execPath);

/**
 * Loads an icon for the provided window.
 * @param windowPtr    Window for which the icon should be loaded.
 * @param resourcePath Path to the icon data.
 */
void loadWindowIcon(GLFWwindow *windowPtr, const std::filesystem::path &resourcePath);

/**
 * Prints the corresponding error.
 * @param error       Error code.
 * @param description Detailed error description.
 */
void glfw_error_callback(int error, const char *description);

#endif //RIG_RECONFIGURE_UTILS_HPP
