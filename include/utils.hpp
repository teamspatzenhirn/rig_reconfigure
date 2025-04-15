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
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

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

std::string getFormatStringFromStep(double step, int max_digits = 10);

/**
 * Checks if a given parameter has a range that is effectively bounded on both sides.
 *
 * @param param Parameter description.
 */
bool hasBoundedRange(const rcl_interfaces::msg::ParameterDescriptor& param);

/**
 * Checks if two double values are equal to within a specified tolerance.
 *
 * @param x   First value.
 * @param y   Second value.
 * @param ulp Maximum number of representable numbers between x and y.
 */
bool areDoublesEqual(double x, double y, double ulp = 100.0);

/**
 * Snaps and clamps a double value to a given range and step.
 *
 * Note: The value is snapped to the nearest integer number of steps from the 'from_value'
 *       or clamped to the 'to_value'.
 *
 *       The magnitude of the 'step' is used and the sign is ignored.
 *
 *       If 'step' == 0 or if 'from_value' is == double::lowest(), the range is considered
 *       continuous.
 *
 * @param value      Input value.
 * @param from_value Range minimum.
 * @param to_value   Range maximum.
 * @param step       Step size.
 */
double snapToDoubleRange(double value, double from_value, double to_value, double step);

/**
 * Snaps and clamps an integer value to a given range and step.
 *
 * Note: The value is snapped to the nearest integer number of steps from the 'from_value'
 *       or clamped to the 'to_value'.
 *
 *       The magnitude of 'step' is used and the sign is ignored.
 *
 *       If 'step' == 0, the step size is considered 1.
 *
 * @param value      Input value.
 * @param from_value Range minimum.
 * @param to_value   Range maximum.
 * @param step       Step size.
 */
int64_t snapToIntegerRange(int64_t value, int64_t from_value, int64_t to_value, uint64_t step);

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
