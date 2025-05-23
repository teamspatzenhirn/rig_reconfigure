cmake_minimum_required(VERSION 3.8)
project(rig_reconfigure)

set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(backward_ros QUIET)

# Download imgui
include(FetchContent)
FetchContent_Declare(imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.89.8-docking
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        )
FetchContent_MakeAvailable(imgui)

add_library(imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
        ${imgui_SOURCE_DIR}/misc/cpp/imgui_stdlib.cpp
        )
target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
find_package(glfw3 3.3 REQUIRED)
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)

add_executable(${PROJECT_NAME} src/rig_reconfigure.cpp src/service_wrapper.cpp src/parameter_tree.cpp
                               src/utils.cpp src/node_window.cpp src/parameter_window.cpp external/lodepng/lodepng.cpp)

# uncomment for checking the executable with tsan
#target_compile_options(${PROJECT_NAME} PRIVATE -g -fsanitize=thread)
#target_link_options(${PROJECT_NAME} PRIVATE -fsanitize=thread)

target_include_directories(${PROJECT_NAME} PRIVATE include external/lodepng)
target_link_libraries(${PROJECT_NAME} imgui rclcpp::rclcpp ament_index_cpp::ament_index_cpp)

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
)

install(FILES resource/rig_reconfigure.png resource/rig_reconfigure_text.png
        DESTINATION share/${PROJECT_NAME}/resource
)

install(FILES resource/rig_reconfigure.svg
        DESTINATION share/icons/hicolor/scalable/apps
)

install(FILES resource/rig_reconfigure.desktop
        DESTINATION share/applications
)

ament_package()
