cmake_minimum_required(VERSION 3.10)
SET(PROJECT_NAME "RoboSim")

project(${PROJECT_NAME})
# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)

# Add your source files
set(SOURCES 
    "src/main.cpp")

set(PHYSICS_SOURCES
    # Bodies
    "src/physics/bodies/DynamicBody.cpp"
    "src/physics/bodies/Body.cpp"
    # Constraints
    "src/physics/constraints/PositionalConstraint.cpp"
    )

set(
    VIEWER_SOURCES
    "src/viewer/Camera.cpp"
    "src/viewer/Shader.cpp"
)

set(
    PHYSICS_INCLUDE_DIRS
    "src"
)

# Add ImGui source files
set(IMGUI_SOURCES 
    "thirdParty/imgui/imgui.cpp" 
    "thirdParty/imgui/imgui_draw.cpp" 
    "thirdParty/imgui/imgui_widgets.cpp" 
    "thirdParty/imgui/imgui_tables.cpp"
    "thirdParty/imgui/backends/imgui_impl_opengl3.cpp" 
    "thirdParty/imgui/backends/imgui_impl_glfw.cpp")

# GLEW
find_package(GLEW REQUIRED) # Im using GLEW 2.2.0

# Add ImPlot source files
set(IMPLOT_SOURCES 
    "thirdParty/implot/implot.cpp" 
    "thirdParty/implot/implot_items.cpp")

# If you're on Linux, use the OpenGL3 and GLFW backends
if(UNIX AND NOT APPLE)
    add_subdirectory("thirdParty/glfw")
    include_directories("thirdParty/glfw/include")
    set(OpenGL_GL_PREFERENCE GLVND)
    find_package(OpenGL REQUIRED)
    add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GLAD)
    list(APPEND IMGUI_SOURCES "thirdParty/imgui/backends/imgui_impl_opengl3.cpp" "thirdParty/imgui/backends/imgui_impl_glfw.cpp")

    set(OPENGL_opengl_LIBRARY "/usr/lib/x86_64-linux-gnu/libGL.so")
    set(OPENGL_glx_LIBRARY "/usr/lib/x86_64-linux-gnu/libGLX.so")
    set(OPENGL_INCLUDE_DIR "/usr/include/GL")
endif()

# If you're on Mac, use the OpenGL2 (legacy) and GLFW backends
if(APPLE)
    find_package(OpenGL REQUIRED)
    find_package(glfw3 REQUIRED)
    list(APPEND IMGUI_SOURCES "thirdParty/imgui/backends/imgui_impl_opengl2.cpp" "thirdParty/imgui/backends/imgui_impl_glfw.cpp")
endif()

# If you're on Windows, use the DirectX11 and Win32 backends
if(WIN32)
    list(APPEND IMGUI_SOURCES "thirdParty/imgui/backends/imgui_impl_dx11.cpp" "thirdParty/imgui/backends/imgui_impl_win32.cpp")
endif()

add_executable(${PROJECT_NAME} ${SOURCES} ${IMGUI_SOURCES} ${IMPLOT_SOURCES} ${PHYSICS_SOURCES} ${VIEWER_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE "thirdParty/imgui" "thirdParty/implot" "thirdParty/qvm"  ${PHYSICS_INCLUDE_DIRS})

if(UNIX AND NOT APPLE)
    target_link_libraries(${PROJECT_NAME} glfw ${OPENGL_LIBRARIES} GLEW::GLEW)
endif()

if(APPLE)
    target_link_libraries(${PROJECT_NAME} glfw ${OPENGL_LIBRARIES})
endif()

if(WIN32)
    target_link_libraries(${PROJECT_NAME} d3d11.lib)
endif()
