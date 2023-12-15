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
    "src/physics/bodies/Body.cpp"
    # Constraints
    "src/physics/constraints/PositionalConstraint.cpp"
    "src/physics/constraints/RotationalConstraint.cpp"
    # Math
    "src/physics/math/math.cpp"    
    # World
    "src/physics/World.cpp"
    )


set(
    PHYSICS_INCLUDE_DIRS
    "src"
    "src/physics/constraints"
)


# GLEW
find_package(GLEW REQUIRED) # Im using GLEW 2.2.0
find_package(raylib REQUIRED)

include_directories("thirdParty/assimp/include/")
add_subdirectory("thirdParty/assimp")


include_directories("src/RoboVis/src" "src/RoboVis/thirdParty/rlImGui" "src/RoboVis/thirdParty/imgui" "src/RoboVis/thirdParty/rlights")
add_subdirectory("src/RoboVis")

add_executable(${PROJECT_NAME} ${SOURCES} ${IMGUI_SOURCES} ${IMPLOT_SOURCES} ${PHYSICS_SOURCES})



target_include_directories(${PROJECT_NAME} PRIVATE  ${PHYSICS_INCLUDE_DIRS} "src/RoboVis/src")
target_link_libraries(${PROJECT_NAME} PRIVATE RoboVis raylib )
