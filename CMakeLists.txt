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
    "src/physics/constraints/RotationalConstraint.cpp"
    # Math
    "src/physics/math/math.cpp"    
    )


set(
    PHYSICS_INCLUDE_DIRS
    "src"
    "src/physics/constraints"
)


# GLEW
find_package(GLEW REQUIRED) # Im using GLEW 2.2.0

include_directories("thirdParty/assimp/include/")
add_subdirectory("thirdParty/assimp")

add_executable(${PROJECT_NAME} ${SOURCES} ${IMGUI_SOURCES} ${IMPLOT_SOURCES} ${PHYSICS_SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE  ${PHYSICS_INCLUDE_DIRS})
