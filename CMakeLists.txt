cmake_minimum_required(VERSION 3.10)
SET(PROJECT_NAME "RoboSim")

project(${PROJECT_NAME})
# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)

# Add your source files
set(SOURCES 
    "src/main.cpp")

set(COLLISION_SOURCES
    "src/physics/collisions/broad_phase.cpp"
    "src/physics/collisions/aabb_tree.cpp"
    "src/physics/collisions/sphere_v_sphere.cpp"
    "src/physics/collisions/sphere_v_plane.cpp"
    "src/physics/collisions/sphere_v_box.cpp"
    "src/physics/collisions/box_v_box.cpp"
    "src/physics/collisions/plane_v_box.cpp"
    "src/physics/collisions/collision_utils.cpp"
    )

set(PHYSICS_SOURCES
    # Bodies
    "src/physics/bodies/Body.cpp"
    # Constraints
    "src/physics/constraints/PositionalConstraint.cpp"
    "src/physics/constraints/RotationalConstraint.cpp"
    "src/physics/constraints/RevoluteJointConstraint.cpp"
    "src/physics/constraints/ContactConstraint.cpp"
    "src/physics/constraints/Utils.cpp"
    # Collisions
    ${COLLISION_SOURCES}
    # Math
    "src/physics/math/math.cpp"    
    # World
    "src/physics/World.cpp"
    "src/physics/WorldColliders.cpp"
    "src/physics/WorldCollisions.cpp"
    "src/physics/WorldArticulatedSystem.cpp"
    "src/physics/WorldURDFLoader.cpp"
    #Interface
    "src/visInterface/Interface.cpp"
    )


set(
    PHYSICS_INCLUDE_DIRS
    "src"
    "src/physics/collisions"
    "src/physics/shapes"
    "src/physics/constraints"
)

set(
    INTERFACE_INCLUDE_DIRS
    "src/visInterface"
)


set(
    DEMOS_INCLUDE_DIRS
    "scenarios"
)

# GLEW
find_package(GLEW REQUIRED) # Im using GLEW 2.2.0
find_package(raylib REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(hpp-fcl REQUIRED)
include_directories(${hpp-fcl_INCLUDE_DIRS})

# include_directories("thirdParty/assimp/include/")
# add_subdirectory("thirdParty/assimp")


include_directories("src/RoboVis/src" "src/RoboVis/thirdParty/rlImGui" "src/RoboVis/thirdParty/imgui" "src/RoboVis/thirdParty/rlights")
add_subdirectory("src/RoboVis")
add_subdirectory("thirdParty/URDF_Parser")

add_executable(${PROJECT_NAME} ${SOURCES} ${IMGUI_SOURCES} ${IMPLOT_SOURCES} ${PHYSICS_SOURCES})



target_include_directories(${PROJECT_NAME} PRIVATE  ${PHYSICS_INCLUDE_DIRS} "src/RoboVis/src" ${INTERFACE_INCLUDE_DIRS} ${DEMOS_INCLUDE_DIRS} "thirdParty/URDF_Parser/include")
target_link_libraries(${PROJECT_NAME} PRIVATE RoboVis raylib Eigen3::Eigen ${hpp-fcl_LIBRARIES} urdfparser)
