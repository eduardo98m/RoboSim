#!/bin/bash

# Create the build directory if it doesn't exist
mkdir -p thirdParty

cd thirdParty
git clone https://github.com/ocornut/imgui.git
git clone https://github.com/epezent/implot.git
git clone https://github.com/glfw/glfw.git
