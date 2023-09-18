#!/bin/bash

# Create the build directory if it doesn't exist
mkdir -p build

# Navigate into the build directory
cd build

# Run CMake to generate the Makefile
cmake ..

# Run make to build the project
make