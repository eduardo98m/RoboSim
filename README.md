# Robo Sim

## Description

This project serves as a base for creating GUI applications using C++, Dear ImGui, and ImPlot. Currently it works on Linux systems (tested on Linux Mint) and uses the OpenGL and GLFW backends of ImGui, but you can modify it to work in your particullar system (a PR for your OS will be appreciated). The project includes a thirdParty folder containing external repositories for ImGui, ImPlot, and GLFW.

## Dependencies

First you have to get the imgui, implot and glfw repositories. You can do this by clonning the 

```bash
chmod +x getThirdParty.sh
```

```bash
./getThirdParty.sh
```

Before you can build this project, you need to install some dependencies. You can install these packages using the following commands:

For the glfw backend you will need:

```bash
sudo apt-get install libxi-dev libxcursor-dev libxinerama-dev libxrandr-dev  
```

If not already installed you should install OpenGL libraries

```bash
sudo apt-get install libgl1-mesa-dev
```

###  Optional dependencies

Optionally you can install the following packages if you want to generate the documentation:

```bash
sudo apt-get install doxygen
```


## Building the Project
The project uses CMake for building. A build.sh script is provided to automate the build process. 
This script executes the CMake script and builds the project in a `build` folder.

```bash
chmod +x build.sh
```

``` bash
./build.sh
```

Please modify this template as needed to fit your project’s specific requirements and details. If you have any more questions or need further assistance, feel free to ask! 😊