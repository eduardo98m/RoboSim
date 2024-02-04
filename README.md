<p align="center">
  <img src="RoboSimWhiteLogo.svg" alt="RoboSim Logo" width="20%"/>
</p>


<p align="center">
  <a href="https://opensource.org/licenses/MIT"><img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="License: MIT"></a>
  <a href="https://GitHub.com/eduardo98m/RoboSim/issues/"><img src="https://img.shields.io/github/issues/eduardo98m/RoboSim.svg" alt="GitHub issues"></a>
  <a href="https://GitHub.com/eduardo98m/RoboSim/pulls/"><img src="https://img.shields.io/github/issues-pr/eduardo98m/RoboSim.svg" alt="GitHub pull-requests"></a>
</p>

# RoboSim

RoboSim (ROBOtics + SIMulator) is a projects that aims to create an easy to use and easy to customize rigid body physics simulator for robotics applications. The project is currently under early development.

![RoboSim](Image2.png)

## Current State of the Simulator

You can see the current state of the simulator in the following video:

[![RoboSim Demo](http://img.youtube.com/vi/cLKrVKXZ3UE/0.jpg)](http://www.youtube.com/watch?v=cLKrVKXZ3UE "RoboSim Demo")


## Installation

**Note:** These instructions have been tested on Linux, but there should be no problems following them on Windows (or Mac) as well.

### Prerequisites

Before you begin, ensure you have met the following requirements:

* You have installed the latest version of [raylib](https://www.raylib.com/) and [CMake](https://cmake.org/).

### Installing RoboSim

To install RoboSim, follow these steps:

1. Clone this repository recursively:

```bash
git clone --recursive https://github.com/your_username/RoboSim.git
```

2. Navigate to the cloned repository:

```bash
cd RoboSim
```
3. Use CMake to build the project :

```bash
mkdir build
cd build
cmake ..
make
```

# TODO:
* [ ] Build intructions/Tutorial
* [ ] Documentation
* [-] Interface Simulator <-> Visualizer (WIP)
* [ ] **Collisions**
    * [-] Add collision shapes
        * [x] Sphere
        * [x] Box
        * [ ] Capsule
        * [ ] Plane
        * [ ] Heightmap
    * [ ] Add collision detection and response:
        * [ ] Sphere <-> Sphere
        * [ ] Sphere <-> Box
        * [ ] Sphere <-> Capsule
        * [ ] Sphere <-> Plane
        * [ ] Sphere <-> Heightmap
        * [ ] Box <-> Box
        * [ ] Box <-> Capsule
        * [ ] Box <-> Plane
        * [ ] Box <-> Heightmap
        * [ ] Capsule <-> Capsule
        * [ ] Capsule <-> Plane
        * [ ] Capsule <-> Heightmap
    * [ ] Add collision manifolds.
    * [ ] Implement the Sutherland–Hodgman algorithm.
* [ ] **Constraints**
    * [ ] Contact constraints.
    * [ ] Prismatic joint constraint
        * [ ] Limited
        * [ ] Driven
        * [ ] Damped
    * [ ] Revolute joint constraint
        * [x] Limited
        * [x] Driven
            * [x] Position driven
            * [-] Speed driven (It is implemented but there are errors on the implementation: target speed doesnt mathc actual speed). 
        * [x] Damped
    * [ ] Spherica joint
        * [ ] Limited
        * [ ] Damped
* [ ] URDF importer
* [ ] Paralelization with OpenMP

    




###  Optional dependencies


Please modify this template as needed to fit your project’s specific requirements and details. If you have any more questions or need further assistance, feel free to ask! 😊

![Alt text](image.png)