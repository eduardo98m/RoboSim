# RoboSim

RoboSim (ROBOtics + SIMulator) is a projects that aims to create an easy to use and easy to customize rigid body physics simulator for robotics applications. The project is currently in development.

## Description



## Instalation

```

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