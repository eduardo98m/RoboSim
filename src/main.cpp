
#include "physics/math/math.hpp" 
#include "physics/bodies/DynamicBody.hpp"


// Import the sine function from the standard library
#include <math.h>
#include <iostream>
#include "Visualizer.hpp"

int main(int argc, char *argv[]){



    // Define a vector 3
    vec3 v = vec3{1.0, 2.1, 69.0};

    // Create a dynamic body
    vec3 pos = vec3{0.0, 0.0, 0.0};
    quat ori = quat{1.0, 0.0, 0.0, 0.0};
    vec3 lin_vel = vec3{0.0, 0.0, 0.0};
    vec3 ang_vel = vec3{0.0, 0.0, 0.0};
    scalar mass = 1.0;
    mat3 inertia_tensor = mat3{1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};
    DynamicBody body = DynamicBody();

    // Print the vector
    std::cout << "The vector is: " << v << std::endl;
    // Print the properties of the body
    std::cout << "The position of the body is: " << body.position << std::endl;

    std::cout << "inertia_tensor" << inertia_tensor << std::endl;

    std::cout << "holar" << body.get_rotational_generalized_inverse_mass(v) << "\n";
    

    return 0;
}