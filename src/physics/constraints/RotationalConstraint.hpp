
#include "physics/math/math.hpp"
#include "physics/bodies/Body.hpp"

class RotationalConstraint{
    private:
        // Reference to the first body
        Body * body_1;
        // Reference to the second body
        Body * body_2;
        // Constraint position relative to the first body
        vec3 r_1;
        // Constraint position relative to the second body
        vec3 r_2;
        // Normal vector of the constraint
        vec3 n;
        // Magnitude of the constraint
        scalar magnitude;
        // Lagrange multiplier
        scalar lambda;
        // Compliance
        scalar compliance;
        // Damping
        scalar damping;
        // Torque value of the constraint
        vec3 torque;
    public:
        // Constructor
        /*
        * Class constructor
        * @param body_1 The first body of the constraint
        * @param body_2 The second body of the constraint
        * @param r_1 The constraint position relative to the first body (in the local coordinates of the first body)
        * @param r_2 The constraint position relative to the second body (in the local coordinates of the second body)
        * @param compliance The compliance of the constraint
        * @param damping The damping of the constraint
        */
        RotationalConstraint(Body * body_1, 
                             Body * body_2, 
                             vec3 r_1 = vec3{0.0, 0.0, 0.0}, 
                             vec3 r_2 = vec3{0.0, 0.0, 0.0}, 
                             scalar compliance = 0.0, 
                             scalar damping = 0.0);
        // Destructor
        ~RotationalConstraint();

        /*
        * Function that sets the value of the constriant.
        * Note: The value of the constraint is given by a 3D vector that is the decomposed into 
        * Its normal and its magnitude.
        */
        void set_value(vec3 value);

        
        /*
        * Function that computest the delta lambda for the Rotational constraint
        * @param w_1 The first body's generalized inverse mass
        * @param w_2 The second body's generalized inverse mass
        * @param inverse_time_step The inverse of the time step
        */
        scalar compute_rotational_delta_lambda(scalar w_1, scalar w_2, scalar inverse_time_step);

        /*
        * Function that computes (and applies) the Rotational constraint. 
        * @param inverse_time_step The inverse of the time step (specifically the sub step h)
        */
        void apply_constraint(scalar inverse_time_step);

};