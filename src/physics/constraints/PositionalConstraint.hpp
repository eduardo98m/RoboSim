
#include "physics/math/math.hpp"
#include "physics/bodies/Body.hpp"

class PositionalConstraint{
    private:
        // Reference to the first body
        Body * body_1;
        // Reference to the second body
        Body * body_2;
        // Constraint position relative to the first body
        vec3 r_1;
        // Constraint position relative to the second body
        vec3 r_2;
        // Value of the constraint
        vec3 value;
        // Lagrange multiplier
        vec3 lambda;
        // Compliance
        scalar compliance;
        // Damping
        scalar damping;
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
        PositionalConstraint(Body * body_1, 
                             Body * body_2, 
                             vec3 r_1 = vec3{0.0, 0.0, 0.0}, 
                             vec3 r_2 = vec3{0.0, 0.0, 0.0}, 
                             scalar compliance = 0.0, 
                             scalar damping = 0.0);
        // Destructor
        ~PositionalConstraint();
        void set_value(vec3 value);
        
        /*
        * Function that computest the delta lambda for the positional constraint
        * @param c The magnitude of the constraint
        * @param lambda The current value of the constraint lagrange multiplier
        * @param alpha The compliance of the constraint
        * @param w_1 The first body's generalized inverse mass
        * @param w_2 The second body's generalized inverse mass
        * @param inverse_time_step The inverse of the time step
        */
        scalar compute_positional_delta_lambda(scalar c, scalar lambda, scalar alpha, scalar w_1, scalar w_2, scalar inverse_time_step);

};