#include "stm32f4xx_hal.h"
#include "main.h"
#include "app_mems.h"
#include "motion_fx.h"
#include "arm_math.h"

// Dimension definitions
#define STATE_DIM 8
#define INPUT_DIM 2
#define OUTPUT_DIM 2


// Defining state vector
 float32_t state_vector[/*gyro z, ang_vel_z, x motion, v, force1, force2, volt1, volt2*/];
 float32_t reference_vector[];
// Defininig K Matrix
float32_t K_matrix[INPUT_DIM * STATE_DIM] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

arm_matrix_instance_f32 K;
float32_t control_vector[INPUT_DIM];

void control_update(float32_t* state_vector, float32_t* reference_vector){
    arm_matrix_instance_f32 x; // State vector
    arm_matrix_instance_f32 r; // Reference vector
    arm_matrix_instance_f32 u; // Control vector

    arm_mat_init_f32(&x, STATE_DIM, 1, state_vector);
    arm_mat_init_f32(&r, INPUT_DIM, 1, reference_vector);
    arm_mat_init_f32(&u, INPUT_DIM, 1, control_vector);

    // Calculate the control law u = -K*x + r
    arm_mat_mult_f32(&K, &x, &u);       // u = K*x
    arm_sub_f32(r.pData, u.pData, u.pData, INPUT_DIM); // u = r - u (since we are using u = r - Kx)

    // Copy the control output to the global control_vector
    arm_copy_f32(u.pData, control_vector, INPUT_DIM);
}

int control_process(control_flag){
	if(control_flag){
			 control_flag = 0;
			 control_update(state_vector, reference_vector);
	}
}
