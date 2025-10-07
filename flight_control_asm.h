// flight_control_asm.h
#ifndef FLIGHT_CONTROL_ASM_H
#define FLIGHT_CONTROL_ASM_H

#ifdef __cplusplus
extern "C" {
#endif

double compute_dynamic_pressure_asm(double rho, double V);
void compute_forces_moments_asm(const double* params, const double* state, 
                               const double* controls, double* result);
void matrix_multiply_4x4_asm(const double* A, const double* B, double* C);
void quaternion_rotate_asm(const double* quat, const double* vec, double* result);

#ifdef __cplusplus
}
#endif

#endif
