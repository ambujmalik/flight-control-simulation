; flight_control_asm.asm
; x86-64 Assembly optimizations for flight control calculations
; NASM syntax for 64-bit Linux/Unix targets

section .data
    align 16
    half: dd 0.5, 0.5, 0.5, 0.5
    two: dd 2.0, 2.0, 2.0, 2.0
    one: dq 1.0
    g_const: dd 9.81, 9.81, 9.81, 9.81

section .text
    global compute_dynamic_pressure_asm
    global compute_forces_moments_asm
    global matrix_multiply_4x4_asm
    global quaternion_rotate_asm

; double compute_dynamic_pressure_asm(double rho, double V)
; Input: xmm0 = rho (air density), xmm1 = V (velocity)
; Output: xmm0 = dynamic pressure (0.5 * rho * V^2)
compute_dynamic_pressure_asm:
    mulsd xmm0, xmm1           ; rho * V
    mulsd xmm0, xmm1           ; rho * V^2
    movsd xmm1, [rel half]     ; load 0.5
    mulsd xmm0, xmm1           ; 0.5 * rho * V^2 = Q
    ret

; void compute_forces_moments_asm(const double* params, const double* state, 
;                                const double* controls, double* result)
; Parameters: rdi = params, rsi = state, rdx = controls, rcx = result
;
; params layout (offsets in bytes):
;   0: rho                 8: CL0              16: CLa             24: CLq
;   32: MAC                40: CLde            48: S (wing area)   56: CD0
;   64: K                  72: CDde            80: CY_beta         88: CY_dr
;   96: Cm0                104: Cm_alpha       112: Cm_q           120: Cm_de
;   128: Cl_beta           136: Cl_p           144: Cl_r           152: Cl_da
;   160: Cl_dr             168: Cn_beta        176: Cn_p           184: Cn_r
;   192: Cn_da             200: Cn_dr          208: wing_span
;
; state layout (offsets in bytes):
;   0: V                   8: alpha            16: beta            24: p
;   32: q                  40: r               48: phi             56: theta
;   64: psi                72: pos_n           80: pos_e           88: pos_d
;
; controls layout:
;   0: delta_e             8: delta_a          16: delta_r         24: delta_t
;
; result layout:
;   0: Fx                  8: Fy               16: Fz              24: Mx
;   32: My                 40: Mz
compute_forces_moments_asm:
    push rbp
    mov rbp, rsp
    sub rsp, 80                ; local storage for intermediate values
    
    ; Load state variables
    movsd xmm0, [rsi]          ; V
    movsd xmm1, [rsi+8]        ; alpha
    movsd xmm2, [rsi+16]       ; beta
    movsd xmm3, [rsi+24]       ; p
    movsd xmm4, [rsi+32]       ; q
    movsd xmm5, [rsi+40]       ; r
    
    ; Compute dynamic pressure (Q = 0.5 * rho * V^2)
    movsd xmm6, [rdi]          ; rho
    movsd xmm7, xmm0           ; V
    mulsd xmm6, xmm0           ; rho * V
    mulsd xmm6, xmm0           ; rho * V^2
    movsd xmm10, [rel half]    ; load 0.5
    mulsd xmm6, xmm10          ; xmm6 = Q (dynamic pressure)
    movsd [rbp-8], xmm6        ; save Q
    
    ; Lift coefficient: CL = CL0 + CLa*alpha + CLq*(q*MAC/(2*V)) + CLde*delta_e
    movsd xmm7, [rdi+8]        ; CL0
    movsd xmm8, [rdi+16]       ; CLa
    mulsd xmm8, xmm1           ; CLa * alpha
    addsd xmm7, xmm8           ; CL = CL0 + CLa*alpha
    
    ; Add pitch rate damping: CLq * (q * MAC / (2*V))
    movsd xmm8, [rdi+24]       ; CLq
    movsd xmm9, [rdi+32]       ; MAC
    movsd xmm11, xmm4          ; q
    mulsd xmm11, xmm9          ; q * MAC
    movsd xmm12, xmm0          ; V
    addsd xmm12, xmm12         ; 2*V
    divsd xmm11, xmm12         ; (q*MAC)/(2*V)
    mulsd xmm8, xmm11          ; CLq * (q*MAC/(2*V))
    addsd xmm7, xmm8           ; CL += damping term
    
    ; Add elevator control: CLde * delta_e
    movsd xmm8, [rdi+40]       ; CLde
    movsd xmm9, [rdx]          ; delta_e
    mulsd xmm8, xmm9           ; CLde * delta_e
    addsd xmm7, xmm8           ; CL += control term
    movsd [rbp-16], xmm7       ; save CL
    
    ; Drag coefficient: CD = CD0 + K*CL^2 + CDde*delta_e^2
    movsd xmm7, [rdi+56]       ; CD0
    movsd xmm8, [rdi+64]       ; K
    movsd xmm9, [rbp-16]       ; CL
    movsd xmm11, xmm9          ; CL
    mulsd xmm11, xmm9          ; CL^2
    mulsd xmm8, xmm11          ; K*CL^2
    addsd xmm7, xmm8           ; CD = CD0 + K*CL^2
    
    ; Add elevator induced drag: CDde*delta_e^2
    movsd xmm8, [rdi+72]       ; CDde
    movsd xmm9, [rdx]          ; delta_e
    mulsd xmm9, xmm9           ; delta_e^2
    mulsd xmm8, xmm9           ; CDde*delta_e^2
    addsd xmm7, xmm8           ; CD += induced drag term
    movsd [rbp-24], xmm7       ; save CD
    
    ; Side force: CY = CY_beta*beta + CY_dr*delta_r
    movsd xmm7, [rdi+80]       ; CY_beta
    mulsd xmm7, xmm2           ; CY_beta * beta
    movsd xmm8, [rdi+88]       ; CY_dr
    movsd xmm9, [rdx+16]       ; delta_r
    mulsd xmm8, xmm9           ; CY_dr * delta_r
    addsd xmm7, xmm8           ; CY = CY_beta*beta + CY_dr*delta_r
    movsd [rbp-32], xmm7       ; save CY
    
    ; Forces in wind axes
    movsd xmm6, [rbp-8]        ; Q
    movsd xmm7, [rdi+48]       ; S (wing area)
    movsd xmm8, [rbp-16]       ; CL
    mulsd xmm8, xmm6           ; CL * Q
    mulsd xmm8, xmm7           ; CL * Q * S (lift magnitude)
    movsd [rbp-40], xmm8       ; save L_lift
    
    movsd xmm8, [rbp-24]       ; CD
    mulsd xmm8, xmm6           ; CD * Q
    mulsd xmm8, xmm7           ; CD * Q * S (drag magnitude)
    movsd [rbp-48], xmm8       ; save D_drag
    
    movsd xmm8, [rbp-32]       ; CY
    mulsd xmm8, xmm6           ; CY * Q
    mulsd xmm8, xmm7           ; CY * Q * S (side force magnitude)
    movsd [rbp-56], xmm8       ; save Y_side
    
    ; Transform to body axes using cos/sin of alpha
    ; For simplicity, use small angle approximation here
    ; In production, would use precomputed trig or separate trig kernel
    movsd xmm10, xmm1          ; alpha
    movsd xmm11, xmm1
    mulsd xmm11, xmm1          ; alpha^2
    
    ; sin(alpha) ≈ alpha - alpha^3/6
    movsd xmm8, xmm1           ; alpha
    movsd xmm9, xmm11
    mulsd xmm9, xmm1           ; alpha^3
    movsd xmm12, [rel two]
    addsd xmm12, xmm12
    addsd xmm12, xmm12         ; 6.0 approximation
    divsd xmm9, xmm12
    subsd xmm8, xmm9           ; sin(alpha)
    movsd [rbp-64], xmm8       ; save sin(alpha)
    
    ; cos(alpha) ≈ 1 - alpha^2/2
    movsd xmm8, xmm11          ; alpha^2
    movsd xmm9, [rel half]
    mulsd xmm8, xmm9           ; alpha^2/2
    movsd xmm12, [rel one]
    subsd xmm12, xmm8          ; cos(alpha)
    movsd [rbp-72], xmm12      ; save cos(alpha)
    
    ; Fx = -D*cos(alpha) + L*sin(alpha)
    movsd xmm8, [rbp-48]       ; D_drag
    movsd xmm9, [rbp-72]       ; cos(alpha)
    mulsd xmm8, xmm9           ; D*cos(alpha)
    movsd xmm10, [rbp-40]      ; L_lift
    movsd xmm11, [rbp-64]      ; sin(alpha)
    mulsd xmm10, xmm11         ; L*sin(alpha)
    subsd xmm10, xmm8          ; L*sin(alpha) - D*cos(alpha)
    movsd [rcx], xmm10         ; store Fx (result[0])
    
    ; Fy = Y_side (no transformation needed for side axis)
    movsd xmm8, [rbp-56]       ; Y_side
    movsd [rcx+8], xmm8        ; store Fy (result[1])
    
    ; Fz = -D*sin(alpha) - L*cos(alpha)
    movsd xmm8, [rbp-48]       ; D_drag
    movsd xmm9, [rbp-64]       ; sin(alpha)
    mulsd xmm8, xmm9           ; D*sin(alpha)
    movsd xmm10, [rbp-40]      ; L_lift
    movsd xmm11, [rbp-72]      ; cos(alpha)
    mulsd xmm10, xmm11         ; L*cos(alpha)
    addsd xmm8, xmm10          ; D*sin(alpha) + L*cos(alpha)
    negsd xmm8                 ; -(D*sin + L*cos)
    movsd [rcx+16], xmm8       ; store Fz (result[2])
    
    ; Pitching moment: Cm = Cm0 + Cm_alpha*alpha + Cm_q*(q*MAC/(2*V)) + Cm_de*delta_e
    movsd xmm8, [rdi+96]       ; Cm0
    movsd xmm9, [rdi+104]      ; Cm_alpha
    movsd xmm10, [rsi+8]       ; alpha
    mulsd xmm9, xmm10          ; Cm_alpha * alpha
    addsd xmm8, xmm9           ; Cm0 + Cm_alpha*alpha
    
    ; Pitch damping: Cm_q * (q*MAC/(2*V))
    movsd xmm9, [rdi+112]      ; Cm_q
    movsd xmm10, [rsi+32]      ; q
    movsd xmm11, [rdi+32]      ; MAC
    mulsd xmm10, xmm11         ; q*MAC
    movsd xmm12, [rsi]         ; V
    addsd xmm12, xmm12         ; 2*V
    divsd xmm10, xmm12         ; (q*MAC)/(2*V)
    mulsd xmm9, xmm10          ; Cm_q * damping
    addsd xmm8, xmm9           ; Cm += damping
    
    ; Elevator control: Cm_de * delta_e
    movsd xmm9, [rdi+120]      ; Cm_de
    movsd xmm10, [rdx]         ; delta_e
    mulsd xmm9, xmm10          ; Cm_de * delta_e
    addsd xmm8, xmm9           ; Cm += control term
    
    ; My = Q * S * MAC * Cm
    movsd xmm9, [rbp-8]        ; Q
    movsd xmm10, [rdi+48]      ; S
    mulsd xmm9, xmm10          ; Q*S
    movsd xmm10, [rdi+32]      ; MAC
    mulsd xmm9, xmm10          ; Q*S*MAC
    mulsd xmm8, xmm9           ; Cm * Q*S*MAC
    movsd [rcx+32], xmm8       ; store My (result[4])
    
    ; Rolling moment: Cl = Cl_beta*beta + Cl_p*(p*b/(2*V)) + Cl_da*delta_a + Cl_dr*delta_r
    movsd xmm8, [rdi+128]      ; Cl_beta
    movsd xmm9, [rsi+16]       ; beta
    mulsd xmm8, xmm9           ; Cl_beta*beta
    
    movsd xmm9, [rdi+136]      ; Cl_p
    movsd xmm10, [rsi+24]      ; p
    movsd xmm11, [rdi+208]     ; wing_span (b)
    mulsd xmm10, xmm11         ; p*b
    movsd xmm12, [rsi]         ; V
    addsd xmm12, xmm12         ; 2*V
    divsd xmm10, xmm12         ; (p*b)/(2*V)
    mulsd xmm9, xmm10          ; Cl_p * damping
    addsd xmm8, xmm9           ; Cl += roll damping
    
    movsd xmm9, [rdi+152]      ; Cl_da
    movsd xmm10, [rdx+8]       ; delta_a
    mulsd xmm9, xmm10          ; Cl_da*delta_a
    addsd xmm8, xmm9           ; Cl += aileron control
    
    movsd xmm9, [rdi+160]      ; Cl_dr
    movsd xmm10, [rdx+16]      ; delta_r
    mulsd xmm9, xmm10          ; Cl_dr*delta_r
    addsd xmm8, xmm9           ; Cl += rudder effect
    
    ; Mx = Q * S * b * Cl
    movsd xmm9, [rbp-8]        ; Q
    movsd xmm10, [rdi+48]      ; S
    mulsd xmm9, xmm10          ; Q*S
    movsd xmm10, [rdi+208]     ; b (wing_span)
    mulsd xmm9, xmm10          ; Q*S*b
    mulsd xmm8, xmm9           ; Cl * Q*S*b
    movsd [rcx+24], xmm8       ; store Mx (result[3])
    
    ; Yawing moment: Cn = Cn_beta*beta + Cn_p*(p*b/(2*V)) + Cn_r*(r*b/(2*V)) + Cn_da*delta_a + Cn_dr*delta_r
    movsd xmm8, [rdi+168]      ; Cn_beta
    movsd xmm9, [rsi+16]       ; beta
    mulsd xmm8, xmm9           ; Cn_beta*beta
    
    movsd xmm9, [rdi+176]      ; Cn_p
    movsd xmm10, [rsi+24]      ; p
    movsd xmm11, [rdi+208]     ; b
    mulsd xmm10, xmm11         ; p*b
    movsd xmm12, [rsi]         ; V
    addsd xmm12, xmm12         ; 2*V
    divsd xmm10, xmm12         ; (p*b)/(2*V)
    mulsd xmm9, xmm10          ; Cn_p*damping
    addsd xmm8, xmm9           ; Cn += roll rate effect
    
    movsd xmm9, [rdi+184]      ; Cn_r
    movsd xmm10, [rsi+40]      ; r
    movsd xmm11, [rdi+208]     ; b
    mulsd xmm10, xmm11         ; r*b
    movsd xmm12, [rsi]         ; V
    addsd xmm12, xmm12         ; 2*V
    divsd xmm10, xmm12         ; (r*b)/(2*V)
    mulsd xmm9, xmm10          ; Cn_r*damping
    addsd xmm8, xmm9           ; Cn += yaw damping
    
    movsd xmm9, [rdi+192]      ; Cn_da
    movsd xmm10, [rdx+8]       ; delta_a
    mulsd xmm9, xmm10          ; Cn_da*delta_a
    addsd xmm8, xmm9           ; Cn += aileron effect
    
    movsd xmm9, [rdi+200]      ; Cn_dr
    movsd xmm10, [rdx+16]      ; delta_r
    mulsd xmm9, xmm10          ; Cn_dr*delta_r
    addsd xmm8, xmm9           ; Cn += rudder control
    
    ; Mz = Q * S * b * Cn
    movsd xmm9, [rbp-8]        ; Q
    movsd xmm10, [rdi+48]      ; S
    mulsd xmm9, xmm10          ; Q*S
    movsd xmm10, [rdi+208]     ; b
    mulsd xmm9, xmm10          ; Q*S*b
    mulsd xmm8, xmm9           ; Cn * Q*S*b
    movsd [rcx+40], xmm8       ; store Mz (result[5])
    
    add rsp, 80
    pop rbp
    ret

; void matrix_multiply_4x4_asm(const double* A, const double* B, double* C)
; rdi = A (4x4 matrix), rsi = B (4x4 matrix), rdx = C (result 4x4 matrix)
; Uses scalar double precision FP for portability
matrix_multiply_4x4_asm:
    push rbp
    mov rbp, rsp
    
    mov r8, 0                  ; row counter
.row_loop:
    cmp r8, 4
    jge .row_done
    
    mov r9, 0                  ; column counter
.col_loop:
    cmp r9, 4
    jge .col_done
    
    pxor xmm0, xmm0            ; accumulator = 0
    mov r10, 0                 ; inner loop counter
    
.inner_loop:
    cmp r10, 4
    jge .inner_done
    
    ; Load A[r8*4 + r10]
    mov rax, r8
    imul rax, 32               ; 4 elements * 8 bytes per row
    lea r11, [rdi + rax]
    movsd xmm1, [r11 + r10*8]
    
    ; Load B[r10*4 + r9]
    mov rax, r10
    imul rax, 32
    lea r11, [rsi + rax]
    movsd xmm2, [r11 + r9*8]
    
    ; Multiply and accumulate
    mulsd xmm1, xmm2
    addsd xmm0, xmm1
    
    inc r10
    jmp .inner_loop
    
.inner_done:
    ; Store result in C[r8*4 + r9]
    mov rax, r8
    imul rax, 32
    lea r11, [rdx + rax]
    movsd [r11 + r9*8], xmm0
    
    inc r9
    jmp .col_loop
    
.col_done:
    inc r8
    jmp .row_loop
    
.row_done:
    pop rbp
    ret

; void quaternion_rotate_asm(const double* quat, const double* vec, double* result)
; rdi = quat [w, x, y, z], rsi = vec [x, y, z], rdx = result
; Performs quaternion rotation: v' = q * v * q_conjugate
quaternion_rotate_asm:
    push rbp
    mov rbp, rsp
    sub rsp, 96                ; local storage for intermediate quaternions
    
    ; Load quaternion [w, x, y, z]
    movsd xmm0, [rdi]          ; w
    movsd xmm1, [rdi+8]        ; x
    movsd xmm2, [rdi+16]       ; y
    movsd xmm3, [rdi+24]       ; z
    
    ; Save quaternion to stack
    movsd [rbp-8], xmm0        ; qw
    movsd [rbp-16], xmm1       ; qx
    movsd [rbp-24], xmm2       ; qy
    movsd [rbp-32], xmm3       ; qz
    
    ; Load vector [x, y, z]
    movsd xmm4, [rsi]          ; vx
    movsd xmm5, [rsi+8]        ; vy
    movsd xmm6, [rsi+16]       ; vz
    
    ; Convert vector to pure quaternion: v_quat = [0, vx, vy, vz]
    movsd [rbp-40], xmm4       ; temp_vx
    movsd [rbp-48], xmm5       ; temp_vy
    movsd [rbp-56], xmm6       ; temp_vz
    
    ; Hamilton product: temp = q * v_quat
    ; temp.w = -qx*vx - qy*vy - qz*vz
    movsd xmm7, [rbp-16]       ; qx
    movsd xmm8, [rbp-40]       ; vx
    mulsd xmm7, xmm8           ; qx*vx
    movsd xmm9, [rbp-24]       ; qy
    movsd xmm10, [rbp-48]      ; vy
    mulsd xmm9, xmm10          ; qy*vy
    addsd xmm7, xmm9           ; qx*vx + qy*vy
    movsd xmm11, [rbp-32]      ; qz
    movsd xmm12, [rbp-56]      ; vz
    mulsd xmm11, xmm12         ; qz*vz
    addsd xmm7, xmm11          ; qx*vx + qy*vy + qz*vz
    negsd xmm7                 ; -(qx*vx + qy*vy + qz*vz)
    movsd [rbp-64], xmm7       ; temp.w
    
    ; temp.x = qw*vx + qy*vz - qz*vy
    movsd xmm7, [rbp-8]        ; qw
    movsd xmm8, [rbp-40]       ; vx
    mulsd xmm7, xmm8           ; qw*vx
    movsd xmm9, [rbp-24]       ; qy
    movsd xmm10, [rbp-56]      ; vz
    mulsd xmm9, xmm10          ; qy*vz
    addsd xmm7, xmm9           ; qw*vx + qy*vz
    movsd xmm11, [rbp-32]      ; qz
    movsd xmm12, [rbp-48]      ; vy
    mulsd xmm11, xmm12         ; qz*vy
    subsd xmm7, xmm11          ; qw*vx + qy*vz - qz*vy
    movsd [rbp-72], xmm7       ; temp.x
    
    ; temp.y = qw*vy - qx*vz + qz*vx
    movsd xmm7, [rbp-8]        ; qw
    movsd xmm8, [rbp-48]       ; vy
    mulsd xmm7, xmm8           ; qw*vy
    movsd xmm9, [rbp-16]       ; qx
    movsd xmm10, [rbp-56]      ; vz
    mulsd xmm9, xmm10          ; qx*vz
    subsd xmm7, xmm9           ; qw*vy - qx*vz
    movsd xmm11, [rbp-32]      ; qz
    movsd xmm12, [rbp-40]      ; vx
    mulsd xmm11, xmm12         ; qz*vx
    addsd xmm7, xmm11          ; qw*vy - qx*vz + qz*vx
    movsd [rbp-80], xmm7       ; temp.y
    
    ; temp.z = qw*vz + qx*vy - qy*vx
    movsd xmm7, [rbp-8]        ; qw
    movsd xmm8, [rbp-56]       ; vz
    mulsd xmm7, xmm8           ; qw*vz
    movsd xmm9, [rbp-16]       ; qx
    movsd xmm10, [rbp-48]      ; vy
    mulsd xmm9, xmm10          ; qx*vy
    addsd xmm7, xmm9           ; qw*vz + qx*vy
    movsd xmm11, [rbp-24]      ; qy
    movsd xmm12, [rbp-40]      ; vx
    mulsd xmm11, xmm12         ; qy*vx
    subsd xmm7, xmm11          ; qw*vz + qx*vy - qy*vx
    movsd [rbp-88], xmm7       ; temp.z
    
    ; Quaternion conjugate: q_conj = [qw, -qx, -qy, -qz]
    movsd xmm0, [rbp-8]        ; qw
    movsd xmm1, [rbp-16]       ; qx
    negsd xmm1                 ; -qx
    movsd xmm2, [rbp-24]       ; qy
    negsd xmm2                 ; -qy
    movsd xmm3, [rbp-32]       ; qz
    negsd xmm3                 ; -qz
    
    ; Final Hamilton product: result = temp * q_conj
    ; result.x = temp.w*(-qx) + temp.x*qw + temp.y*(-qz) - temp.z*(-qy)
    movsd xmm4, [rbp-64]       ; temp.w
    mulsd xmm4, xmm1           ; temp.w*(-qx)
    movsd xmm5, [rbp-72]       ; temp.x
    mulsd xmm5, xmm0           ; temp.x*qw
    addsd xmm4, xmm5           ; temp.w*(-qx) + temp.x*qw
    movsd xmm6, [rbp-80]       ; temp.y
    mulsd xmm6, xmm2           ; temp.y*(-qz)
    addsd xmm4, xmm6           ; + temp.y*(-qz)
    movsd xmm7, [rbp-88]       ; temp.z
    mulsd xmm7, xmm3           ; temp.z*(-qy)
    subsd xmm4, xmm7           ; - temp.z*(-qy)
    movsd [rdx], xmm4          ; result[0] = result.x
    
    ; result.y = temp.w*(-qy) + temp.y*qw + temp.z*(-qx) - temp.x*(-qz)
    movsd xmm4, [rbp-64]       ; temp.w
    mulsd xmm4, xmm2           ; temp.w*(-qy)
    movsd xmm5, [rbp-80]       ; temp.y
    mulsd xmm5, xmm0           ; temp.y*qw
    addsd xmm4, xmm5           ; temp.w*(-qy) + temp.y*qw
    movsd xmm6, [rbp-88]       ; temp.z
    mulsd xmm6, xmm1           ; temp.z*(-qx)
    addsd xmm4, xmm6           ; + temp.z*(-qx)
    movsd xmm7, [rbp-72]       ; temp.x
    mulsd xmm7, xmm3           ; temp.x*(-qz)
    subsd xmm4, xmm7           ; - temp.x*(-qz)
    movsd [rdx+8], xmm4        ; result[1] = result.y
    
    ; result.z = temp.w*(-qz) + temp.z*qw + temp.x*(-qy) - temp.y*(-qx)
    movsd xmm4, [rbp-64]       ; temp.w
    mulsd xmm4, xmm3           ; temp.w*(-qz)
    movsd xmm5, [rbp-88]       ; temp.z
    mulsd xmm5, xmm0           ; temp.z*qw
    addsd xmm4, xmm5           ; temp.w*(-qz) + temp.z*qw
    movsd xmm6, [rbp-72]       ; temp.x
    mulsd xmm6, xmm2           ; temp.x*(-qy)
    addsd xmm4, xmm6           ; + temp.x*(-qy)
    movsd xmm7, [rbp-80]       ; temp.y
    mulsd xmm7, xmm1           ; temp.y*(-qx)
    subsd xmm4, xmm7           ; - temp.y*(-qx)
    movsd [rdx+16], xmm4       ; result[2] = result.z
    
    add rsp, 96
    pop rbp
    ret
