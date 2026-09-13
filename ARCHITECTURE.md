# Architecture & Implementation Guide

## Overview

This document describes the internal architecture of the flight control simulation, focusing on the optimization layers and assembly kernel implementations.

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│  simulation.py                                          │
│  ├─ FlightControlSimulation (main entry point)         │
│  ├─ AircraftParameters (Boeing 737-800 specs)          │
│  ├─ Aerodynamics (coefficient database)                │
│  ├─ Atmosphere (ISA model)                             │
│  ├─ Autopilot (PID controllers)                        │
│  └─ Visualization (Matplotlib 3D, PFD, systems panel)  │
└──────────────────────┬──────────────────────────────────┘
                       │ calls
                       ▼
┌─────────────────────────────────────────────────────────┐
│  flight_control_optimized.py (optimization dispatch)   │
│  ├─ compute_forces_moments_optimized()                 │
│  │  ├─ [if ASM available] → flight_control_asm (C ext)│
│  │  └─ [fallback] → compute_forces_moments_numba()    │
│  ├─ fast_quaternion_rotate() [@njit Numba]            │
│  ├─ fast_matrix_multiply() [@njit Numba]              │
│  └─ compute_rotation_matrix_fast() [@njit Numba]      │
└──────────────────────┬──────────────────────────────────┘
                       │
        ┌──────────────┴──────────────┐
        ▼                             ▼
    ┌────────────────┐        ┌──────────────────┐
    │  Assembly Ext  │        │ Numba (Python)   │
    │ (C extension)  │        │ JIT compiled     │
    └────────────────┘        └──────────────────┘
        │                            │
        ▼                            ▼
    ┌────────────────────────────────────────┐
    │  flight_control_wrapper.c (Python API) │
    │  - compute_forces_moments              │
    │  - matrix_multiply_4x4                 │
    │  - quaternion_rotate                   │
    └────────┬─────────────────────────────────┘
             │
             ▼
    ┌────────────────────────────────────────┐
    │  flight_control_asm.asm (x86-64 ASM)   │
    │  - compute_dynamic_pressure_asm        │
    │  - compute_forces_moments_asm          │
    │  - matrix_multiply_4x4_asm             │
    │  - quaternion_rotate_asm               │
    └────────────────────────────────────────┘
```

## Data Flow: ODE Integration

1. **Main Loop** (`simulation.py:FlightControlSimulation.update()`)
   - Gets current aircraft state (12 elements)
   - Reads control inputs from sliders or autopilot
   - Calls `scipy.integrate.solve_ivp()` with RK45 method

2. **ODE Right-Hand Side** (`aircraft_dynamics()`)
   - Computes forces and moments using optimized dispatch
   - Solves 6-DOF rigid body equations
   - Returns 12 state derivatives

3. **Force/Moment Computation** (`compute_forces_moments_optimized()`)
   - Checks if assembly is available
   - Calls assembly kernel OR Numba fallback
   - Returns [Fx, Fy, Fz, Mx, My, Mz]

4. **State Integration**
   - RK45 integrator advances state by one time step
   - New state is stored in history for visualization

## Assembly Kernel Interface

### C-to-Assembly Calling Convention (System V AMD64 ABI)

All assembly functions follow the x86-64 System V ABI:
- **Integer arguments**: `rdi`, `rsi`, `rdx`, `rcx`, `r8`, `r9`
- **Float arguments**: `xmm0`-`xmm7`
- **Return values**: `rax` (integer), `xmm0` (float)
- **Callee-saved registers**: `rbx`, `rbp`, `r12`-`r15`
- **Caller-saved registers**: `rax`, `rcx`, `rdx`, `rsi`, `rdi`, `r8`-`r11`

### Function: `compute_forces_moments_asm()`

**Signature:**
```c
void compute_forces_moments_asm(const double* params, const double* state, 
                               const double* controls, double* result)
```

**Parameters (System V ABI):**
- `rdi` = `params` pointer (26 doubles)
- `rsi` = `state` pointer (12 doubles)
- `rdx` = `controls` pointer (4 doubles)
- `rcx` = `result` pointer (6 doubles output)

**Parameter Layout (offsets in bytes from `params` base):**

```
Offset  Size  Name                Description
──────  ────  ────────────────    ──────────────────────────
0       8     rho                 Air density (kg/m³)
8       8     CL0                 Zero-lift coefficient
16      8     CLa                 Lift curve slope (per rad)
24      8     CLq                 Pitch rate damping coeff
32      8     MAC                 Mean aerodynamic chord (m)
40      8     CLde                Elevator effectiveness
48      8     S                   Wing area (m²)
56      8     CD0                 Parasite drag coefficient
64      8     K                   Lift-induced drag factor
72      8     CDde                Elevator drag coefficient
80      8     CY_beta             Sideslip effectiveness
88      8     CY_dr               Rudder side force coeff
96      8     Cm0                 Pitch moment bias
104     8     Cm_alpha            Alpha pitch moment coeff
112     8     Cm_q                Pitch damping coefficient
120     8     Cm_de               Elevator pitch moment
128     8     Cl_beta             Roll damping from sideslip
136     8     Cl_p                Roll damping from roll rate
144     8     Cl_r                Roll from yaw rate
152     8     Cl_da               Aileron effectiveness
160     8     Cl_dr               Rudder roll moment
168     8     Cn_beta             Yaw moment from sideslip
176     8     Cn_p                Yaw damping from roll rate
184     8     Cn_r                Yaw damping from yaw rate
192     8     Cn_da               Aileron yaw coupling
200     8     Cn_dr               Rudder yaw moment
208     8     wing_span           Wing span (m)
```

**State Layout (offsets in bytes from `state` base):**

```
Offset  Size  Name        Description
──────  ────  ──────      ──────────────────────────
0       8     V           True airspeed (m/s)
8       8     alpha       Angle of attack (rad)
16      8     beta        Sideslip angle (rad)
24      8     p           Roll rate (rad/s)
32      8     q           Pitch rate (rad/s)
40      8     r           Yaw rate (rad/s)
48      8     phi         Roll angle (rad)
56      8     theta       Pitch angle (rad)
64      8     psi         Yaw angle (rad)
72      8     pos_n       North position (m)
80      8     pos_e       East position (m)
88      8     pos_d       Down position (m)
```

**Controls Layout (offsets in bytes from `controls` base):**

```
Offset  Size  Name        Description
──────  ────  ──────      ──────────────────────────
0       8     delta_e     Elevator deflection [-1, 1] normalized
8       8     delta_a     Aileron deflection [-1, 1] normalized
16      8     delta_r     Rudder deflection [-1, 1] normalized
24      8     delta_t     Throttle [0, 1] normalized
```

**Result Layout (output, 6 doubles):**

```
Offset  Size  Name        Description
──────  ────  ──────      ──────────────────────────
0       8     Fx          Longitudinal force (N)
8       8     Fy          Lateral force (N)
16      8     Fz          Vertical force (N)
24      8     Mx          Roll moment (N·m)
32      8     My          Pitch moment (N·m)
40      8     Mz          Yaw moment (N·m)
```

### Function: `quaternion_rotate_asm()`

**Signature:**
```c
void quaternion_rotate_asm(const double* quat, const double* vec, double* result)
```

**Parameters:**
- `rdi` = `quat` pointer (4 doubles: [w, x, y, z])
- `rsi` = `vec` pointer (3 doubles: [x, y, z])
- `rdx` = `result` pointer (3 doubles output: [x', y', z'])

**Algorithm:**
Performs quaternion rotation using Hamilton product: `v' = q * v * q_conj`

1. Convert vector to pure quaternion: `v_quat = [0, vx, vy, vz]`
2. Compute `temp = q * v_quat`
3. Compute `q_conj = [w, -x, -y, -z]`
4. Compute `result = temp * q_conj`
5. Extract vector part (ignore w component)

### Function: `matrix_multiply_4x4_asm()`

**Signature:**
```c
void matrix_multiply_4x4_asm(const double* A, const double* B, double* C)
```

**Parameters:**
- `rdi` = A matrix (16 doubles, row-major)
- `rsi` = B matrix (16 doubles, row-major)
- `rdx` = C matrix output (16 doubles, row-major)

**Layout (4×4 row-major):**
```
Offset   0    8   16   24
0-32:   [a00 a01 a02 a03]
32-64:  [a10 a11 a12 a13]
64-96:  [a20 a21 a22 a23]
96-128: [a30 a31 a32 a33]
```

## Python-C-Assembly Integration

### Build Process

```
1. User runs: bash build_asm.sh

2. build_asm.sh:
   ├─ Checks for NASM (installs if missing)
   ├─ Assembles flight_control_asm.asm → flight_control_asm.o
   └─ Calls: python setup.py build_ext --inplace

3. setup.py:
   ├─ Links flight_control_asm.o with flight_control_wrapper.c
   ├─ Compiles with -O3 -march=native
   └─ Produces: flight_control_asm.*.so (shared library)

4. Python imports:
   ├─ import flight_control_asm (loads .so)
   ├─ Calls fca.compute_forces_moments(params, state, controls)
   └─ Returns numpy array
```

### Array Validation in C Wrapper

Before calling assembly functions, `flight_control_wrapper.c` validates:

```c
validate_array(arr, expected_size, name):
  - Check arr is not NULL
  - Check arr.dtype == NPY_DOUBLE
  - Check arr.size >= expected_size
  - Return 0 (error) or 1 (valid)
```

If validation fails, Python exception is raised with descriptive message.

## Fallback Mechanism

**Dispatch Logic in `compute_forces_moments_optimized()`:**

```python
if self.use_asm:
    try:
        result = fca.compute_forces_moments(params, state, controls)
        return result
    except Exception as e:
        logger.error(f"Assembly failed: {e}")
        self.use_asm = False  # Disable for rest of run
        return self.compute_forces_moments_numba(state, controls, params)
else:
    return self.compute_forces_moments_numba(state, controls, params)
```

**Rationale:**
- Assembly failures are likely systemic (bad parameters, memory issue)
- Once failed, keep fallback enabled for stability
- Log errors for post-mortem debugging

## Performance Characteristics

### Assembly vs Numba

Assembly optimizations target:
- **Hotspot:** Force/moment calculation (called ~100× per second in real-time sim)
- **Compute:** ~50 double operations per call
- **Bottleneck:** Floating-point arithmetic (not memory-bound)

Expected speedup (x86-64):
- Numba (JIT): 5–10× speedup vs pure Python
- Assembly: 1.5–2× speedup vs Numba (small for CPU-bound code)

**Reality check:** Assembly provides ~15–20% speedup for force calculations. Full simulation speedup is ~5% due to orchestration overhead.

### Register Usage in Kernels

**compute_forces_moments_asm:**
- Registers: `rdi`, `rsi`, `rdx`, `rcx` (params)
- XMM: `xmm0`–`xmm12` (intermediates)
- Stack: 80 bytes local storage

**quaternion_rotate_asm:**
- Registers: `rdi`, `rsi`, `rdx` (args), `rbp` (frame)
- XMM: `xmm0`–`xmm12` (quaternion, vector, temps)
- Stack: 96 bytes local storage

## Testing & Validation

### Unit Tests (`test_assembly_optimizations.py`)

1. **Mathematical Properties**
   - Quaternion rotation preserves magnitude
   - Rotation matrices are orthogonal (R^T R = I, det(R) = 1)
   - Matrix multiplication is associative

2. **Physics Constraints**
   - Positive alpha → downward Fz (lift)
   - Dynamic pressure > 0 for V > 0
   - Forces/moments are finite

3. **Regression Tests** (when ASM available)
   - Assembly output ≈ Numba output (< 1e-5 relative error)
   - Test multiple flight regimes (climb, descent, turn)

### Running Tests

```bash
pytest test_assembly_optimizations.py -v
pytest test_assembly_optimizations.py::TestAssemblyVsNumba -v  # ASM only
```

## Platform Support

### Supported Platforms
- **Linux x86-64** (primary)
- **macOS x86-64** (with NASM, untested)

### Unsupported Platforms
- **Windows**: Requires NASM elf64 support (cross-compile only)
- **ARM/ARM64**: Requires rewrite of all assembly kernels
- **WebAssembly**: Would need asm.js or wasm port

### Graceful Degradation
- On unsupported platforms, build fails silently
- Python fallback (Numba) runs automatically
- No simulation errors; just slower performance

## Debugging Assembly Issues

### Common Problems

| Symptom | Cause | Fix |
|---------|-------|-----|
| `ImportError: No module named flight_control_asm` | Assembly didn't build | Run `bash build_asm.sh` |
| Segmentation fault in force calculation | Array size mismatch | Check param count (must be 26) |
| Wrong forces produced | Incorrect offset in assembly | Verify parameter offsets in .asm |
| Assembly slower than Numba | Inefficient register spilling | Profile with `perf` |

### Profiling Commands

```bash
# Measure assembly performance
python -m cProfile -s cumtime flight_control_optimized.py

# Profile with perf (Linux)
perf record -g python simulation.py
perf report

# Disassemble to verify compilation
objdump -d flight_control_asm.o | grep -A50 compute_forces_moments_asm
```

## Future Improvements

1. **SIMD Vectorization**
   - Use AVX-256 for parallel force calculations
   - Batch multiple aircraft states
   - Expected 2–3× speedup over current scalar code

2. **ARM64 Port**
   - Rewrite kernels for AArch64 ABI
   - Use NEON SIMD instructions
   - Support Apple Silicon (M1/M2)

3. **WebAssembly**
   - Compile Numba to WebAssembly
   - Run simulation in browser
   - No assembly needed (wasm is portable)

4. **GPU Acceleration**
   - CUDA/OpenCL for matrix operations
   - Quaternion batch rotations
   - Expected 5–10× speedup for large simulations

## References

- **System V AMD64 ABI**: https://refspecs.linuxbase.org/elf/x86-64-abi-0.99.pdf
- **NASM Assembly Manual**: https://www.nasm.us/doc/
- **NumPy C API**: https://numpy.org/doc/stable/reference/c-api/
- **Aerodynamics**: Anderson, J. D. "Aircraft Performance and Design" (2nd ed.)
- **Flight Dynamics**: Stevens & Lewis, "Aircraft Control and Simulation" (3rd ed.)
