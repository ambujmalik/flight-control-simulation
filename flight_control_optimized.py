# flight_control_optimized.py
import numpy as np
import flight_control_asm as fca
from numba import jit, njit
import ctypes
import logging

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class OptimizedFlightControlSimulation:
    def __init__(self):
        # Initialize as before...
        self.use_asm = self.detect_asm_support()
        
    def detect_asm_support(self):
        """Detect if assembly optimizations are available"""
        try:
            import flight_control_asm
            logger.info("Assembly optimizations available")
            return True
        except ImportError as e:
            logger.warning(f"Assembly optimizations not available: {e}")
            logger.info("Using Python/Numba fallback")
            return False
    
    @njit(fastmath=True)
    def compute_forces_moments_numba(self, state, controls, params):
        """Numba-optimized version for fallback
        
        Args:
            state: 12-element array [V, alpha, beta, p, q, r, phi, theta, psi, pos_n, pos_e, pos_d]
            controls: 4-element array [delta_e, delta_a, delta_r, delta_t]
            params: 26-element array of aerodynamic coefficients
        
        Returns:
            6-element array [Fx, Fy, Fz, Mx, My, Mz]
        """
        V, alpha, beta, p, q, r, phi, theta, psi, pos_n, pos_e, pos_d = state
        delta_e, delta_a, delta_r, delta_t = controls
        
        # Extract parameters
        rho, wing_area, wing_span, mac = params[0], params[1], params[2], params[3]
        
        # Dynamic pressure
        Q = 0.5 * rho * V * V
        
        # Aerodynamic coefficients (simplified)
        CL = 0.4 + 5.0 * alpha + 5.0 * (q * mac / (2 * V)) + 0.3 * delta_e
        CD = 0.03 + 0.3 * alpha * alpha + 0.01 * delta_e * delta_e
        CY = -0.8 * beta + 0.15 * delta_r
        
        # Forces
        L = Q * wing_area * CL
        D = Q * wing_area * CD
        Y = Q * wing_area * CY
        
        # Transform to body axes
        Fx = -D * np.cos(alpha) + L * np.sin(alpha)
        Fy = Y
        Fz = -D * np.sin(alpha) - L * np.cos(alpha)
        
        # Moments (simplified)
        Mx = Q * wing_area * wing_span * (-0.1 * beta - 0.5 * (p * wing_span / (2 * V)) + 0.15 * delta_a)
        My = Q * wing_area * mac * (0.05 - 1.0 * alpha - 20.0 * (q * mac / (2 * V)) - 1.2 * delta_e)
        Mz = Q * wing_area * wing_span * (0.15 * beta - 0.3 * (r * wing_span / (2 * V)) - 0.1 * delta_r)
        
        return np.array([Fx, Fy, Fz, Mx, My, Mz])
    
    def compute_forces_moments_optimized(self, state, controls, params):
        """Use assembly optimization if available, otherwise use Numba
        
        Attempts to call assembly-optimized force/moment calculation.
        On failure, logs the error and disables assembly for remainder of run.
        
        Args:
            state: 12-element array [V, alpha, beta, p, q, r, phi, theta, psi, pos_n, pos_e, pos_d]
            controls: 4-element array [delta_e, delta_a, delta_r, delta_t]
            params: 26-element array of aerodynamic coefficients
        
        Returns:
            6-element array [Fx, Fy, Fz, Mx, My, Mz]
        """
        if self.use_asm:
            try:
                result = fca.compute_forces_moments(params, state, controls)
                return result
            except Exception as e:
                logger.error(f"Assembly optimization failed during compute_forces_moments: {type(e).__name__}: {e}")
                logger.info("Falling back to Numba-optimized Python implementation")
                self.use_asm = False
                return self.compute_forces_moments_numba(state, controls, params)
        else:
            return self.compute_forces_moments_numba(state, controls, params)
    
    @staticmethod
    @njit
    def fast_matrix_multiply(A, B):
        """Optimized matrix multiplication
        
        Args:
            A: MxN matrix
            B: NxP matrix
        
        Returns:
            MxP result matrix
        """
        m, n = A.shape
        n, p = B.shape
        C = np.zeros((m, p))
        
        for i in range(m):
            for k in range(n):
                if A[i, k] != 0:  # Skip zeros for sparse matrices
                    for j in range(p):
                        C[i, j] += A[i, k] * B[k, j]
        return C
    
    @staticmethod
    @njit
    def fast_quaternion_rotate(q, v):
        """Optimized quaternion rotation using Hamilton product
        
        Performs rotation: v' = q * v * q_conjugate
        where v is converted to pure quaternion [0, vx, vy, vz]
        
        Args:
            q: quaternion [w, x, y, z] (must be normalized)
            v: vector [x, y, z]
        
        Returns:
            rotated vector [x', y', z']
        """
        qw, qx, qy, qz = q
        vx, vy, vz = v
        
        # Convert vector to quaternion
        v_quat = np.array([0, vx, vy, vz])
        
        # q * v_quat
        temp = np.array([
            -qx*vx - qy*vy - qz*vz,
            qw*vx + qy*vz - qz*vy,
            qw*vy - qx*vz + qz*vx,
            qw*vz + qx*vy - qy*vx
        ])
        
        # q_conjugate = [qw, -qx, -qy, -qz]
        # temp * q_conjugate
        result = np.array([
            -temp[1]*qx - temp[2]*qy - temp[3]*qz,
            temp[0]*qx + temp[2]*qz - temp[3]*qy,
            temp[0]*qy - temp[1]*qz + temp[3]*qx,
            temp[0]*qz + temp[1]*qy - temp[2]*qx
        ])
        
        return result[1:]  # Return vector part [x', y', z']
    
    def optimized_aircraft_dynamics(self, t, state):
        """Optimized version of aircraft dynamics"""
        V, alpha, beta, p, q, r, phi, theta, psi, pos_n, pos_e, pos_d = state
        
        # Prepare parameters for force/moment calculation
        params = np.array([
            self.atmosphere.isa_density(-pos_d),  # rho
            self.aircraft.wing_area,
            self.aircraft.wing_span,
            self.aircraft.mean_aerodynamic_chord,
            # Add remaining parameters as needed
        ])
        
        # Use optimized force/moment calculation
        controls = np.array([self.delta_e, self.delta_a, self.delta_r, self.delta_t])
        F, M = self.compute_forces_moments_optimized(state, controls, params)
        Fx, Fy, Fz = F
        Mx, My, Mz = M
        
        # Mass properties
        m = self.aircraft.mass - self.fuel_used
        Ixx, Iyy, Izz, Ixz = (self.aircraft.Ixx, self.aircraft.Iyy, 
                             self.aircraft.Izz, self.aircraft.Ixz)
        
        # Kinematic equations - optimized
        u = V * np.cos(alpha) * np.cos(beta)
        v = V * np.sin(beta)
        w = V * np.sin(alpha) * np.cos(beta)
        
        # Force equations
        udot = (Fx / m) - (q * w) + (r * v)
        vdot = (Fy / m) - (r * u) + (p * w)
        wdot = (Fz / m) - (p * v) + (q * u)
        
        # Precompute common terms for moment equations
        denom = 1.0 / (Ixx * Izz - Ixz * Ixz)
        
        pdot = denom * (Izz * Mx + Ixz * Mz - 
                       Ixz * (Iyy - Ixx - Izz) * p * q + 
                       (Ixz * Ixz + Izz * (Izz - Iyy)) * q * r)
        
        qdot = (My - (Ixx - Izz) * p * r - Ixz * (p * p - r * r)) / Iyy
        
        rdot = denom * (Ixz * Mx + Ixx * Mz + 
                       Ixz * (Iyy - Ixx - Izz) * q * r + 
                       (Ixz * Ixz + Ixx * (Ixx - Iyy)) * p * q)
        
        # Navigation equations using optimized matrix multiplication
        # (Rotation matrix calculation optimized in the main class)
        
        # Convert force derivatives to state derivatives
        Vdot = (u * udot + v * vdot + w * wdot) / V if V != 0 else 0.0
        alphadot = (u * wdot - w * udot) / (u * u + w * w) if (u * u + w * w) != 0 else 0.0
        betadot = (V * vdot - v * Vdot) / (V * V * np.cos(beta)) if (V != 0 and np.cos(beta) != 0) else 0.0
        
        # Euler angle derivatives
        tan_theta = np.tan(theta)
        cos_theta = np.cos(theta)
        sin_phi = np.sin(phi)
        cos_phi = np.cos(phi)
        
        phidot = p + (q * sin_phi + r * cos_phi) * tan_theta
        thetadot = q * cos_phi - r * sin_phi
        psidot = (q * sin_phi + r * cos_phi) / cos_theta if cos_theta != 0 else 0.0
        
        # Position derivatives
        # Use precomputed rotation matrix or optimized calculation
        R_bi = self.compute_rotation_matrix_fast(phi, theta, psi)
        vel_body = np.array([u, v, w])
        pos_dot = R_bi @ vel_body
        
        return [Vdot, alphadot, betadot, pdot, qdot, rdot, 
                phidot, thetadot, psidot, pos_dot[0], pos_dot[1], -pos_dot[2]]
    
    @staticmethod
    @njit
    def compute_rotation_matrix_fast(phi, theta, psi):
        """Optimized rotation matrix calculation
        
        Computes body-to-inertial rotation matrix from Euler angles.
        Applies ZYX rotation sequence (yaw, pitch, roll).
        
        Args:
            phi: roll angle (rad)
            theta: pitch angle (rad)
            psi: yaw angle (rad)
        
        Returns:
            3x3 rotation matrix
        """
        cos_phi = np.cos(phi)
        sin_phi = np.sin(phi)
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)
        
        R = np.zeros((3, 3))
        
        R[0, 0] = cos_theta * cos_psi
        R[0, 1] = sin_phi * sin_theta * cos_psi - cos_phi * sin_psi
        R[0, 2] = cos_phi * sin_theta * cos_psi + sin_phi * sin_psi
        
        R[1, 0] = cos_theta * sin_psi
        R[1, 1] = sin_phi * sin_theta * sin_psi + cos_phi * cos_psi
        R[1, 2] = cos_phi * sin_theta * sin_psi - sin_phi * cos_psi
        
        R[2, 0] = -sin_theta
        R[2, 1] = sin_phi * cos_theta
        R[2, 2] = cos_phi * cos_theta
        
        return R

# Performance benchmarking
def benchmark_optimizations():
    """Benchmark the performance improvements"""
    import time
    
    sim = OptimizedFlightControlSimulation()
    state = np.random.randn(12)
    controls = np.random.randn(4)
    params = np.array([1.225, 125, 35, 4.5, 0.024, 0.045])  # rho, S, b, MAC, CD0, K
    
    # Benchmark Python version
    start = time.time()
    for _ in range(10000):
        sim.compute_forces_moments_numba(state, controls, params)
    python_time = time.time() - start
    
    # Benchmark optimized version
    if sim.use_asm:
        start = time.time()
        for _ in range(10000):
            sim.compute_forces_moments_optimized(state, controls, params)
        asm_time = time.time() - start
        logger.info(f"Python: {python_time:.4f}s, Assembly: {asm_time:.4f}s")
        logger.info(f"Speedup: {python_time/asm_time:.2f}x")
    else:
        logger.info(f"Python time: {python_time:.4f}s")

if __name__ == "__main__":
    benchmark_optimizations()
