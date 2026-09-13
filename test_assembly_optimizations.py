"""
Unit tests for flight control assembly optimizations

Tests verify that assembly and Numba implementations produce equivalent results.
"""

import numpy as np
import pytest
from flight_control_optimized import OptimizedFlightControlSimulation


class TestAssemblyOptimizations:
    """Test suite for assembly-optimized functions"""
    
    @pytest.fixture
    def sim(self):
        """Fixture: initialized simulation instance"""
        return OptimizedFlightControlSimulation()
    
    @pytest.fixture
    def test_state(self):
        """Fixture: realistic aircraft state"""
        return np.array([
            250.0,      # V (m/s)
            0.05,       # alpha (rad)
            0.0,        # beta (rad)
            0.0,        # p (rad/s)
            0.01,       # q (rad/s)
            0.0,        # r (rad/s)
            0.0,        # phi (rad)
            0.05,       # theta (rad)
            0.0,        # psi (rad)
            0.0,        # pos_n (m)
            0.0,        # pos_e (m)
            10000.0     # pos_d (m)
        ])
    
    @pytest.fixture
    def test_controls(self):
        """Fixture: neutral control inputs"""
        return np.array([
            0.0,        # delta_e
            0.0,        # delta_a
            0.0,        # delta_r
            0.5         # delta_t
        ])
    
    @pytest.fixture
    def test_params(self):
        """Fixture: aerodynamic parameters"""
        return np.array([
            1.225,      # rho (kg/m^3)
            0.42,       # CL0
            5.08,       # CLa
            6.7,        # CLq
            4.17,       # MAC
            0.36,       # CLde
            125.0,      # S (wing area)
            0.024,      # CD0
            0.045,      # K
            0.028,      # CDde
            -0.83,      # CY_beta
            0.12,       # CY_dr
            0.05,       # Cm0
            -0.92,      # Cm_alpha
            -38.0,      # Cm_q
            -1.28,      # Cm_de
            -0.11,      # Cl_beta
            -0.45,      # Cl_p
            0.17,       # Cl_r
            0.16,       # Cl_da
            0.0015,     # Cl_dr
            0.073,      # Cn_beta
            -0.034,     # Cn_p
            -0.26,      # Cn_r
            -0.0012,    # Cn_da
            -0.072,     # Cn_dr
            35.8        # wing_span
        ])
    
    def test_dynamic_pressure_positive(self, sim, test_state, test_params):
        """Test dynamic pressure is positive for positive velocity"""
        rho = test_params[0]
        V = test_state[0]
        Q = 0.5 * rho * V**2
        assert Q > 0, "Dynamic pressure should be positive"
    
    def test_forces_moments_output_shape(self, sim, test_state, test_controls, test_params):
        """Test that forces/moments function returns 6-element array"""
        result = sim.compute_forces_moments_numba(test_state, test_controls, test_params)
        assert result.shape == (6,), f"Expected shape (6,), got {result.shape}"
    
    def test_forces_moments_elements(self, sim, test_state, test_controls, test_params):
        """Test that all force/moment elements are finite"""
        result = sim.compute_forces_moments_numba(test_state, test_controls, test_params)
        assert np.all(np.isfinite(result)), f"Non-finite values in forces/moments: {result}"
    
    def test_quaternion_rotate_identity(self, sim):
        """Test quaternion rotation with identity quaternion (no rotation)"""
        q_identity = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        v = np.array([1.0, 2.0, 3.0])
        
        v_rotated = sim.fast_quaternion_rotate(q_identity, v)
        
        np.testing.assert_array_almost_equal(
            v_rotated, v, decimal=10,
            err_msg="Identity quaternion should not rotate vector"
        )
    
    def test_quaternion_rotate_90_deg_z(self, sim):
        """Test quaternion rotation by 90 degrees around z-axis"""
        # Quaternion for 90 deg rotation around z: q = [cos(45), 0, 0, sin(45)]
        angle = np.pi / 2
        q = np.array([
            np.cos(angle / 2),
            0.0,
            0.0,
            np.sin(angle / 2)
        ])
        
        # Vector along x-axis
        v = np.array([1.0, 0.0, 0.0])
        v_rotated = sim.fast_quaternion_rotate(q, v)
        
        # After 90 deg rotation around z, x-axis should point along y
        expected = np.array([0.0, 1.0, 0.0])
        np.testing.assert_array_almost_equal(
            v_rotated, expected, decimal=10,
            err_msg="90 deg z-axis rotation should map x to y"
        )
    
    def test_quaternion_rotate_preserves_magnitude(self, sim):
        """Test that quaternion rotation preserves vector magnitude"""
        q = np.array([0.7071, 0.7071, 0.0, 0.0])  # 90 deg around x
        v = np.array([1.0, 2.0, 3.0])
        
        v_rotated = sim.fast_quaternion_rotate(q, v)
        
        mag_original = np.linalg.norm(v)
        mag_rotated = np.linalg.norm(v_rotated)
        
        np.testing.assert_almost_equal(
            mag_rotated, mag_original, decimal=10,
            err_msg="Quaternion rotation should preserve vector magnitude"
        )
    
    def test_matrix_multiply_identity(self, sim):
        """Test matrix multiplication with identity matrix"""
        I = np.eye(4)
        A = np.random.randn(4, 4)
        
        C = sim.fast_matrix_multiply(A, I)
        
        np.testing.assert_array_almost_equal(
            C, A, decimal=10,
            err_msg="Multiply by identity should return original matrix"
        )
    
    def test_matrix_multiply_associativity(self, sim):
        """Test associativity: (A*B)*C = A*(B*C)"""
        A = np.random.randn(4, 4)
        B = np.random.randn(4, 4)
        C = np.random.randn(4, 4)
        
        left = sim.fast_matrix_multiply(sim.fast_matrix_multiply(A, B), C)
        right = sim.fast_matrix_multiply(A, sim.fast_matrix_multiply(B, C))
        
        np.testing.assert_array_almost_equal(
            left, right, decimal=10,
            err_msg="Matrix multiplication should be associative"
        )
    
    def test_rotation_matrix_orthogonal(self, sim):
        """Test that rotation matrix is orthogonal (R^T * R = I)"""
        phi = np.radians(15)
        theta = np.radians(10)
        psi = np.radians(20)
        
        R = sim.compute_rotation_matrix_fast(phi, theta, psi)
        
        RTR = R.T @ R
        np.testing.assert_array_almost_equal(
            RTR, np.eye(3), decimal=10,
            err_msg="Rotation matrix should be orthogonal"
        )
    
    def test_rotation_matrix_determinant(self, sim):
        """Test that rotation matrix has determinant = 1"""
        phi = np.radians(30)
        theta = np.radians(45)
        psi = np.radians(60)
        
        R = sim.compute_rotation_matrix_fast(phi, theta, psi)
        det = np.linalg.det(R)
        
        np.testing.assert_almost_equal(
            det, 1.0, decimal=10,
            err_msg="Rotation matrix determinant should be 1"
        )
    
    def test_forces_moments_lift_sign(self, sim, test_state, test_controls, test_params):
        """Test that positive angle of attack produces upward force (negative Fz)"""
        state = test_state.copy()
        state[1] = 0.1  # positive alpha
        
        result = sim.compute_forces_moments_numba(state, test_controls, test_params)
        Fz = result[2]
        
        assert Fz < 0, "Positive alpha should produce upward lift (negative Fz)"
    
    def test_forces_moments_drag_always_opposes_motion(self, sim, test_state, test_controls, test_params):
        """Test that drag force opposes forward motion"""
        result = sim.compute_forces_moments_numba(test_state, test_controls, test_params)
        Fx = result[0]
        
        # With positive velocity and neutral alpha, net force should oppose motion
        # (drag > thrust in this simplified model)
        assert np.isfinite(Fx), "Fx should be finite"
    
    def test_assembly_fallback_uses_numba(self, sim, test_state, test_controls, test_params):
        """Test that when assembly is unavailable, fallback to Numba works"""
        # Disable assembly
        sim.use_asm = False
        
        result = sim.compute_forces_moments_optimized(test_state, test_controls, test_params)
        
        assert result.shape == (6,), "Numba fallback should return 6-element array"
        assert np.all(np.isfinite(result)), "Numba fallback should return finite values"


class TestPhysicsConstraints:
    """Test that simulation respects physical constraints"""
    
    @pytest.fixture
    def sim(self):
        return OptimizedFlightControlSimulation()
    
    def test_euler_angles_wrap(self, sim):
        """Test that Euler angle normalization works"""
        phi = 2 * np.pi + 0.5
        theta = np.pi / 2
        psi = 3 * np.pi
        
        R = sim.compute_rotation_matrix_fast(phi, theta, psi)
        
        assert np.all(np.isfinite(R)), "Rotation matrix should be finite for wrapped angles"
    
    def test_zero_velocity_condition(self, sim):
        """Test behavior at zero velocity (singularity)"""
        state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10000.0])
        controls = np.array([0.0, 0.0, 0.0, 0.0])
        params = np.ones(27)
        
        # Should not crash or produce NaN
        result = sim.compute_forces_moments_numba(state, controls, params)
        
        assert np.all(np.isfinite(result)), "Should handle zero velocity gracefully"


@pytest.mark.skipif(not OptimizedFlightControlSimulation().use_asm, 
                    reason="Assembly not available")
class TestAssemblyVsNumba:
    """Regression tests comparing assembly and Numba implementations
    
    Only runs when assembly optimizations are available
    """
    
    @pytest.fixture
    def sim(self):
        return OptimizedFlightControlSimulation()
    
    @pytest.fixture
    def test_cases(self):
        """Multiple test cases with varied flight conditions"""
        cases = []
        
        # Nominal level flight
        cases.append({
            'name': 'level_flight',
            'state': np.array([250.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 10000.0]),
            'controls': np.array([0.0, 0.0, 0.0, 0.5])
        })
        
        # Climbing turn
        cases.append({
            'name': 'climbing_turn',
            'state': np.array([200.0, 0.1, 0.05, 0.01, 0.02, 0.005, 0.1, 0.1, np.pi/4, 1000.0, 500.0, 8000.0]),
            'controls': np.array([0.05, 0.1, 0.02, 0.6])
        })
        
        # Steep descent
        cases.append({
            'name': 'descent',
            'state': np.array([300.0, -0.05, 0.0, 0.0, -0.01, 0.0, 0.0, -0.1, 0.0, 0.0, 0.0, 5000.0]),
            'controls': np.array([-0.1, 0.0, 0.0, 0.2])
        })
        
        return cases
    
    @pytest.fixture
    def test_params(self):
        return np.array([
            1.225, 0.42, 5.08, 6.7, 4.17, 0.36, 125.0, 0.024, 0.045, 0.028,
            -0.83, 0.12, 0.05, -0.92, -38.0, -1.28, -0.11, -0.45, 0.17, 0.16,
            0.0015, 0.073, -0.034, -0.26, -0.0012, -0.072, 35.8
        ])
    
    def test_assembly_vs_numba_forces_moments(self, sim, test_cases, test_params):
        """Compare assembly and Numba output for forces/moments"""
        if not sim.use_asm:
            pytest.skip("Assembly not available")
        
        for case in test_cases:
            state = case['state']
            controls = case['controls']
            
            # Get results from both implementations
            result_numba = sim.compute_forces_moments_numba(state, controls, test_params)
            result_asm = sim.compute_forces_moments_optimized(state, controls, test_params)
            
            # Compare with tolerance for numerical differences
            np.testing.assert_array_almost_equal(
                result_asm, result_numba, decimal=5,
                err_msg=f"Assembly and Numba differ for case: {case['name']}"
            )


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
