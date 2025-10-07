import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button, RadioButtons
import matplotlib.gridspec as gridspec
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d
import time
from dataclasses import dataclass
from typing import List, Tuple, Optional
import math

@dataclass
class AircraftParameters:
    """Professional aircraft parameters for Boeing 737-800"""
    name: str = "Boeing 737-800"
    mass: float = 70000  # kg
    wing_area: float = 125.0  # m²
    wing_span: float = 35.8  # m
    mean_aerodynamic_chord: float = 4.17  # m
    Ixx: float = 5.2e6  # kg·m²
    Iyy: float = 8.1e6  # kg·m²
    Izz: float = 12.3e6  # kg·m²
    Ixz: float = 1.2e5  # kg·m²
    cg_position: np.ndarray = None  # Center of gravity
    
    # Engine parameters (CFM56-7B)
    max_thrust: float = 117000  # N per engine
    thrust_moment_arm: float = 8.5  # m from CG
    fuel_flow_max: float = 3.0  # kg/s per engine
    
    def __post_init__(self):
        if self.cg_position is None:
            self.cg_position = np.array([0, 0, 0])

class Atmosphere:
    """International Standard Atmosphere (ISA) model"""
    @staticmethod
    def isa_temperature(altitude: float) -> float:
        """Temperature in Kelvin at given altitude (m)"""
        if altitude <= 11000:  # Troposphere
            return 288.15 - 0.0065 * altitude
        elif altitude <= 20000:  # Lower stratosphere
            return 216.65
        else:  # Upper stratosphere
            return 216.65 + 0.001 * (altitude - 20000)
    
    @staticmethod
    def isa_pressure(altitude: float) -> float:
        """Pressure in Pa at given altitude (m)"""
        T0 = 288.15  # K
        P0 = 101325  # Pa
        g = 9.80665  # m/s²
        R = 287.05   # J/(kg·K)
        
        if altitude <= 11000:
            T = 288.15 - 0.0065 * altitude
            return P0 * (T / T0) ** (g / (0.0065 * R))
        else:
            P11 = 22632.0  # Pa at 11km
            T11 = 216.65   # K at 11km
            return P11 * np.exp(-g * (altitude - 11000) / (R * T11))
    
    @staticmethod
    def isa_density(altitude: float) -> float:
        """Density in kg/m³ at given altitude (m)"""
        T = Atmosphere.isa_temperature(altitude)
        P = Atmosphere.isa_pressure(altitude)
        R = 287.05  # J/(kg·K)
        return P / (R * T)

class Aerodynamics:
    """Professional aerodynamic model with coefficient tables"""
    
    def __init__(self, aircraft: AircraftParameters):
        self.aircraft = aircraft
        
        # Lift coefficients (CL)
        self.CL_alpha = 5.08  # per radian
        self.CL0 = 0.42
        self.CL_q = 6.7
        self.CL_de = 0.36
        self.CL_max = 1.4
        self.alpha_stall = 0.35  # rad (~20°)
        
        # Drag coefficients (CD)
        self.CD0 = 0.024
        self.K = 0.045  # Lift-induced drag factor
        self.CD_de = 0.028
        
        # Pitching moment coefficients (Cm)
        self.Cm0 = 0.05
        self.Cm_alpha = -0.92
        self.Cm_q = -38.0
        self.Cm_de = -1.28
        
        # Side force coefficients (CY)
        self.CY_beta = -0.83
        self.CY_p = 0.0
        self.CY_r = 0.0
        self.CY_da = 0.0
        self.CY_dr = 0.12
        
        # Rolling moment coefficients (Cl)
        self.Cl_beta = -0.11
        self.Cl_p = -0.45
        self.Cl_r = 0.17
        self.Cl_da = 0.16
        self.Cl_dr = 0.0015
        
        # Yawing moment coefficients (Cn)
        self.Cn_beta = 0.073
        self.Cn_p = -0.034
        self.Cn_r = -0.26
        self.Cn_da = -0.0012
        self.Cn_dr = -0.072

class WindModel:
    """Realistic wind and turbulence model"""
    
    def __init__(self):
        self.wind_n = 5.0  # North wind component (m/s)
        self.wind_e = -3.0  # East wind component (m/s)
        self.wind_d = 0.0  # Down wind component (m/s)
        self.turbulence_intensity = 0.1
        
    def get_wind(self, position: np.ndarray, time: float) -> np.ndarray:
        """Get wind vector at given position and time"""
        # Base wind
        wind = np.array([self.wind_n, self.wind_e, self.wind_d])
        
        # Add turbulence (Dryden model approximation)
        if self.turbulence_intensity > 0:
            turbulence = self.turbulence_intensity * np.array([
                np.sin(0.1 * time + position[0] * 0.001),
                np.cos(0.12 * time + position[1] * 0.001),
                np.sin(0.08 * time + position[2] * 0.002)
            ])
            wind += turbulence
        
        return wind

class Autopilot:
    """Professional autopilot system with PID controllers"""
    
    def __init__(self):
        # Altitude hold PID
        self.altitude_kp = 0.002
        self.altitude_ki = 0.0001
        self.altitude_kd = 0.01
        self.altitude_integral = 0.0
        self.altitude_error_prev = 0.0
        
        # Heading hold PID
        self.heading_kp = 0.8
        self.heading_ki = 0.05
        self.heading_kd = 0.2
        self.heading_integral = 0.0
        self.heading_error_prev = 0.0
        
        # Airspeed hold PID
        self.airspeed_kp = 0.01
        self.airspeed_ki = 0.001
        self.airspeed_kd = 0.005
        self.airspeed_integral = 0.0
        self.airspeed_error_prev = 0.0
        
        # Pitch attitude hold
        self.pitch_kp = 2.0
        self.pitch_ki = 0.1
        self.pitch_kd = 0.5
        
        # Turn coordinator
        self.bank_angle_limit = np.radians(30)
        
    def update(self, dt: float, current_state: dict, setpoints: dict) -> dict:
        """Update autopilot and return control commands"""
        controls = {}
        
        # Altitude hold with pitch control
        alt_error = setpoints['altitude'] - current_state['position_d']
        self.altitude_integral += alt_error * dt
        alt_derivative = (alt_error - self.altitude_error_prev) / dt
        
        pitch_command = (self.altitude_kp * alt_error + 
                        self.altitude_ki * self.altitude_integral + 
                        self.altitude_kd * alt_derivative)
        
        # Limit pitch command
        pitch_command = np.clip(pitch_command, -0.15, 0.15)
        
        # Pitch attitude hold
        pitch_error = pitch_command - current_state['theta']
        controls['elevator'] = self.pitch_kp * pitch_error
        
        # Heading hold with bank angle control
        heading_error = self.normalize_angle(setpoints['heading'] - current_state['psi'])
        self.heading_integral += heading_error * dt
        heading_derivative = (heading_error - self.heading_error_prev) / dt
        
        bank_command = (self.heading_kp * heading_error + 
                       self.heading_ki * self.heading_integral + 
                       self.heading_kd * heading_derivative)
        
        # Limit bank angle
        bank_command = np.clip(bank_command, -self.bank_angle_limit, self.bank_angle_limit)
        
        # Bank angle control
        bank_error = bank_command - current_state['phi']
        controls['aileron'] = 2.0 * bank_error - 0.1 * current_state['p']
        
        # Coordinated turn (rudder for sideslip minimization)
        controls['rudder'] = -0.5 * current_state['beta'] - 0.1 * current_state['r']
        
        # Airspeed hold with throttle control
        airspeed_error = setpoints['airspeed'] - current_state['V']
        self.airspeed_integral += airspeed_error * dt
        airspeed_derivative = (airspeed_error - self.airspeed_error_prev) / dt
        
        controls['throttle'] = 0.5 + (self.airspeed_kp * airspeed_error + 
                                     self.airspeed_ki * self.airspeed_integral + 
                                     self.airspeed_kd * airspeed_derivative)
        
        # Update previous errors
        self.altitude_error_prev = alt_error
        self.heading_error_prev = heading_error
        self.airspeed_error_prev = airspeed_error
        
        # Apply control limits
        controls['elevator'] = np.clip(controls['elevator'], -0.3, 0.3)
        controls['aileron'] = np.clip(controls['aileron'], -0.3, 0.3)
        controls['rudder'] = np.clip(controls['rudder'], -0.3, 0.3)
        controls['throttle'] = np.clip(controls['throttle'], 0.2, 0.9)
        
        return controls
    
    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

class FlightControlSimulation:
    def __init__(self):
        # Initialize subsystems
        self.aircraft = AircraftParameters()
        self.aero = Aerodynamics(self.aircraft)
        self.atmosphere = Atmosphere()
        self.wind = WindModel()
        self.autopilot = Autopilot()
        
        # Initial flight conditions
        self.initialize_state()
        
        # Autopilot setpoints
        self.altitude_setpoint = 10000  # m
        self.heading_setpoint = 0.0  # rad
        self.airspeed_setpoint = 250  # m/s
        
        # Simulation parameters
        self.dt = 0.01  # s
        self.time = 0.0
        self.max_time = 600  # s
        self.sim_speed = 1.0  # Simulation speed multiplier
        
        # Flight mode
        self.flight_mode = "AUTOPILOT"  # AUTOPILOT or MANUAL
        
        # Data recording
        self.time_history = []
        self.state_history = []
        self.control_history = []
        self.performance_history = []
        
        # Setup visualization
        self.setup_plots()
        
    def initialize_state(self):
        """Initialize aircraft state vector"""
        self.V = 250.0  # True airspeed (m/s)
        self.alpha = 0.05  # Angle of attack (rad)
        self.beta = 0.0  # Sideslip angle (rad)
        self.p = 0.0  # Roll rate (rad/s)
        self.q = 0.0  # Pitch rate (rad/s)
        self.r = 0.0  # Yaw rate (rad/s)
        self.phi = 0.0  # Roll angle (rad)
        self.theta = 0.05  # Pitch angle (rad)
        self.psi = 0.0  # Yaw angle (rad)
        self.position_n = 0.0  # North position (m)
        self.position_e = 0.0  # East position (m)
        self.position_d = 10000.0  # Down position (m) - positive down!
        
        # Controls
        self.delta_e = 0.0  # Elevator
        self.delta_a = 0.0  # Aileron
        self.delta_r = 0.0  # Rudder
        self.delta_t = 0.5  # Throttle
        
        # Engine parameters
        self.engine_thrust = 0.0
        self.fuel_flow = 0.0
        self.fuel_used = 0.0
    
    def get_state_vector(self) -> np.ndarray:
        """Return current state as vector"""
        return np.array([
            self.V, self.alpha, self.beta, self.p, self.q, self.r,
            self.phi, self.theta, self.psi, 
            self.position_n, self.position_e, self.position_d
        ])
    
    def set_state_vector(self, state: np.ndarray):
        """Set state from vector"""
        (self.V, self.alpha, self.beta, self.p, self.q, self.r,
         self.phi, self.theta, self.psi,
         self.position_n, self.position_e, self.position_d) = state
    
    def get_state_dict(self) -> dict:
        """Return current state as dictionary"""
        return {
            'V': self.V, 'alpha': self.alpha, 'beta': self.beta,
            'p': self.p, 'q': self.q, 'r': self.r,
            'phi': self.phi, 'theta': self.theta, 'psi': self.psi,
            'position_n': self.position_n, 'position_e': self.position_e, 
            'position_d': self.position_d
        }
    
    def compute_forces_and_moments(self, state: np.ndarray, controls: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Compute aerodynamic forces and moments"""
        V, alpha, beta, p, q, r, phi, theta, psi, pos_n, pos_e, pos_d = state
        delta_e, delta_a, delta_r, delta_t = controls
        
        # Get atmospheric conditions
        altitude = -pos_d  # Convert to altitude (up positive)
        rho = self.atmosphere.isa_density(altitude)
        Q = 0.5 * rho * V**2  # Dynamic pressure
        
        # Lift coefficient with stall model
        if alpha < self.aero.alpha_stall:
            CL_alpha = self.aero.CL_alpha
        else:
            # Stall model - reduced lift gradient
            CL_alpha = self.aero.CL_alpha * (1.0 - (alpha - self.aero.alpha_stall) / 0.3)
        
        CL = (self.aero.CL0 + CL_alpha * alpha + 
              self.aero.CL_q * (q * self.aircraft.mean_aerodynamic_chord / (2 * V)) + 
              self.aero.CL_de * delta_e)
        
        # Drag coefficient
        CD = (self.aero.CD0 + self.aero.K * CL**2 + 
              self.aero.CD_de * delta_e**2)
        
        # Side force
        CY = (self.aero.CY_beta * beta + 
              self.aero.CY_p * (p * self.aircraft.wing_span / (2 * V)) + 
              self.aero.CY_r * (r * self.aircraft.wing_span / (2 * V)) + 
              self.aero.CY_dr * delta_r)
        
        # Pitching moment
        Cm = (self.aero.Cm0 + self.aero.Cm_alpha * alpha + 
              self.aero.Cm_q * (q * self.aircraft.mean_aerodynamic_chord / (2 * V)) + 
              self.aero.Cm_de * delta_e)
        
        # Rolling moment
        Cl = (self.aero.Cl_beta * beta + 
              self.aero.Cl_p * (p * self.aircraft.wing_span / (2 * V)) + 
              self.aero.Cl_r * (r * self.aircraft.wing_span / (2 * V)) + 
              self.aero.Cl_da * delta_a + self.aero.Cl_dr * delta_r)
        
        # Yawing moment
        Cn = (self.aero.Cn_beta * beta + 
              self.aero.Cn_p * (p * self.aircraft.wing_span / (2 * V)) + 
              self.aero.Cn_r * (r * self.aircraft.wing_span / (2 * V)) + 
              self.aero.Cn_da * delta_a + self.aero.Cn_dr * delta_r)
        
        # Forces in body axes
        L_lift = Q * self.aircraft.wing_area * CL
        D_drag = Q * self.aircraft.wing_area * CD
        Y_side = Q * self.aircraft.wing_area * CY
        
        # Transform to body axes
        Fx_aero = -D_drag * np.cos(alpha) + L_lift * np.sin(alpha)
        Fy_aero = Y_side
        Fz_aero = -D_drag * np.sin(alpha) - L_lift * np.cos(alpha)
        
        # Thrust model
        thrust = self.aircraft.max_thrust * 2 * delta_t  # Two engines
        self.engine_thrust = thrust
        self.fuel_flow = self.aircraft.fuel_flow_max * 2 * delta_t
        
        # Thrust forces and moments (assuming engines at sides)
        Fx_prop = thrust
        Fy_prop = 0.0
        Fz_prop = 0.0
        Mx_prop = 0.0
        My_prop = 0.0
        Mz_prop = -thrust * self.aircraft.thrust_moment_arm * 0.1  # Small yaw moment
        
        # Total forces and moments
        Fx = Fx_aero + Fx_prop
        Fy = Fy_aero + Fy_prop
        Fz = Fz_aero + Fz_prop
        
        Mx = Q * self.aircraft.wing_area * self.aircraft.wing_span * Cl + Mx_prop
        My = Q * self.aircraft.wing_area * self.aircraft.mean_aerodynamic_chord * Cm + My_prop
        Mz = Q * self.aircraft.wing_area * self.aircraft.wing_span * Cn + Mz_prop
        
        return np.array([Fx, Fy, Fz]), np.array([Mx, My, Mz])
    
    def aircraft_dynamics(self, t: float, state: np.ndarray) -> np.ndarray:
        """Aircraft equations of motion"""
        V, alpha, beta, p, q, r, phi, theta, psi, pos_n, pos_e, pos_d = state
        
        # Get controls
        controls = np.array([self.delta_e, self.delta_a, self.delta_r, self.delta_t])
        
        # Compute forces and moments
        F, M = self.compute_forces_and_moments(state, controls)
        Fx, Fy, Fz = F
        Mx, My, Mz = M
        
        # Mass and inertia properties
        m = self.aircraft.mass - self.fuel_used
        Ixx, Iyy, Izz, Ixz = self.aircraft.Ixx, self.aircraft.Iyy, self.aircraft.Izz, self.aircraft.Ixz
        
        # Kinematic equations - body velocities
        u = V * np.cos(alpha) * np.cos(beta)
        v = V * np.sin(beta)
        w = V * np.sin(alpha) * np.cos(beta)
        
        # Force equations
        udot = (Fx / m) - (q * w) + (r * v)
        vdot = (Fy / m) - (r * u) + (p * w)
        wdot = (Fz / m) - (p * v) + (q * u)
        
        # Moment equations
        pdot = (Izz * Mx + Ixz * Mz - (Ixz * (Iyy - Ixx - Izz) * p * q) + 
                (Ixz**2 + Izz * (Izz - Iyy)) * q * r) / (Ixx * Izz - Ixz**2)
        qdot = (My - (Ixx - Izz) * p * r - Ixz * (p**2 - r**2)) / Iyy
        rdot = (Ixz * Mx + Ixx * Mz + (Ixz * (Iyy - Ixx - Izz) * q * r) + 
                (Ixz**2 + Ixx * (Ixx - Iyy)) * p * q) / (Ixx * Izz - Ixz**2)
        
        # Navigation equations - rotation from body to NED
        R_bi = np.array([
            [np.cos(theta)*np.cos(psi), np.sin(phi)*np.sin(theta)*np.cos(psi)-np.cos(phi)*np.sin(psi), np.cos(phi)*np.sin(theta)*np.cos(psi)+np.sin(phi)*np.sin(psi)],
            [np.cos(theta)*np.sin(psi), np.sin(phi)*np.sin(theta)*np.sin(psi)+np.cos(phi)*np.cos(psi), np.cos(phi)*np.sin(theta)*np.sin(psi)-np.sin(phi)*np.cos(psi)],
            [-np.sin(theta), np.sin(phi)*np.cos(theta), np.cos(phi)*np.cos(theta)]
        ])
        
        # Add wind effect
        wind = self.wind.get_wind(np.array([pos_n, pos_e, pos_d]), t)
        vel_body = np.array([u, v, w])
        vel_inertial = R_bi @ vel_body
        vel_inertial_with_wind = vel_inertial + wind
        
        # Convert back to body frame for velocity derivatives
        vel_body_with_wind = R_bi.T @ vel_inertial_with_wind
        u_w, v_w, w_w = vel_body_with_wind
        
        # Update true airspeed and angles with wind
        V_w = np.sqrt(u_w**2 + v_w**2 + w_w**2)
        alpha_w = np.arctan2(w_w, u_w) if u_w != 0 else 0.0
        beta_w = np.arcsin(v_w / V_w) if V_w != 0 else 0.0
        
        # Use wind-relative values for aerodynamic calculations but inertial for navigation
        Vdot = (u * udot + v * vdot + w * wdot) / V if V != 0 else 0.0
        alphadot = (u * wdot - w * udot) / (u**2 + w**2) if (u**2 + w**2) != 0 else 0.0
        betadot = (V * vdot - v * Vdot) / (V**2 * np.cos(beta)) if (V != 0 and np.cos(beta) != 0) else 0.0
        
        # Euler angle derivatives
        phidot = p + (q * np.sin(phi) + r * np.cos(phi)) * np.tan(theta)
        thetadot = q * np.cos(phi) - r * np.sin(phi)
        psidot = (q * np.sin(phi) + r * np.cos(phi)) / np.cos(theta) if np.cos(theta) != 0 else 0.0
        
        # Position derivatives (inertial frame without wind for navigation)
        posdot_n, posdot_e, posdot_d = vel_inertial
        
        # Update fuel
        self.fuel_used += self.fuel_flow * self.dt
        
        return [Vdot, alphadot, betadot, pdot, qdot, rdot, 
                phidot, thetadot, psidot, posdot_n, posdot_e, posdot_d]
    
    def update(self, frame):
        """Main simulation update loop"""
        # Update autopilot if in autopilot mode
        if self.flight_mode == "AUTOPILOT":
            setpoints = {
                'altitude': self.altitude_setpoint,
                'heading': self.heading_setpoint,
                'airspeed': self.airspeed_setpoint
            }
            controls = self.autopilot.update(self.dt, self.get_state_dict(), setpoints)
            self.delta_e = controls['elevator']
            self.delta_a = controls['aileron']
            self.delta_r = controls['rudder']
            self.delta_t = controls['throttle']
        
        # Integrate dynamics
        current_state = self.get_state_vector()
        sol = solve_ivp(self.aircraft_dynamics, [0, self.dt], current_state, 
                       method='RK45', t_eval=[self.dt], rtol=1e-6, atol=1e-8)
        
        # Update state
        self.set_state_vector(sol.y[:, -1])
        
        # Store data
        self.time += self.dt
        self.time_history.append(self.time)
        self.state_history.append(self.get_state_vector())
        self.control_history.append([self.delta_e, self.delta_a, self.delta_r, self.delta_t])
        
        # Calculate performance metrics
        altitude = -self.position_d
        mach_number = self.V / 340.0  # Simple speed of sound
        load_factor = np.sqrt(1 + (self.V * self.q / self.atmosphere.g)**2)  # Simplified
        
        self.performance_history.append({
            'altitude': altitude,
            'mach': mach_number,
            'load_factor': load_factor,
            'fuel_flow': self.fuel_flow,
            'thrust': self.engine_thrust
        })
        
        # Update visualization
        self.update_plots()
        
        if self.time >= self.max_time:
            self.animation.event_source.stop()
        
        return self.artists

    def setup_plots(self):
        """Setup professional-grade visualization"""
        self.fig = plt.figure(figsize=(18, 12))
        self.fig.suptitle('Professional Flight Control Simulation - Boeing 737-800', 
                         fontsize=16, fontweight='bold')
        
        # Create main grid
        gs = gridspec.GridSpec(3, 4, figure=self.fig, height_ratios=[2, 1, 1])
        
        # 3D trajectory plot
        self.ax_3d = self.fig.add_subplot(gs[0:2, 0:2], projection='3d')
        self.ax_3d.set_xlabel('East Position (m)')
        self.ax_3d.set_ylabel('North Position (m)')
        self.ax_3d.set_zlabel('Altitude (m)')
        self.ax_3d.set_title('3D Flight Trajectory')
        self.ax_3d.grid(True, alpha=0.3)
        
        # Primary flight display (PFD) style attitude
        self.ax_pfd = self.fig.add_subplot(gs[0, 2])
        self.ax_pfd.set_title('Attitude Director Indicator', fontweight='bold')
        self.ax_pfd.set_aspect('equal')
        self.ax_pfd.set_xlim(-1.5, 1.5)
        self.ax_pfd.set_ylim(-1.5, 1.5)
        self.ax_pfd.axis('off')
        
        # Navigation display
        self.ax_nav = self.fig.add_subplot(gs[0, 3])
        self.ax_nav.set_title('Navigation Display', fontweight='bold')
        self.ax_nav.set_aspect('equal')
        self.ax_nav.grid(True, alpha=0.3)
        
        # Systems status
        self.ax_systems = self.fig.add_subplot(gs[1, 2:])
        self.ax_systems.set_title('Systems Status', fontweight='bold')
        self.ax_systems.axis('off')
        
        # Flight parameters time history
        self.ax_params = self.fig.add_subplot(gs[2, :])
        self.ax_params.set_title('Flight Parameters History', fontweight='bold')
        self.ax_params.set_xlabel('Time (s)')
        self.ax_params.grid(True, alpha=0.3)
        
        # Initialize plot elements
        self.setup_pfd()
        self.setup_nav_display()
        self.setup_systems_display()
        self.setup_parameter_plots()
        
        # Add controls
        self.setup_controls()
        
        plt.tight_layout()
    
    def setup_pfd(self):
        """Setup Primary Flight Display"""
        # Attitude indicator background
        self.horizon_line, = self.ax_pfd.plot([-1, 1], [0, 0], 'b-', linewidth=2)
        self.sky_patch = plt.Rectangle((-1.5, 0), 3, 1.5, facecolor='lightblue', alpha=0.3)
        self.ground_patch = plt.Rectangle((-1.5, -1.5), 3, 1.5, facecolor='sandybrown', alpha=0.3)
        self.ax_pfd.add_patch(self.sky_patch)
        self.ax_pfd.add_patch(self.ground_patch)
        
        # Aircraft symbol
        self.aircraft_symbol, = self.ax_pfd.plot([0], [0], 'ko-', markersize=8, linewidth=2)
        
        # Pitch ladder
        self.pitch_ladders = []
        for pitch in [-30, -20, -10, 10, 20, 30]:
            ladder = self.ax_pfd.plot([-0.3, 0.3], [0, 0], 'w-', alpha=0.5)[0]
            self.pitch_ladders.append(ladder)
        
        # Bank angle indicator
        self.bank_indicator = self.ax_pfd.plot([-0.1, 0, 0.1], [1.3, 1.4, 1.3], 'k-', linewidth=2)[0]
    
    def setup_nav_display(self):
        """Setup Navigation Display"""
        # Course line
        self.course_line, = self.ax_nav.plot([0, 0.5], [0, 0], 'r-', linewidth=2)
        
        # Aircraft position
        self.nav_aircraft = self.ax_nav.plot(0, 0, 'bo', markersize=10)[0]
        
        # Waypoints
        self.waypoints = self.ax_nav.plot([10000, 20000], [5000, 15000], 'g^', markersize=8)[0]
        
        self.ax_nav.set_xlim(-5000, 25000)
        self.ax_nav.set_ylim(-5000, 25000)
    
    def setup_systems_display(self):
        """Setup Systems Status Display"""
        # Engine parameters
        self.engine_text = self.ax_systems.text(0.1, 0.9, 'ENGINES:\nN1: --%\nFF: -- kg/h\nEGT: --°C', 
                                               transform=self.ax_systems.transAxes, fontfamily='monospace')
        
        # Flight controls
        self.controls_text = self.ax_systems.text(0.4, 0.9, 'CONTROLS:\nElev: --°\nAiln: --°\nRudd: --°', 
                                                 transform=self.ax_systems.transAxes, fontfamily='monospace')
        
        # Performance
        self.perf_text = self.ax_systems.text(0.7, 0.9, 'PERFORMANCE:\nMach: --\nG-Load: --\nVs: -- m/s', 
                                             transform=self.ax_systems.transAxes, fontfamily='monospace')
    
    def setup_parameter_plots(self):
        """Setup flight parameter time history plots"""
        self.altitude_line, = self.ax_params.plot([], [], 'r-', label='Altitude (m)')
        self.airspeed_line, = self.ax_params.plot([], [], 'g-', label='Airspeed (m/s)')
        self.vertical_speed_line, = self.ax_params.plot([], [], 'b-', label='Vert Speed (m/s)')
        self.ax_params.legend(loc='upper right')
    
    def setup_controls(self):
        """Setup control panel"""
        # Control area
        control_ax = self.fig.add_axes([0.02, 0.02, 0.96, 0.08])
        control_ax.set_facecolor('lightgray')
        control_ax.axis('off')
        
        # Flight mode selector
        mode_ax = self.fig.add_axes([0.05, 0.04, 0.1, 0.04])
        self.mode_selector = RadioButtons(mode_ax, ['AUTOPILOT', 'MANUAL'], active=0)
        self.mode_selector.on_clicked(self.change_flight_mode)
        
        # Autopilot controls
        slider_width = 0.2
        slider_height = 0.02
        slider_y = 0.04
        
        # Altitude slider
        slider_ax_alt = self.fig.add_axes([0.2, slider_y, slider_width, slider_height])
        self.alt_slider = Slider(slider_ax_alt, 'Altitude (m)', 8000, 12000, 
                                valinit=self.altitude_setpoint, valstep=100)
        
        # Heading slider
        slider_ax_hdg = self.fig.add_axes([0.45, slider_y, slider_width, slider_height])
        self.hdg_slider = Slider(slider_ax_hdg, 'Heading (°)', -180, 180, 
                                valinit=np.degrees(self.heading_setpoint), valstep=5)
        
        # Speed slider
        slider_ax_spd = self.fig.add_axes([0.7, slider_y, slider_width, slider_height])
        self.spd_slider = Slider(slider_ax_spd, 'Airspeed (m/s)', 200, 300, 
                                valinit=self.airspeed_setpoint, valstep=5)
        
        # Reset button
        reset_ax = self.fig.add_axes([0.92, slider_y, 0.06, slider_height])
        self.reset_button = Button(reset_ax, 'RESET', color='red', hovercolor='darkred')
        
        # Connect callbacks
        self.alt_slider.on_changed(self.update_altitude)
        self.hdg_slider.on_changed(self.update_heading)
        self.spd_slider.on_changed(self.update_airspeed)
        self.reset_button.on_clicked(self.reset_simulation)
    
    def change_flight_mode(self, label):
        """Change between autopilot and manual flight"""
        self.flight_mode = label
        print(f"Flight mode changed to: {label}")
    
    def update_altitude(self, val):
        self.altitude_setpoint = val
    
    def update_heading(self, val):
        self.heading_setpoint = np.radians(val)
    
    def update_airspeed(self, val):
        self.airspeed_setpoint = val
    
    def reset_simulation(self, event):
        """Reset simulation to initial conditions"""
        self.initialize_state()
        self.time = 0.0
        self.time_history = []
        self.state_history = []
        self.control_history = []
        self.performance_history = []
        self.fuel_used = 0.0
        
        # Reset autopilot integrals
        self.autopilot = Autopilot()
        
        # Reset sliders
        self.alt_slider.set_val(10000)
        self.hdg_slider.set_val(0)
        self.spd_slider.set_val(250)
        
        print("Simulation reset")
    
    def update_plots(self):
        """Update all visualization elements"""
        if len(self.time_history) == 0:
            return
        
        # Convert to arrays for easier handling
        state_array = np.array(self.state_history)
        control_array = np.array(self.control_history)
        
        # Update 3D trajectory
        x = state_array[:, 10]  # East
        y = state_array[:, 9]   # North  
        z = -state_array[:, 11] # Altitude (convert from down to up)
        
        self.ax_3d.cla()
        self.ax_3d.plot(x, y, z, 'b-', linewidth=2, alpha=0.7)
        self.ax_3d.plot([x[-1]], [y[-1]], [z[-1]], 'ro', markersize=8)
        self.ax_3d.set_xlabel('East (m)')
        self.ax_3d.set_ylabel('North (m)')
        self.ax_3d.set_zlabel('Altitude (m)')
        self.ax_3d.set_title('3D Flight Trajectory')
        
        # Update PFD
        self.update_pfd()
        
        # Update navigation display
        self.update_nav_display(x, y)
        
        # Update systems display
        self.update_systems_display()
        
        # Update parameter plots
        self.update_parameter_plots()
        
        # Dynamic axis limits
        self.dynamic_axis_limits()
    
    def update_pfd(self):
        """Update Primary Flight Display"""
        # Rotate horizon based on roll and pitch
        roll = self.phi
        pitch = self.theta
        
        # Transform pitch ladder
        pitch_scale = 0.02  # degrees per pixel
        for i, ladder in enumerate(self.pitch_ladders):
            pitch_angle = [-30, -20, -10, 10, 20, 30][i]
            y_pos = -pitch_angle * pitch_scale - pitch * 0.1
            
            # Rotate for bank angle
            x_rot = np.array([-0.3, 0.3]) * np.cos(roll)
            y_rot = y_pos + np.array([-0.3, 0.3]) * np.sin(roll)
            
            ladder.set_data(x_rot, y_rot)
        
        # Update bank indicator
        bank_x = np.array([-0.1, 0, 0.1]) * np.cos(roll) - 1.3 * np.sin(roll)
        bank_y = np.array([-0.1, 0, 0.1]) * np.sin(roll) + 1.3 * np.cos(roll)
        self.bank_indicator.set_data(bank_x, bank_y)
    
    def update_nav_display(self, x, y):
        """Update Navigation Display"""
        self.nav_aircraft.set_data([x[-1]], [y[-1]])
        
        # Center display on aircraft
        margin = 5000
        self.ax_nav.set_xlim(x[-1] - margin, x[-1] + margin)
        self.ax_nav.set_ylim(y[-1] - margin, y[-1] + margin)
    
    def update_systems_display(self):
        """Update Systems Status Display"""
        # Engine parameters
        n1 = self.delta_t * 100  # Simplified N1 calculation
        ff = self.fuel_flow * 3600  # kg/h
        egt = 400 + n1 * 2  # Simplified EGT
        
        engine_text = f'ENGINES:\nN1: {n1:.1f}%\nFF: {ff:.1f} kg/h\nEGT: {egt:.0f}°C'
        self.engine_text.set_text(engine_text)
        
        # Control positions
        elev_deg = np.degrees(self.delta_e)
        ail_deg = np.degrees(self.delta_a)
        rudd_deg = np.degrees(self.delta_r)
        
        controls_text = f'CONTROLS:\nElev: {elev_deg:+.1f}°\nAiln: {ail_deg:+.1f}°\nRudd: {rudd_deg:+.1f}°'
        self.controls_text.set_text(controls_text)
        
        # Performance
        mach = self.V / 340.0
        load_factor = 1.0  # Simplified
        vs = -self.position_d / self.dt if len(self.state_history) > 1 else 0
        
        perf_text = f'PERFORMANCE:\nMach: {mach:.2f}\nG-Load: {load_factor:.1f}\nVs: {vs:+.1f} m/s'
        self.perf_text.set_text(perf_text)
    
    def update_parameter_plots(self):
        """Update flight parameter history plots"""
        if len(self.time_history) < 2:
            return
            
        state_array = np.array(self.state_history)
        
        # Altitude
        altitude = -state_array[:, 11]
        self.altitude_line.set_data(self.time_history, altitude)
        
        # Airspeed
        airspeed = state_array[:, 0]
        self.airspeed_line.set_data(self.time_history, airspeed)
        
        # Vertical speed (simplified)
        if len(altitude) > 1:
            vs = np.gradient(altitude, self.time_history)
            self.vertical_speed_line.set_data(self.time_history[:-1], vs[:-1])
        
        self.ax_params.relim()
        self.ax_params.autoscale_view()
    
    def dynamic_axis_limits(self):
        """Update axis limits dynamically"""
        # 3D plot limits
        if len(self.state_history) > 1:
            state_array = np.array(self.state_history)
            x, y, z = state_array[:, 10], state_array[:, 9], -state_array[:, 11]
            
            # Add some margin
            margin = max(1000, np.std(x) * 3, np.std(y) * 3, np.std(z) * 3)
            
            self.ax_3d.set_xlim(np.mean(x) - margin, np.mean(x) + margin)
            self.ax_3d.set_ylim(np.mean(y) - margin, np.mean(y) + margin)
            self.ax_3d.set_zlim(np.mean(z) - margin, np.mean(z) + margin)
    
    def run_simulation(self):
        """Start the simulation"""
        self.animation = FuncAnimation(self.fig, self.update, interval=50, 
                                     blit=False, cache_frame_data=False)
        plt.show()

# Run the simulation
if __name__ == "__main__":
    print("Starting Professional Flight Control Simulation...")
    print("Aircraft: Boeing 737-800")
    print("Features:")
    print("- Realistic 6-DOF flight dynamics")
    print("- International Standard Atmosphere model")
    print("- Professional autopilot with PID controllers")
    print("- Wind and turbulence model")
    print("- Advanced visualization with PFD and systems displays")
    print("- Real-time performance monitoring")
    
    sim = FlightControlSimulation()
    sim.run_simulation()
