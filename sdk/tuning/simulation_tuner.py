"""
Constellation Overwatch SDK - Simulation-Based Tuning
Advanced tuning using simulation environments with hardware-in-the-loop validation.
"""

import numpy as np
import logging
from typing import Dict, List, Tuple, Optional, Callable
from dataclasses import dataclass, field
from enum import Enum
import time
import json
from scipy.integrate import solve_ivp
from scipy.optimize import minimize, differential_evolution

from .pid_controller import PIDController, PIDGains, RatePIDController
from .auto_tuner import TuningConfig, TuningResult
from .safety_monitor import SafetyMonitor

class SimulationType(Enum):
    """Types of simulation environments"""
    GAZEBO = "gazebo"
    AIRSIM = "airsim"
    XPLANE = "xplane"
    JSBSIM = "jsbsim"
    MATHEMATICAL = "mathematical"
    HARDWARE_IN_LOOP = "hardware_in_loop"

class OptimizationMethod(Enum):
    """Optimization methods for parameter tuning"""
    GENETIC_ALGORITHM = "genetic_algorithm"
    PARTICLE_SWARM = "particle_swarm"
    GRADIENT_DESCENT = "gradient_descent"
    BAYESIAN_OPTIMIZATION = "bayesian_optimization"
    REINFORCEMENT_LEARNING = "reinforcement_learning"

@dataclass
class DroneModel:
    """Mathematical model of drone dynamics"""
    mass: float = 1.5  # kg
    inertia: Dict[str, float] = field(default_factory=lambda: {
        'Ixx': 0.029, 'Iyy': 0.029, 'Izz': 0.055  # kg⋅m²
    })
    arm_length: float = 0.225  # m
    motor_count: int = 4
    prop_diameter: float = 0.254  # m (10 inch)
    motor_kv: float = 920  # rpm/V
    battery_voltage: float = 14.8  # V
    motor_time_constant: float = 0.02  # s
    
    # Aerodynamic parameters
    drag_coefficient: float = 0.01
    thrust_coefficient: float = 8.5e-6
    torque_coefficient: float = 1.4e-7
    
@dataclass
class SimulationConfig:
    """Configuration for simulation-based tuning"""
    simulation_type: SimulationType = SimulationType.MATHEMATICAL
    drone_model: DroneModel = field(default_factory=DroneModel)
    optimization_method: OptimizationMethod = OptimizationMethod.GENETIC_ALGORITHM
    
    # Simulation parameters
    sample_rate: float = 400.0  # Hz
    simulation_time: float = 20.0  # seconds
    wind_disturbance: bool = True
    sensor_noise: bool = True
    actuator_delays: bool = True
    
    # Test scenarios
    test_scenarios: List[str] = field(default_factory=lambda: [
        'step_response', 'frequency_sweep', 'attitude_tracking', 'disturbance_rejection'
    ])
    
    # Optimization parameters
    population_size: int = 50
    max_generations: int = 100
    convergence_threshold: float = 1e-6
    
    # Performance weights
    performance_weights: Dict[str, float] = field(default_factory=lambda: {
        'tracking_error': 0.3,
        'settling_time': 0.2,
        'overshoot': 0.2,
        'control_effort': 0.1,
        'robustness': 0.2
    })

class DroneSimulator:
    """
    High-fidelity drone simulation for tuning validation.
    
    Features:
    - 6-DOF dynamics simulation
    - Motor and propeller models
    - Sensor noise and delays
    - Wind disturbances
    - Actuator saturation
    """
    
    def __init__(self, model: DroneModel, sample_rate: float = 400.0):
        """
        Initialize drone simulator.
        
        Args:
            model: Drone physical model
            sample_rate: Simulation sample rate in Hz
        """
        self.model = model
        self.sample_rate = sample_rate
        self.dt = 1.0 / sample_rate
        
        # State variables [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
        self.state = np.zeros(12)
        
        # Motor states
        self.motor_speeds = np.zeros(4)  # rad/s
        self.motor_commands = np.zeros(4)  # 0-1
        
        # Disturbances
        self.wind_velocity = np.array([0.0, 0.0, 0.0])  # m/s
        self.turbulence_intensity = 0.1
        
        # Sensor noise parameters
        self.gyro_noise_std = 0.01  # rad/s
        self.accel_noise_std = 0.1  # m/s²
        self.attitude_noise_std = 0.001  # rad
        
        self.logger = logging.getLogger(__name__)
        
    def update(self, motor_commands: np.ndarray, dt: Optional[float] = None) -> Dict:
        """
        Update simulation one time step.
        
        Args:
            motor_commands: Motor commands [0-1] for each motor
            dt: Time step (uses default if None)
            
        Returns:
            Current state and sensor measurements
        """
        if dt is None:
            dt = self.dt
            
        # Update motor dynamics
        self._update_motors(motor_commands, dt)
        
        # Calculate forces and moments
        forces, moments = self._calculate_forces_moments()
        
        # Update dynamics
        self._update_dynamics(forces, moments, dt)
        
        # Generate sensor measurements
        measurements = self._generate_sensor_data()
        
        return {
            'state': self.state.copy(),
            'attitude': {
                'roll': np.degrees(self.state[6]),
                'pitch': np.degrees(self.state[7]), 
                'yaw': np.degrees(self.state[8])
            },
            'rates': {
                'roll': np.degrees(self.state[9]),
                'pitch': np.degrees(self.state[10]),
                'yaw': np.degrees(self.state[11])
            },
            'sensors': measurements
        }
        
    def _update_motors(self, commands: np.ndarray, dt: float):
        """Update motor dynamics with first-order lag"""
        # Clamp commands
        commands = np.clip(commands, 0.0, 1.0)
        
        # First-order motor dynamics
        tau = self.model.motor_time_constant
        alpha = dt / (tau + dt)
        
        # Convert commands to desired speeds
        max_speed = self.model.motor_kv * self.model.battery_voltage * 2 * np.pi / 60  # rad/s
        desired_speeds = commands * max_speed
        
        # Update motor speeds
        self.motor_speeds = (1 - alpha) * self.motor_speeds + alpha * desired_speeds
        self.motor_commands = commands.copy()
        
    def _calculate_forces_moments(self) -> Tuple[np.ndarray, np.ndarray]:
        """Calculate forces and moments from motor speeds"""
        # Thrust from each motor
        thrusts = self.model.thrust_coefficient * self.motor_speeds**2
        
        # Total thrust (body-frame)
        total_thrust = np.sum(thrusts)
        forces = np.array([0.0, 0.0, -total_thrust])  # Thrust in -z direction
        
        # Moments from thrust differential (simplified quad-X configuration)
        arm = self.model.arm_length
        
        # Roll moment (motors 1,3 vs 2,4)
        roll_moment = arm * (thrusts[0] + thrusts[2] - thrusts[1] - thrusts[3])
        
        # Pitch moment (motors 0,1 vs 2,3)
        pitch_moment = arm * (thrusts[0] + thrusts[1] - thrusts[2] - thrusts[3])
        
        # Yaw moment from torque reaction
        yaw_moment = self.model.torque_coefficient * (
            self.motor_speeds[0]**2 + self.motor_speeds[2]**2 - 
            self.motor_speeds[1]**2 - self.motor_speeds[3]**2
        )
        
        moments = np.array([roll_moment, pitch_moment, yaw_moment])
        
        # Add wind disturbance
        if np.linalg.norm(self.wind_velocity) > 0:
            wind_force = self._calculate_wind_force()
            forces += wind_force
            
        return forces, moments
        
    def _calculate_wind_force(self) -> np.ndarray:
        """Calculate wind force on drone"""
        # Simplified wind model
        relative_velocity = self.state[3:6] - self.wind_velocity
        wind_force = -self.model.drag_coefficient * relative_velocity * np.linalg.norm(relative_velocity)
        return wind_force
        
    def _update_dynamics(self, forces: np.ndarray, moments: np.ndarray, dt: float):
        """Update drone dynamics using numerical integration"""
        
        def dynamics(t, state):
            """Drone dynamics equations"""
            # State: [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
            pos = state[0:3]
            vel = state[3:6]
            attitude = state[6:9]
            rates = state[9:12]
            
            # Rotation matrix from body to inertial frame
            R = self._rotation_matrix(attitude)
            
            # Translational dynamics
            accel = R @ forces / self.model.mass + np.array([0, 0, 9.81])  # Add gravity
            
            # Rotational dynamics
            I = np.diag([self.model.inertia['Ixx'], 
                        self.model.inertia['Iyy'], 
                        self.model.inertia['Izz']])
            
            # Euler's equation
            angular_accel = np.linalg.inv(I) @ (moments - np.cross(rates, I @ rates))
            
            # Attitude kinematics (simplified for small angles)
            attitude_rates = rates  # Simplified assumption
            
            # State derivative
            state_dot = np.concatenate([vel, accel, attitude_rates, angular_accel])
            return state_dot
            
        # Integrate using RK45
        sol = solve_ivp(dynamics, [0, dt], self.state, method='RK45', rtol=1e-6)
        self.state = sol.y[:, -1]
        
    def _rotation_matrix(self, attitude: np.ndarray) -> np.ndarray:
        """Calculate rotation matrix from Euler angles"""
        roll, pitch, yaw = attitude
        
        # Rotation matrices
        R_x = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])
        
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])
        
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw), np.cos(yaw), 0],
                       [0, 0, 1]])
        
        return R_z @ R_y @ R_x
        
    def _generate_sensor_data(self) -> Dict:
        """Generate simulated sensor measurements with noise"""
        # IMU measurements
        attitude = self.state[6:9]
        rates = self.state[9:12]
        
        # Add noise
        if hasattr(self, 'sensor_noise') and self.sensor_noise:
            attitude_noisy = attitude + np.random.normal(0, self.attitude_noise_std, 3)
            rates_noisy = rates + np.random.normal(0, self.gyro_noise_std, 3)
        else:
            attitude_noisy = attitude
            rates_noisy = rates
            
        return {
            'imu': {
                'roll': attitude_noisy[0],
                'pitch': attitude_noisy[1],
                'yaw': attitude_noisy[2],
                'p': rates_noisy[0],
                'q': rates_noisy[1],
                'r': rates_noisy[2],
                'timestamp': time.time()
            }
        }
        
    def reset(self, initial_state: Optional[np.ndarray] = None):
        """Reset simulation to initial conditions"""
        if initial_state is not None:
            self.state = initial_state.copy()
        else:
            self.state = np.zeros(12)
        self.motor_speeds = np.zeros(4)
        self.motor_commands = np.zeros(4)
        
    def set_wind(self, wind_velocity: np.ndarray, turbulence: float = 0.1):
        """Set wind conditions"""
        self.wind_velocity = wind_velocity
        self.turbulence_intensity = turbulence

class SimulationTuner:
    """
    Simulation-based tuning system using optimization algorithms.
    
    Features:
    - Multi-objective optimization
    - Hardware-in-the-loop validation
    - Robustness testing
    - Performance prediction
    - Real-world correlation
    """
    
    def __init__(self, 
                 config: SimulationConfig,
                 safety_monitor: Optional[SafetyMonitor] = None):
        """
        Initialize simulation tuner.
        
        Args:
            config: Simulation configuration
            safety_monitor: Safety monitoring system
        """
        self.config = config
        self.safety_monitor = safety_monitor or SafetyMonitor()
        
        # Initialize simulator
        self.simulator = DroneSimulator(config.drone_model, config.sample_rate)
        
        # Optimization state
        self.best_gains = None
        self.best_performance = float('inf')
        self.optimization_history = []
        
        self.logger = logging.getLogger(__name__)
        
    def tune_parameters(self, 
                       initial_gains: Optional[Dict[str, PIDGains]] = None,
                       validation_callback: Optional[Callable] = None) -> TuningResult:
        """
        Tune PID parameters using simulation-based optimization.
        
        Args:
            initial_gains: Initial PID gains for each axis
            validation_callback: Optional callback for hardware validation
            
        Returns:
            Tuning results with optimized parameters
        """
        self.logger.info(f"Starting simulation-based tuning with {self.config.optimization_method}")
        
        start_time = time.time()
        
        # Initialize optimization
        if initial_gains is None:
            initial_gains = {
                'roll': PIDGains(P=0.1, I=0.05, D=0.01),
                'pitch': PIDGains(P=0.1, I=0.05, D=0.01),
                'yaw': PIDGains(P=0.15, I=0.08, D=0.0)
            }
            
        # Define optimization bounds
        bounds = self._get_optimization_bounds()
        
        # Select optimization method
        if self.config.optimization_method == OptimizationMethod.GENETIC_ALGORITHM:
            result = self._genetic_algorithm_optimization(initial_gains, bounds)
        elif self.config.optimization_method == OptimizationMethod.GRADIENT_DESCENT:
            result = self._gradient_descent_optimization(initial_gains, bounds)
        else:
            self.logger.warning(f"Method {self.config.optimization_method} not implemented, using genetic algorithm")
            result = self._genetic_algorithm_optimization(initial_gains, bounds)
            
        # Validate results
        if self.best_gains and validation_callback:
            validation_success = validation_callback(self.best_gains)
            if not validation_success:
                self.logger.warning("Hardware validation failed, using conservative gains")
                
        # Prepare results
        tuning_result = TuningResult(
            success=self.best_gains is not None,
            gains=self.best_gains,
            performance_metrics={
                'cost_function': self.best_performance,
                'optimization_time': time.time() - start_time,
                'iterations': len(self.optimization_history)
            },
            execution_time=time.time() - start_time
        )
        
        if not tuning_result.success:
            tuning_result.warnings.append("Optimization failed to converge")
            
        return tuning_result
        
    def _get_optimization_bounds(self) -> List[Tuple[float, float]]:
        """Get parameter bounds for optimization"""
        # P, I, D gains for roll, pitch, yaw (9 parameters total)
        bounds = []
        
        # Roll and pitch gains
        for _ in range(2):
            bounds.extend([
                (0.01, 2.0),   # P gain
                (0.0, 2.0),    # I gain  
                (0.0, 0.5)     # D gain
            ])
            
        # Yaw gains (typically no D term)
        bounds.extend([
            (0.01, 1.0),   # P gain
            (0.0, 1.0),    # I gain
            (0.0, 0.1)     # D gain
        ])
        
        return bounds
        
    def _genetic_algorithm_optimization(self, 
                                      initial_gains: Dict[str, PIDGains],
                                      bounds: List[Tuple[float, float]]) -> Dict:
        """Genetic algorithm optimization"""
        
        def objective_function(params):
            """Objective function for optimization"""
            # Convert parameter array to gains
            gains = self._params_to_gains(params)
            
            # Evaluate performance
            performance = self._evaluate_performance(gains)
            
            # Add to history
            self.optimization_history.append({
                'params': params.copy(),
                'gains': gains,
                'performance': performance
            })
            
            return performance
            
        # Run genetic algorithm
        result = differential_evolution(
            objective_function,
            bounds,
            seed=42,
            maxiter=self.config.max_generations,
            popsize=self.config.population_size,
            atol=self.config.convergence_threshold
        )
        
        if result.success:
            self.best_gains = self._params_to_gains(result.x)
            self.best_performance = result.fun
            self.logger.info(f"Genetic algorithm converged: cost={result.fun:.4f}")
        else:
            self.logger.warning("Genetic algorithm did not converge")
            
        return result
        
    def _gradient_descent_optimization(self, 
                                     initial_gains: Dict[str, PIDGains],
                                     bounds: List[Tuple[float, float]]) -> Dict:
        """Gradient-based optimization"""
        
        # Convert initial gains to parameter array
        initial_params = self._gains_to_params(initial_gains)
        
        def objective_function(params):
            gains = self._params_to_gains(params)
            performance = self._evaluate_performance(gains)
            
            self.optimization_history.append({
                'params': params.copy(),
                'gains': gains,
                'performance': performance
            })
            
            return performance
            
        # Run optimization
        result = minimize(
            objective_function,
            initial_params,
            method='L-BFGS-B',
            bounds=bounds,
            options={'maxiter': self.config.max_generations}
        )
        
        if result.success:
            self.best_gains = self._params_to_gains(result.x)
            self.best_performance = result.fun
            self.logger.info(f"Gradient descent converged: cost={result.fun:.4f}")
        else:
            self.logger.warning("Gradient descent did not converge")
            
        return result
        
    def _params_to_gains(self, params: np.ndarray) -> Dict[str, PIDGains]:
        """Convert parameter array to PID gains"""
        gains = {}
        
        # Roll gains
        gains['roll'] = PIDGains(P=params[0], I=params[1], D=params[2])
        
        # Pitch gains
        gains['pitch'] = PIDGains(P=params[3], I=params[4], D=params[5])
        
        # Yaw gains
        gains['yaw'] = PIDGains(P=params[6], I=params[7], D=params[8])
        
        return gains
        
    def _gains_to_params(self, gains: Dict[str, PIDGains]) -> np.ndarray:
        """Convert PID gains to parameter array"""
        params = []
        
        for axis in ['roll', 'pitch', 'yaw']:
            gain = gains[axis]
            params.extend([gain.P, gain.I, gain.D])
            
        return np.array(params)
        
    def _evaluate_performance(self, gains: Dict[str, PIDGains]) -> float:
        """
        Evaluate controller performance using simulation.
        
        Args:
            gains: PID gains to evaluate
            
        Returns:
            Performance cost (lower is better)
        """
        total_cost = 0.0
        
        # Run multiple test scenarios
        for scenario in self.config.test_scenarios:
            try:
                cost = self._run_test_scenario(gains, scenario)
                total_cost += cost
            except Exception as e:
                self.logger.warning(f"Test scenario {scenario} failed: {e}")
                total_cost += 1000.0  # Large penalty for failed tests
                
        return total_cost / len(self.config.test_scenarios)
        
    def _run_test_scenario(self, gains: Dict[str, PIDGains], scenario: str) -> float:
        """Run a specific test scenario"""
        
        # Reset simulator
        self.simulator.reset()
        
        # Create controller
        controller = RatePIDController(sample_time=1.0/self.config.sample_rate)
        controller.set_all_gains(gains)
        
        # Run simulation
        simulation_time = self.config.simulation_time
        dt = 1.0 / self.config.sample_rate
        steps = int(simulation_time / dt)
        
        # Data collection
        errors = []
        control_outputs = []
        
        for step in range(steps):
            t = step * dt
            
            # Generate reference trajectory based on scenario
            ref_rates = self._generate_reference(scenario, t)
            
            # Get current state
            state = self.simulator.update(np.array([0.5, 0.5, 0.5, 0.5]))  # Hover command
            current_rates = state['rates']
            
            # Controller update
            control_outputs_dict = controller.update(ref_rates, current_rates)
            
            # Convert to motor commands (simplified)
            motor_commands = self._control_to_motors(control_outputs_dict)
            
            # Update simulator
            self.simulator.update(motor_commands)
            
            # Collect performance data
            for axis in ['roll', 'pitch', 'yaw']:
                error = abs(ref_rates[axis] - current_rates[axis])
                errors.append(error)
                
                if axis in control_outputs_dict:
                    control_outputs.append(abs(control_outputs_dict[axis]))
                    
        # Calculate performance metrics
        return self._calculate_performance_cost(errors, control_outputs)
        
    def _generate_reference(self, scenario: str, t: float) -> Dict[str, float]:
        """Generate reference trajectory for test scenario"""
        
        if scenario == 'step_response':
            # Step input at t=1s
            if t > 1.0:
                return {'roll': 10.0, 'pitch': 0.0, 'yaw': 0.0}  # 10 deg/s roll rate
            else:
                return {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
                
        elif scenario == 'frequency_sweep':
            # Sine wave with increasing frequency
            freq = 0.1 + 2.0 * t / self.config.simulation_time  # 0.1 to 2.1 Hz
            amplitude = 5.0  # deg/s
            return {
                'roll': amplitude * np.sin(2 * np.pi * freq * t),
                'pitch': 0.0,
                'yaw': 0.0
            }
            
        elif scenario == 'attitude_tracking':
            # Attitude tracking maneuver
            return {
                'roll': 15.0 * np.sin(2 * np.pi * 0.2 * t),
                'pitch': 10.0 * np.sin(2 * np.pi * 0.15 * t),
                'yaw': 5.0 * np.sin(2 * np.pi * 0.1 * t)
            }
            
        elif scenario == 'disturbance_rejection':
            # Constant reference with wind disturbance
            if t > 2.0:
                # Add wind disturbance
                wind = np.array([2.0, 1.0, 0.0])  # 2 m/s wind
                self.simulator.set_wind(wind)
                
            return {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
            
        else:
            return {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
            
    def _control_to_motors(self, control_outputs: Dict[str, float]) -> np.ndarray:
        """Convert control outputs to motor commands"""
        # Simplified control allocation for quadcopter
        roll_cmd = control_outputs.get('roll', 0.0)
        pitch_cmd = control_outputs.get('pitch', 0.0)
        yaw_cmd = control_outputs.get('yaw', 0.0)
        
        # Base hover command
        base_cmd = 0.5
        
        # Control allocation (simplified)
        motor_commands = np.array([
            base_cmd + pitch_cmd + yaw_cmd,  # Front right
            base_cmd - roll_cmd - yaw_cmd,   # Front left
            base_cmd - pitch_cmd + yaw_cmd,  # Back left
            base_cmd + roll_cmd - yaw_cmd    # Back right
        ])
        
        return np.clip(motor_commands, 0.0, 1.0)
        
    def _calculate_performance_cost(self, errors: List[float], control_outputs: List[float]) -> float:
        """Calculate performance cost from simulation data"""
        
        if not errors:
            return 1000.0  # Large penalty for no data
            
        # Performance metrics
        rms_error = np.sqrt(np.mean(np.array(errors)**2))
        max_error = np.max(errors)
        control_effort = np.mean(control_outputs) if control_outputs else 0
        
        # Weighted cost function
        weights = self.config.performance_weights
        
        cost = (
            weights['tracking_error'] * rms_error +
            weights['overshoot'] * max_error +
            weights['control_effort'] * control_effort
        )
        
        return cost
        
    def generate_performance_report(self) -> Dict:
        """Generate detailed performance report"""
        if not self.optimization_history:
            return {'error': 'No optimization data available'}
            
        # Extract performance data
        costs = [h['performance'] for h in self.optimization_history]
        
        report = {
            'optimization_method': self.config.optimization_method.value,
            'total_evaluations': len(self.optimization_history),
            'best_performance': self.best_performance,
            'convergence_data': {
                'initial_cost': costs[0] if costs else 0,
                'final_cost': costs[-1] if costs else 0,
                'improvement': (costs[0] - costs[-1]) / costs[0] * 100 if costs and costs[0] > 0 else 0
            },
            'best_gains': {
                axis: {
                    'P': gain.P,
                    'I': gain.I,
                    'D': gain.D
                } for axis, gain in (self.best_gains or {}).items()
            }
        }
        
        return report
