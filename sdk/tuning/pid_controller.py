"""
Constellation Overwatch SDK - PID Controller Implementation
Advanced PID controllers with auto-tuning capabilities for flight control systems.
"""

import numpy as np
import logging
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum

class PIDForm(Enum):
    """PID controller mathematical forms"""
    PARALLEL = "parallel"      # Output = P + I + D
    STANDARD = "standard"      # Output = K(error + (1/Ti)∫error + Td*d(error)/dt)

@dataclass
class PIDGains:
    """PID gain parameters"""
    P: float = 0.1
    I: float = 0.0
    D: float = 0.0
    K: float = 1.0  # For standard form
    Ti: float = 0.0  # Integral time constant
    Td: float = 0.0  # Derivative time constant
    
    # Limits
    output_min: float = -1.0
    output_max: float = 1.0
    integral_min: float = -1.0
    integral_max: float = 1.0

@dataclass 
class PIDState:
    """PID controller internal state"""
    previous_error: float = 0.0
    integral: float = 0.0
    derivative: float = 0.0
    previous_measurement: float = 0.0
    last_time: float = 0.0

class PIDController:
    """
    Advanced PID Controller with multiple forms and auto-tuning support.
    
    Supports both parallel and standard PID forms as used in ArduPilot and PX4:
    - Parallel: u = P*e + I*∫e + D*de/dt
    - Standard: u = K(e + (1/Ti)*∫e + Td*de/dt)
    
    Features:
    - Derivative kick prevention (D-term on measurement, not error)
    - Integral windup protection
    - Output clamping
    - Real-time parameter adjustment
    - Performance monitoring
    """
    
    def __init__(self, 
                 gains: PIDGains,
                 form: PIDForm = PIDForm.PARALLEL,
                 sample_time: float = 0.01,
                 derivative_on_measurement: bool = True):
        """
        Initialize PID controller.
        
        Args:
            gains: PID gain parameters
            form: Mathematical form (parallel or standard)
            sample_time: Control loop sample time in seconds
            derivative_on_measurement: True to compute D-term on measurement to prevent derivative kick
        """
        self.gains = gains
        self.form = form
        self.sample_time = sample_time
        self.derivative_on_measurement = derivative_on_measurement
        self.state = PIDState()
        
        # Performance monitoring
        self.performance_history = []
        self.oscillation_count = 0
        self.settling_time = 0.0
        
        self.logger = logging.getLogger(__name__)
        
    def update(self, setpoint: float, measurement: float, dt: Optional[float] = None) -> float:
        """
        Update PID controller and return control output.
        
        Args:
            setpoint: Desired value
            measurement: Current measured value
            dt: Time step (uses sample_time if None)
            
        Returns:
            Control output value
        """
        if dt is None:
            dt = self.sample_time
            
        error = setpoint - measurement
        
        # Proportional term
        if self.form == PIDForm.PARALLEL:
            proportional = self.gains.P * error
        else:  # Standard form
            proportional = self.gains.K * error
            
        # Integral term with windup protection
        integral_term = self._update_integral(error, dt)
        
        # Derivative term (on measurement to prevent derivative kick)
        derivative_term = self._update_derivative(error, measurement, dt)
        
        # Calculate output
        if self.form == PIDForm.PARALLEL:
            output = proportional + integral_term + derivative_term
        else:  # Standard form
            integral_component = (self.gains.K / self.gains.Ti) * self.state.integral if self.gains.Ti > 0 else 0
            derivative_component = self.gains.K * self.gains.Td * self.state.derivative
            output = proportional + integral_component + derivative_component
            
        # Clamp output
        output = np.clip(output, self.gains.output_min, self.gains.output_max)
        
        # Update state
        self.state.previous_error = error
        self.state.previous_measurement = measurement
        
        # Performance monitoring
        self._monitor_performance(error, output, dt)
        
        return output
        
    def _update_integral(self, error: float, dt: float) -> float:
        """Update integral term with anti-windup"""
        # Only accumulate if within limits
        new_integral = self.state.integral + error * dt
        new_integral = np.clip(new_integral, self.gains.integral_min, self.gains.integral_max)
        
        if self.form == PIDForm.PARALLEL:
            integral_term = self.gains.I * new_integral
        else:
            integral_term = new_integral
            
        # Anti-windup: only update integral if not saturated
        self.state.integral = new_integral
        
        return integral_term
        
    def _update_derivative(self, error: float, measurement: float, dt: float) -> float:
        """Update derivative term"""
        if dt <= 0:
            return 0.0
            
        if self.derivative_on_measurement:
            # D-term on measurement (prevents derivative kick)
            derivative = -(measurement - self.state.previous_measurement) / dt
        else:
            # D-term on error (traditional)
            derivative = (error - self.state.previous_error) / dt
            
        self.state.derivative = derivative
        
        if self.form == PIDForm.PARALLEL:
            return self.gains.D * derivative
        else:
            return derivative  # Will be scaled by K*Td in main calculation
            
    def _monitor_performance(self, error: float, output: float, dt: float):
        """Monitor controller performance for auto-tuning feedback"""
        # Track oscillations
        if len(self.performance_history) >= 2:
            prev_error = self.performance_history[-1]['error']
            if np.sign(error) != np.sign(prev_error) and abs(error) > 0.01:
                self.oscillation_count += 1
                
        # Store performance data
        self.performance_history.append({
            'time': self.performance_history[-1]['time'] + dt if self.performance_history else 0,
            'error': error,
            'output': output,
            'derivative': self.state.derivative,
            'integral': self.state.integral
        })
        
        # Keep only recent history (last 1000 samples)
        if len(self.performance_history) > 1000:
            self.performance_history.pop(0)
            
    def get_gains(self) -> PIDGains:
        """Get current PID gains"""
        return self.gains
        
    def set_gains(self, gains: PIDGains):
        """Set new PID gains"""
        self.gains = gains
        self.logger.info(f"Updated PID gains: P={gains.P}, I={gains.I}, D={gains.D}")
        
    def reset(self):
        """Reset controller state"""
        self.state = PIDState()
        self.performance_history.clear()
        self.oscillation_count = 0
        
    def get_performance_metrics(self) -> Dict:
        """
        Get performance metrics for tuning analysis.
        
        Returns:
            Dictionary with performance metrics
        """
        if len(self.performance_history) < 10:
            return {}
            
        errors = [h['error'] for h in self.performance_history]
        outputs = [h['output'] for h in self.performance_history]
        
        # Calculate settling time (time to stay within 5% of setpoint)
        settling_threshold = 0.05
        settling_time = None
        for i, error in enumerate(reversed(errors)):
            if abs(error) > settling_threshold:
                settling_time = i * self.sample_time
                break
                
        # Calculate overshoot
        max_error = max(errors) if errors else 0
        min_error = min(errors) if errors else 0
        overshoot = max(abs(max_error), abs(min_error))
        
        # Calculate oscillation frequency
        oscillation_frequency = self.oscillation_count / (len(self.performance_history) * self.sample_time) if self.performance_history else 0
        
        return {
            'rms_error': np.sqrt(np.mean(np.array(errors)**2)),
            'max_overshoot': overshoot,
            'settling_time': settling_time,
            'oscillation_frequency': oscillation_frequency,
            'output_saturation': sum(1 for o in outputs if abs(o) >= 0.95) / len(outputs)
        }
        
    def is_stable(self) -> bool:
        """Check if controller is stable based on recent performance"""
        metrics = self.get_performance_metrics()
        if not metrics:
            return True  # No data yet
            
        # Stability criteria
        stable = (
            metrics.get('oscillation_frequency', 0) < 2.0 and  # Less than 2 Hz oscillation
            metrics.get('output_saturation', 0) < 0.1 and      # Less than 10% saturation
            metrics.get('rms_error', float('inf')) < 0.2        # RMS error less than 0.2
        )
        
        return stable
        
    def suggest_gain_adjustments(self) -> Dict[str, float]:
        """
        Suggest gain adjustments based on performance analysis.
        
        Returns:
            Dictionary with suggested gain multipliers
        """
        metrics = self.get_performance_metrics()
        suggestions = {'P': 1.0, 'I': 1.0, 'D': 1.0}
        
        if not metrics:
            return suggestions
            
        # High oscillation frequency -> reduce P gain
        if metrics.get('oscillation_frequency', 0) > 1.0:
            suggestions['P'] = 0.8
            
        # High overshoot -> reduce P, increase D
        if metrics.get('max_overshoot', 0) > 0.3:
            suggestions['P'] = 0.9
            suggestions['D'] = 1.2
            
        # High steady-state error -> increase I gain
        if metrics.get('rms_error', 0) > 0.1:
            suggestions['I'] = 1.1
            
        # High output saturation -> reduce all gains
        if metrics.get('output_saturation', 0) > 0.2:
            suggestions = {k: v * 0.8 for k, v in suggestions.items()}
            
        return suggestions

class RatePIDController:
    """
    Multi-axis rate PID controller for drone attitude control.
    
    Manages roll, pitch, and yaw rate controllers with cross-axis coordination.
    """
    
    def __init__(self, sample_time: float = 0.01):
        """Initialize multi-axis rate controller"""
        self.sample_time = sample_time
        
        # Default gains based on research data (conservative starting values)
        roll_gains = PIDGains(P=0.05, I=0.03, D=0.003, output_min=-1.0, output_max=1.0)
        pitch_gains = PIDGains(P=0.05, I=0.03, D=0.003, output_min=-1.0, output_max=1.0) 
        yaw_gains = PIDGains(P=0.1, I=0.05, D=0.0, output_min=-1.0, output_max=1.0)
        
        self.controllers = {
            'roll': PIDController(roll_gains, sample_time=sample_time),
            'pitch': PIDController(pitch_gains, sample_time=sample_time),
            'yaw': PIDController(yaw_gains, sample_time=sample_time)
        }
        
        self.logger = logging.getLogger(__name__)
        
    def update(self, rate_setpoints: Dict[str, float], 
               rate_measurements: Dict[str, float],
               dt: Optional[float] = None) -> Dict[str, float]:
        """
        Update all rate controllers.
        
        Args:
            rate_setpoints: Desired rates {'roll': rad/s, 'pitch': rad/s, 'yaw': rad/s}
            rate_measurements: Measured rates {'roll': rad/s, 'pitch': rad/s, 'yaw': rad/s}
            dt: Time step
            
        Returns:
            Control outputs {'roll': output, 'pitch': output, 'yaw': output}
        """
        outputs = {}
        
        for axis in ['roll', 'pitch', 'yaw']:
            setpoint = rate_setpoints.get(axis, 0.0)
            measurement = rate_measurements.get(axis, 0.0)
            outputs[axis] = self.controllers[axis].update(setpoint, measurement, dt)
            
        return outputs
        
    def get_all_gains(self) -> Dict[str, PIDGains]:
        """Get gains for all axes"""
        return {axis: ctrl.get_gains() for axis, ctrl in self.controllers.items()}
        
    def set_all_gains(self, gains: Dict[str, PIDGains]):
        """Set gains for all axes"""
        for axis, gain in gains.items():
            if axis in self.controllers:
                self.controllers[axis].set_gains(gain)
                
    def reset_all(self):
        """Reset all controllers"""
        for controller in self.controllers.values():
            controller.reset()
            
    def is_stable(self) -> bool:
        """Check if all controllers are stable"""
        return all(ctrl.is_stable() for ctrl in self.controllers.values())
        
    def get_performance_summary(self) -> Dict:
        """Get performance summary for all axes"""
        summary = {}
        for axis, controller in self.controllers.items():
            summary[axis] = controller.get_performance_metrics()
        return summary
