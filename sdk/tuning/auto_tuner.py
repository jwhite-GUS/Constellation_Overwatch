"""
Constellation Overwatch SDK - Auto-Tuner Implementation  
Advanced automatic tuning system for flight controllers with safety monitoring.
"""

import numpy as np
import logging
from typing import Dict, List, Tuple, Optional, Callable
from dataclasses import dataclass, field
from enum import Enum
import time
import threading
from scipy import signal
from scipy.optimize import minimize

from .pid_controller import PIDController, PIDGains, RatePIDController
from .safety_monitor import SafetyMonitor

class TuningMethod(Enum):
    """Available auto-tuning methods"""
    RELAY_FEEDBACK = "relay_feedback"      # Classical relay feedback method
    SINE_SWEEP = "sine_sweep"              # Frequency sweep identification
    STEP_RESPONSE = "step_response"        # Step response analysis
    RECURSIVE_LSE = "recursive_lse"        # Recursive least squares estimation
    ZIEGLER_NICHOLS = "ziegler_nichols"   # Ziegler-Nichols method

class TuningPhase(Enum):
    """Auto-tuning phases"""
    IDLE = "idle"
    SYSTEM_ID = "system_identification"
    PARAMETER_CALC = "parameter_calculation"  
    VALIDATION = "validation"
    COMPLETE = "complete"
    FAILED = "failed"

@dataclass
class TuningConfig:
    """Auto-tuning configuration parameters"""
    method: TuningMethod = TuningMethod.RELAY_FEEDBACK
    target_bandwidth: float = 10.0  # Hz
    phase_margin: float = 45.0      # degrees
    gain_margin: float = 6.0        # dB
    max_overshoot: float = 15.0     # percent
    settling_time: float = 0.5      # seconds
    
    # Safety limits
    max_output_amplitude: float = 0.5
    max_test_time: float = 60.0     # seconds
    min_stability_margin: float = 0.3
    
    # Relay feedback parameters
    relay_amplitude: float = 0.2
    hysteresis: float = 0.01
    
    # Sine sweep parameters
    freq_start: float = 0.1         # Hz
    freq_end: float = 100.0         # Hz
    sweep_time: float = 20.0        # seconds
    sine_amplitude: float = 0.1
    
@dataclass 
class TuningResult:
    """Auto-tuning results"""
    success: bool = False
    gains: Optional[PIDGains] = None
    system_model: Optional[Dict] = None
    performance_metrics: Dict = field(default_factory=dict)
    frequency_response: Optional[Dict] = None
    warnings: List[str] = field(default_factory=list)
    execution_time: float = 0.0

class SystemIdentifier:
    """System identification algorithms for auto-tuning"""
    
    @staticmethod
    def relay_feedback_identification(input_data: np.ndarray, 
                                    output_data: np.ndarray,
                                    sample_time: float) -> Dict:
        """
        Identify system using relay feedback method.
        
        Args:
            input_data: Relay input signal
            output_data: System response
            sample_time: Sampling period
            
        Returns:
            System identification results
        """
        # Find ultimate gain and period from relay test
        # Detect oscillation frequency and amplitude
        
        # Remove DC component
        output_centered = output_data - np.mean(output_data)
        
        # Find peaks for oscillation analysis
        peaks_pos, _ = signal.find_peaks(output_centered, height=0.1*np.std(output_centered))
        peaks_neg, _ = signal.find_peaks(-output_centered, height=0.1*np.std(output_centered))
        
        if len(peaks_pos) < 3 or len(peaks_neg) < 3:
            return {'success': False, 'error': 'Insufficient oscillation detected'}
            
        # Calculate oscillation period
        periods = np.diff(peaks_pos) * sample_time
        ultimate_period = np.mean(periods) if len(periods) > 0 else 0
        
        # Calculate oscillation amplitude
        amplitude = np.mean([np.mean(output_centered[peaks_pos]), 
                           abs(np.mean(output_centered[peaks_neg]))])
        
        if ultimate_period <= 0 or amplitude <= 0:
            return {'success': False, 'error': 'Invalid oscillation parameters'}
            
        # Calculate ultimate gain (Ku)
        relay_amplitude = np.mean(np.abs(input_data))
        ultimate_gain = (4 * relay_amplitude) / (np.pi * amplitude)
        
        return {
            'success': True,
            'ultimate_gain': ultimate_gain,
            'ultimate_period': ultimate_period,
            'ultimate_frequency': 1.0 / ultimate_period,
            'oscillation_amplitude': amplitude
        }
        
    @staticmethod
    def frequency_response_identification(input_data: np.ndarray,
                                        output_data: np.ndarray,
                                        frequencies: np.ndarray,
                                        sample_time: float) -> Dict:
        """
        Identify frequency response from sine sweep data.
        
        Args:
            input_data: Sine sweep input
            output_data: System response
            frequencies: Test frequencies
            sample_time: Sampling period
            
        Returns:
            Frequency response data
        """
        # Calculate frequency response using FFT
        from scipy.fft import fft, fftfreq
        
        n_samples = len(output_data)
        freqs = fftfreq(n_samples, sample_time)
        
        # FFT of input and output
        input_fft = fft(input_data)
        output_fft = fft(output_data)
        
        # Calculate transfer function
        H = output_fft / (input_fft + 1e-10)  # Avoid division by zero
        
        # Extract magnitude and phase
        magnitude = np.abs(H)
        phase = np.angle(H)
        
        # Find indices for desired frequencies
        positive_freqs = freqs[:n_samples//2]
        magnitude = magnitude[:n_samples//2]
        phase = phase[:n_samples//2]
        
        return {
            'frequencies': positive_freqs,
            'magnitude': magnitude,
            'phase': phase,
            'coherence': np.ones_like(magnitude)  # Simplified
        }

class AutoTuner:
    """
    Advanced auto-tuning system for flight controllers.
    
    Features:
    - Multiple identification methods (relay feedback, frequency sweep, step response)
    - Safety monitoring during tuning
    - Real-time parameter calculation
    - Validation testing
    - Performance optimization
    """
    
    def __init__(self, 
                 controller: PIDController,
                 config: TuningConfig,
                 safety_monitor: Optional[SafetyMonitor] = None):
        """
        Initialize auto-tuner.
        
        Args:
            controller: PID controller to tune
            config: Tuning configuration
            safety_monitor: Safety monitoring system
        """
        self.controller = controller
        self.config = config
        self.safety_monitor = safety_monitor or SafetyMonitor()
        
        self.phase = TuningPhase.IDLE
        self.start_time = 0.0
        self.test_data = {'time': [], 'input': [], 'output': [], 'setpoint': []}
        self.result = TuningResult()
        
        # Tuning state
        self.original_gains = controller.get_gains()
        self.test_input = 0.0
        self.oscillation_count = 0
        
        self.logger = logging.getLogger(__name__)
        self._stop_requested = False
        
    def start_tuning(self, measurement_callback: Callable[[], float],
                    output_callback: Callable[[float], None]) -> TuningResult:
        """
        Start automatic tuning process.
        
        Args:
            measurement_callback: Function to get current measurement
            output_callback: Function to apply control output
            
        Returns:
            Tuning results
        """
        self.logger.info(f"Starting auto-tuning with method: {self.config.method}")
        self.start_time = time.time()
        self._stop_requested = False
        
        try:
            # Phase 1: System Identification
            self.phase = TuningPhase.SYSTEM_ID
            system_model = self._perform_system_identification(measurement_callback, output_callback)
            
            if not system_model.get('success', False):
                self.result.success = False
                self.result.warnings.append("System identification failed")
                self.phase = TuningPhase.FAILED
                return self.result
                
            # Phase 2: Parameter Calculation
            self.phase = TuningPhase.PARAMETER_CALC
            new_gains = self._calculate_pid_parameters(system_model)
            
            if new_gains is None:
                self.result.success = False
                self.result.warnings.append("PID parameter calculation failed")
                self.phase = TuningPhase.FAILED
                return self.result
                
            # Phase 3: Validation
            self.phase = TuningPhase.VALIDATION
            validation_result = self._validate_parameters(new_gains, measurement_callback, output_callback)
            
            if not validation_result:
                self.result.success = False
                self.result.warnings.append("Parameter validation failed")
                self.phase = TuningPhase.FAILED
                return self.result
                
            # Success!
            self.phase = TuningPhase.COMPLETE
            self.result.success = True
            self.result.gains = new_gains
            self.result.system_model = system_model
            self.result.execution_time = time.time() - self.start_time
            
            self.logger.info(f"Auto-tuning completed successfully in {self.result.execution_time:.1f}s")
            
        except Exception as e:
            self.logger.error(f"Auto-tuning failed with exception: {e}")
            self.result.success = False
            self.result.warnings.append(f"Exception: {str(e)}")
            self.phase = TuningPhase.FAILED
            
        finally:
            # Restore original gains if tuning failed
            if not self.result.success:
                self.controller.set_gains(self.original_gains)
                
        return self.result
        
    def _perform_system_identification(self, 
                                     measurement_callback: Callable[[], float],
                                     output_callback: Callable[[float], None]) -> Dict:
        """Perform system identification using configured method"""
        
        if self.config.method == TuningMethod.RELAY_FEEDBACK:
            return self._relay_feedback_test(measurement_callback, output_callback)
        elif self.config.method == TuningMethod.SINE_SWEEP:
            return self._sine_sweep_test(measurement_callback, output_callback)
        elif self.config.method == TuningMethod.STEP_RESPONSE:
            return self._step_response_test(measurement_callback, output_callback)
        else:
            self.logger.warning(f"Method {self.config.method} not implemented, using relay feedback")
            return self._relay_feedback_test(measurement_callback, output_callback)
            
    def _relay_feedback_test(self,
                           measurement_callback: Callable[[], float],
                           output_callback: Callable[[float], None]) -> Dict:
        """
        Perform relay feedback test for system identification.
        
        The relay feedback method applies a relay (bang-bang) controller
        and analyzes the resulting oscillations to determine system parameters.
        """
        self.logger.info("Starting relay feedback test")
        
        # Reset controller and data
        self.controller.reset()
        self.test_data = {'time': [], 'input': [], 'output': [], 'setpoint': []}
        
        # Test parameters
        relay_amplitude = self.config.relay_amplitude
        hysteresis = self.config.hysteresis
        max_test_time = self.config.max_test_time
        sample_time = self.controller.sample_time
        
        start_time = time.time()
        setpoint = 0.0  # Test around zero setpoint
        relay_state = 1.0
        
        while (time.time() - start_time) < max_test_time and not self._stop_requested:
            current_time = time.time() - start_time
            
            # Get measurement
            measurement = measurement_callback()
            
            # Relay logic with hysteresis
            error = setpoint - measurement
            if error > hysteresis:
                relay_state = relay_amplitude
            elif error < -hysteresis:
                relay_state = -relay_amplitude
            # Otherwise maintain current relay state
            
            # Apply relay output
            output_callback(relay_state)
            
            # Store data
            self.test_data['time'].append(current_time)
            self.test_data['input'].append(relay_state)
            self.test_data['output'].append(measurement)
            self.test_data['setpoint'].append(setpoint)
            
            # Safety check
            if not self.safety_monitor.is_safe(measurement, relay_state, current_time):
                self.logger.warning("Safety monitor triggered, stopping relay test")
                break
                
            # Check for sufficient oscillation
            if len(self.test_data['output']) > 1000:  # At least 10 seconds at 100Hz
                # Quick oscillation check
                recent_output = np.array(self.test_data['output'][-500:])
                if np.std(recent_output) > 0.01:  # Some oscillation detected
                    self.oscillation_count += 1
                    
                if self.oscillation_count > 5:  # Multiple oscillation periods detected
                    break
                    
            time.sleep(sample_time)
            
        # Analyze relay test data
        if len(self.test_data['output']) < 100:
            return {'success': False, 'error': 'Insufficient test data collected'}
            
        input_array = np.array(self.test_data['input'])
        output_array = np.array(self.test_data['output'])
        
        # Use SystemIdentifier for analysis
        identification_result = SystemIdentifier.relay_feedback_identification(
            input_array, output_array, sample_time
        )
        
        if identification_result['success']:
            self.logger.info(f"Relay test successful: Ku={identification_result['ultimate_gain']:.3f}, "
                           f"Tu={identification_result['ultimate_period']:.3f}s")
                           
        return identification_result
        
    def _sine_sweep_test(self,
                        measurement_callback: Callable[[], float], 
                        output_callback: Callable[[float], None]) -> Dict:
        """
        Perform frequency sweep test for system identification.
        
        Applies logarithmic sine sweep and analyzes frequency response.
        """
        self.logger.info("Starting sine sweep test")
        
        # Test parameters
        freq_start = self.config.freq_start
        freq_end = self.config.freq_end
        sweep_time = self.config.sweep_time
        amplitude = self.config.sine_amplitude
        sample_time = self.controller.sample_time
        
        # Generate frequency sweep
        t = np.arange(0, sweep_time, sample_time)
        frequencies = np.logspace(np.log10(freq_start), np.log10(freq_end), len(t))
        
        # Reset data collection
        self.test_data = {'time': [], 'input': [], 'output': [], 'frequencies': frequencies}
        
        start_time = time.time()
        
        for i, freq in enumerate(frequencies):
            if self._stop_requested:
                break
                
            current_time = time.time() - start_time
            
            # Generate sine wave
            sine_input = amplitude * np.sin(2 * np.pi * freq * current_time)
            
            # Apply input
            output_callback(sine_input)
            
            # Get measurement  
            measurement = measurement_callback()
            
            # Store data
            self.test_data['time'].append(current_time)
            self.test_data['input'].append(sine_input)
            self.test_data['output'].append(measurement)
            
            # Safety check
            if not self.safety_monitor.is_safe(measurement, sine_input, current_time):
                self.logger.warning("Safety monitor triggered, stopping sweep test")
                break
                
            time.sleep(sample_time)
            
        # Analyze frequency response
        if len(self.test_data['output']) < 100:
            return {'success': False, 'error': 'Insufficient sweep data collected'}
            
        input_array = np.array(self.test_data['input'])
        output_array = np.array(self.test_data['output'])
        freq_array = frequencies[:len(output_array)]
        
        # Use SystemIdentifier for analysis
        freq_response = SystemIdentifier.frequency_response_identification(
            input_array, output_array, freq_array, sample_time
        )
        
        return {
            'success': True,
            'frequency_response': freq_response,
            'method': 'sine_sweep'
        }
        
    def _step_response_test(self,
                          measurement_callback: Callable[[], float],
                          output_callback: Callable[[float], None]) -> Dict:
        """Perform step response test for system identification"""
        self.logger.info("Starting step response test")
        
        # Apply step input and measure response
        step_amplitude = self.config.max_output_amplitude * 0.5
        test_time = min(self.config.max_test_time, 10.0)  # Shorter for step test
        sample_time = self.controller.sample_time
        
        # Reset data
        self.test_data = {'time': [], 'input': [], 'output': []}
        
        start_time = time.time()
        
        # Initial settling period (zero input)
        for i in range(int(1.0 / sample_time)):  # 1 second
            measurement = measurement_callback()
            output_callback(0.0)
            self.test_data['time'].append(i * sample_time)
            self.test_data['input'].append(0.0)
            self.test_data['output'].append(measurement)
            time.sleep(sample_time)
            
        # Step input period
        step_start_time = time.time()
        while (time.time() - step_start_time) < test_time and not self._stop_requested:
            current_time = time.time() - start_time
            measurement = measurement_callback()
            
            output_callback(step_amplitude)
            
            self.test_data['time'].append(current_time)
            self.test_data['input'].append(step_amplitude)
            self.test_data['output'].append(measurement)
            
            # Safety check
            if not self.safety_monitor.is_safe(measurement, step_amplitude, current_time):
                break
                
            time.sleep(sample_time)
            
        # Analyze step response
        if len(self.test_data['output']) < 50:
            return {'success': False, 'error': 'Insufficient step response data'}
            
        # Simple step response analysis
        output_array = np.array(self.test_data['output'])
        step_start_idx = int(1.0 / sample_time)  # Start of step
        
        if len(output_array) <= step_start_idx:
            return {'success': False, 'error': 'Step response too short'}
            
        initial_value = np.mean(output_array[:step_start_idx])
        final_value = np.mean(output_array[-50:]) if len(output_array) > 50 else output_array[-1]
        
        # Find settling time and overshoot
        step_response = output_array[step_start_idx:]
        steady_state = final_value - initial_value
        
        if abs(steady_state) < 1e-6:
            return {'success': False, 'error': 'No significant step response'}
            
        # Estimate time constant (63% of final value)
        target_63 = initial_value + 0.63 * steady_state
        
        time_constant = None
        for i, value in enumerate(step_response):
            if abs(value - target_63) < abs(steady_state) * 0.1:
                time_constant = i * sample_time
                break
                
        return {
            'success': True,
            'time_constant': time_constant or 1.0,
            'steady_state_gain': steady_state / step_amplitude,
            'overshoot': max(step_response) - final_value if len(step_response) > 0 else 0,
            'method': 'step_response'
        }
        
    def _calculate_pid_parameters(self, system_model: Dict) -> Optional[PIDGains]:
        """
        Calculate PID parameters from system identification results.
        
        Args:
            system_model: System identification results
            
        Returns:
            Calculated PID gains or None if calculation fails
        """
        try:
            if system_model.get('method') == 'step_response':
                return self._calculate_from_step_response(system_model)
            elif 'ultimate_gain' in system_model:
                return self._calculate_ziegler_nichols(system_model)
            elif 'frequency_response' in system_model:
                return self._calculate_from_frequency_response(system_model)
            else:
                self.logger.error("Unknown system model format for PID calculation")
                return None
                
        except Exception as e:
            self.logger.error(f"PID parameter calculation failed: {e}")
            return None
            
    def _calculate_ziegler_nichols(self, system_model: Dict) -> PIDGains:
        """
        Calculate PID gains using Ziegler-Nichols method from relay test.
        
        Args:
            system_model: Must contain 'ultimate_gain' and 'ultimate_period'
            
        Returns:
            PID gains
        """
        Ku = system_model['ultimate_gain']
        Tu = system_model['ultimate_period']
        
        # Classic Ziegler-Nichols PID tuning rules
        # These are conservative values based on research
        Kp = 0.6 * Ku
        Ki = 1.2 * Ku / Tu  
        Kd = 0.075 * Ku * Tu
        
        # Apply tuning aggressiveness factor (similar to ArduPilot AUTOTUNE_AGGR)
        aggressiveness = 0.75  # Conservative default
        
        Kp *= aggressiveness
        Ki *= aggressiveness
        Kd *= aggressiveness
        
        # Ensure reasonable limits
        Kp = np.clip(Kp, 0.01, 2.0)
        Ki = np.clip(Ki, 0.0, 5.0)
        Kd = np.clip(Kd, 0.0, 0.5)
        
        gains = PIDGains(
            P=Kp,
            I=Ki, 
            D=Kd,
            output_min=self.original_gains.output_min,
            output_max=self.original_gains.output_max
        )
        
        self.logger.info(f"Ziegler-Nichols PID gains: P={Kp:.4f}, I={Ki:.4f}, D={Kd:.4f}")
        
        return gains
        
    def _calculate_from_step_response(self, system_model: Dict) -> PIDGains:
        """Calculate PID gains from step response analysis"""
        time_constant = system_model.get('time_constant', 1.0)
        steady_state_gain = system_model.get('steady_state_gain', 1.0)
        
        # Cohen-Coon method for step response
        if abs(steady_state_gain) < 1e-6:
            steady_state_gain = 1.0
            
        Kp = 1.2 / steady_state_gain
        Ki = Kp / (2.0 * time_constant)
        Kd = 0.5 * Kp * time_constant
        
        # Apply safety limits
        Kp = np.clip(Kp, 0.01, 2.0)
        Ki = np.clip(Ki, 0.0, 2.0)
        Kd = np.clip(Kd, 0.0, 0.2)
        
        return PIDGains(P=Kp, I=Ki, D=Kd,
                       output_min=self.original_gains.output_min,
                       output_max=self.original_gains.output_max)
                       
    def _calculate_from_frequency_response(self, system_model: Dict) -> PIDGains:
        """Calculate PID gains from frequency response data"""
        # This is a simplified approach - would need more sophisticated
        # frequency domain design for production use
        
        freq_response = system_model['frequency_response']
        frequencies = freq_response['frequencies']
        magnitude = freq_response['magnitude']
        
        # Find crossover frequency (where gain ≈ 1)
        crossover_idx = np.argmin(np.abs(magnitude - 1.0))
        crossover_freq = frequencies[crossover_idx]
        
        # Design for desired bandwidth
        desired_bandwidth = self.config.target_bandwidth
        
        # Simple approximation
        Kp = 2 * np.pi * desired_bandwidth
        Ki = Kp / 4.0
        Kd = Kp / (8 * np.pi * desired_bandwidth)
        
        return PIDGains(P=Kp, I=Ki, D=Kd,
                       output_min=self.original_gains.output_min,
                       output_max=self.original_gains.output_max)
        
    def _validate_parameters(self, 
                           gains: PIDGains,
                           measurement_callback: Callable[[], float],
                           output_callback: Callable[[float], None]) -> bool:
        """
        Validate calculated PID parameters with closed-loop test.
        
        Args:
            gains: PID gains to validate
            measurement_callback: Measurement function
            output_callback: Output function
            
        Returns:
            True if validation successful
        """
        self.logger.info("Starting parameter validation")
        
        # Apply new gains
        self.controller.set_gains(gains)
        self.controller.reset()
        
        # Validation test parameters
        test_time = 5.0  # 5 second validation
        sample_time = self.controller.sample_time
        setpoint = 0.2  # Small setpoint step
        
        validation_data = {'time': [], 'setpoint': [], 'output': [], 'control': []}
        
        start_time = time.time()
        max_error = 0.0
        oscillation_count = 0
        previous_error = 0.0
        
        while (time.time() - start_time) < test_time and not self._stop_requested:
            current_time = time.time() - start_time
            
            # Get measurement
            measurement = measurement_callback()
            
            # Update controller
            control_output = self.controller.update(setpoint, measurement)
            
            # Apply control output
            output_callback(control_output)
            
            # Monitor performance
            error = abs(setpoint - measurement)
            max_error = max(max_error, error)
            
            # Count oscillations
            if (error - previous_error) * previous_error < 0:  # Sign change
                oscillation_count += 1
            previous_error = error
            
            # Store validation data
            validation_data['time'].append(current_time)
            validation_data['setpoint'].append(setpoint)
            validation_data['output'].append(measurement)
            validation_data['control'].append(control_output)
            
            # Safety check
            if not self.safety_monitor.is_safe(measurement, control_output, current_time):
                self.logger.warning("Safety violation during validation")
                return False
                
            # Check for instability
            if max_error > 1.0 or oscillation_count > 20:
                self.logger.warning("Validation failed: unstable response")
                return False
                
            time.sleep(sample_time)
            
        # Analyze validation results
        if len(validation_data['output']) < 10:
            return False
            
        # Calculate performance metrics
        errors = [abs(setpoint - out) for out in validation_data['output']]
        
        # Settling criteria
        settling_time = None
        for i in range(len(errors)):
            if i > len(errors) // 2:  # Second half of test
                if all(e < 0.05 for e in errors[i:i+10]) and i+10 < len(errors):
                    settling_time = validation_data['time'][i]
                    break
                    
        # Validation criteria
        final_error = np.mean(errors[-10:]) if len(errors) >= 10 else errors[-1]
        max_overshoot = max(validation_data['output']) - setpoint if validation_data['output'] else 0
        
        # Success criteria
        validation_success = (
            final_error < 0.1 and              # Good steady-state accuracy
            max_overshoot < setpoint * 0.3 and  # Limited overshoot  
            oscillation_count < 10 and          # Stable response
            (settling_time is None or settling_time < test_time * 0.8)  # Reasonable settling
        )
        
        if validation_success:
            self.result.performance_metrics = {
                'final_error': final_error,
                'max_overshoot': max_overshoot,
                'settling_time': settling_time,
                'oscillation_count': oscillation_count
            }
            self.logger.info(f"Validation successful: final_error={final_error:.3f}, "
                           f"overshoot={max_overshoot:.3f}")
        else:
            self.logger.warning(f"Validation failed: final_error={final_error:.3f}, "
                              f"overshoot={max_overshoot:.3f}, oscillations={oscillation_count}")
            
        return validation_success
        
    def stop_tuning(self):
        """Stop tuning process"""
        self._stop_requested = True
        self.phase = TuningPhase.IDLE
        
    def get_progress(self) -> float:
        """Get tuning progress as percentage"""
        if self.phase == TuningPhase.IDLE:
            return 0.0
        elif self.phase == TuningPhase.SYSTEM_ID:
            return 30.0
        elif self.phase == TuningPhase.PARAMETER_CALC:
            return 70.0
        elif self.phase == TuningPhase.VALIDATION:
            return 90.0
        elif self.phase == TuningPhase.COMPLETE:
            return 100.0
        else:
            return 0.0
            
    def get_current_phase(self) -> TuningPhase:
        """Get current tuning phase"""
        return self.phase
