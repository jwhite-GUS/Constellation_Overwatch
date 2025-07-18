"""
Constellation Overwatch SDK - Tuning Analyzer
Advanced analysis tools for flight controller tuning and performance evaluation.
"""

import numpy as np
import logging
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass, field
from enum import Enum
import time
import json
from scipy import signal
from scipy.stats import pearsonr
import matplotlib.pyplot as plt
from pathlib import Path

from .pid_controller import PIDController, PIDGains

class AnalysisType(Enum):
    """Types of tuning analysis"""
    STEP_RESPONSE = "step_response"
    FREQUENCY_RESPONSE = "frequency_response"
    STABILITY_ANALYSIS = "stability_analysis"
    DISTURBANCE_REJECTION = "disturbance_rejection"
    NOISE_ANALYSIS = "noise_analysis"
    PERFORMANCE_COMPARISON = "performance_comparison"

@dataclass
class PerformanceMetrics:
    """Performance metrics for controller analysis"""
    # Time domain metrics
    rise_time: float = 0.0          # Time to reach 90% of final value
    settling_time: float = 0.0       # Time to settle within 2% of final value
    overshoot: float = 0.0          # Maximum overshoot percentage
    undershoot: float = 0.0         # Maximum undershoot percentage
    steady_state_error: float = 0.0  # Final tracking error
    
    # Frequency domain metrics
    bandwidth: float = 0.0          # -3dB bandwidth
    phase_margin: float = 0.0       # Phase margin in degrees
    gain_margin: float = 0.0        # Gain margin in dB
    crossover_frequency: float = 0.0 # Gain crossover frequency
    
    # Statistical metrics
    rms_error: float = 0.0          # RMS tracking error
    max_error: float = 0.0          # Maximum tracking error
    mean_error: float = 0.0         # Mean tracking error
    error_variance: float = 0.0     # Error variance
    
    # Control effort metrics
    control_effort: float = 0.0     # Integrated absolute control effort
    max_control: float = 0.0        # Maximum control output
    control_variance: float = 0.0   # Control signal variance
    
    # Stability metrics
    stability_margin: float = 0.0   # Stability margin
    oscillation_frequency: float = 0.0 # Dominant oscillation frequency
    damping_ratio: float = 0.0      # Damping ratio
    
    def to_dict(self) -> Dict:
        """Convert metrics to dictionary"""
        return {
            'time_domain': {
                'rise_time': self.rise_time,
                'settling_time': self.settling_time,
                'overshoot': self.overshoot,
                'undershoot': self.undershoot,
                'steady_state_error': self.steady_state_error
            },
            'frequency_domain': {
                'bandwidth': self.bandwidth,
                'phase_margin': self.phase_margin,
                'gain_margin': self.gain_margin,
                'crossover_frequency': self.crossover_frequency
            },
            'statistical': {
                'rms_error': self.rms_error,
                'max_error': self.max_error,
                'mean_error': self.mean_error,
                'error_variance': self.error_variance
            },
            'control_effort': {
                'control_effort': self.control_effort,
                'max_control': self.max_control,
                'control_variance': self.control_variance
            },
            'stability': {
                'stability_margin': self.stability_margin,
                'oscillation_frequency': self.oscillation_frequency,
                'damping_ratio': self.damping_ratio
            }
        }

@dataclass
class AnalysisResult:
    """Result of tuning analysis"""
    analysis_type: AnalysisType
    metrics: PerformanceMetrics
    data: Dict[str, Any] = field(default_factory=dict)
    recommendations: List[str] = field(default_factory=list)
    plots: List[str] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)
    
    def to_dict(self) -> Dict:
        """Convert result to dictionary"""
        return {
            'analysis_type': self.analysis_type.value,
            'metrics': self.metrics.to_dict(),
            'data': self.data,
            'recommendations': self.recommendations,
            'plots': self.plots,
            'timestamp': self.timestamp
        }

class TuningAnalyzer:
    """
    Advanced tuning analyzer for flight controller performance evaluation.
    
    Features:
    - Comprehensive performance metrics calculation
    - Multiple analysis types (time/frequency domain)
    - Automatic tuning recommendations
    - Comparative analysis between configurations
    - Report generation with visualizations
    - Real-time performance monitoring
    """
    
    def __init__(self, sample_rate: float = 100.0):
        """
        Initialize tuning analyzer.
        
        Args:
            sample_rate: Data sample rate in Hz
        """
        self.sample_rate = sample_rate
        self.dt = 1.0 / sample_rate
        
        # Analysis history
        self.analysis_history: List[AnalysisResult] = []
        
        # Configuration
        self.enable_plots = True
        self.plot_directory = Path("tuning_plots")
        self.plot_directory.mkdir(exist_ok=True)
        
        self.logger = logging.getLogger(__name__)
        
    def analyze_step_response(self, 
                            time_data: np.ndarray,
                            setpoint_data: np.ndarray,
                            response_data: np.ndarray,
                            control_data: Optional[np.ndarray] = None) -> AnalysisResult:
        """
        Analyze step response performance.
        
        Args:
            time_data: Time vector
            setpoint_data: Reference/setpoint signal
            response_data: System response
            control_data: Control signal (optional)
            
        Returns:
            Analysis result with performance metrics
        """
        self.logger.info("Analyzing step response")
        
        # Calculate performance metrics
        metrics = self._calculate_time_domain_metrics(time_data, setpoint_data, response_data)
        
        if control_data is not None:
            control_metrics = self._calculate_control_metrics(control_data)
            metrics.control_effort = control_metrics['control_effort']
            metrics.max_control = control_metrics['max_control']
            metrics.control_variance = control_metrics['control_variance']
            
        # Generate recommendations
        recommendations = self._generate_step_response_recommendations(metrics)
        
        # Create plots
        plots = []
        if self.enable_plots:
            plot_path = self._plot_step_response(time_data, setpoint_data, response_data, control_data)
            if plot_path:
                plots.append(plot_path)
                
        # Create result
        result = AnalysisResult(
            analysis_type=AnalysisType.STEP_RESPONSE,
            metrics=metrics,
            data={
                'time': time_data.tolist(),
                'setpoint': setpoint_data.tolist(),
                'response': response_data.tolist(),
                'control': control_data.tolist() if control_data is not None else None
            },
            recommendations=recommendations,
            plots=plots
        )
        
        self.analysis_history.append(result)
        return result
        
    def analyze_frequency_response(self,
                                 frequencies: np.ndarray,
                                 magnitude: np.ndarray,
                                 phase: np.ndarray) -> AnalysisResult:
        """
        Analyze frequency response characteristics.
        
        Args:
            frequencies: Frequency vector (Hz)
            magnitude: Magnitude response
            phase: Phase response (degrees)
            
        Returns:
            Analysis result with frequency domain metrics
        """
        self.logger.info("Analyzing frequency response")
        
        # Calculate frequency domain metrics
        metrics = PerformanceMetrics()
        
        # Convert magnitude to dB
        magnitude_db = 20 * np.log10(magnitude + 1e-10)
        
        # Find bandwidth (-3dB point)
        bandwidth_idx = np.where(magnitude_db <= -3.0)[0]
        if len(bandwidth_idx) > 0:
            metrics.bandwidth = frequencies[bandwidth_idx[0]]
        else:
            metrics.bandwidth = frequencies[-1]  # Use max frequency if no -3dB point
            
        # Find gain crossover frequency (magnitude = 1 or 0 dB)
        crossover_idx = np.argmin(np.abs(magnitude_db))
        metrics.crossover_frequency = frequencies[crossover_idx]
        
        # Calculate phase margin
        if crossover_idx < len(phase):
            metrics.phase_margin = 180.0 + phase[crossover_idx]
        else:
            metrics.phase_margin = 180.0
            
        # Calculate gain margin
        phase_180_idx = np.where(phase <= -180.0)[0]
        if len(phase_180_idx) > 0:
            metrics.gain_margin = -magnitude_db[phase_180_idx[0]]
        else:
            metrics.gain_margin = 40.0  # Large gain margin if no -180° phase
            
        # Generate recommendations
        recommendations = self._generate_frequency_response_recommendations(metrics)
        
        # Create plots
        plots = []
        if self.enable_plots:
            plot_path = self._plot_frequency_response(frequencies, magnitude_db, phase)
            if plot_path:
                plots.append(plot_path)
                
        result = AnalysisResult(
            analysis_type=AnalysisType.FREQUENCY_RESPONSE,
            metrics=metrics,
            data={
                'frequencies': frequencies.tolist(),
                'magnitude_db': magnitude_db.tolist(),
                'phase': phase.tolist()
            },
            recommendations=recommendations,
            plots=plots
        )
        
        self.analysis_history.append(result)
        return result
        
    def analyze_stability(self,
                         time_data: np.ndarray,
                         response_data: np.ndarray,
                         control_data: Optional[np.ndarray] = None) -> AnalysisResult:
        """
        Analyze system stability characteristics.
        
        Args:
            time_data: Time vector
            response_data: System response
            control_data: Control signal (optional)
            
        Returns:
            Analysis result with stability metrics
        """
        self.logger.info("Analyzing system stability")
        
        metrics = PerformanceMetrics()
        
        # Oscillation analysis
        oscillation_freq = self._detect_oscillation_frequency(response_data)
        metrics.oscillation_frequency = oscillation_freq
        
        # Damping ratio estimation
        damping_ratio = self._estimate_damping_ratio(response_data)
        metrics.damping_ratio = damping_ratio
        
        # Stability margin (simplified)
        response_variance = np.var(response_data)
        metrics.stability_margin = 1.0 / (1.0 + response_variance)
        
        # Control signal analysis
        if control_data is not None:
            control_metrics = self._calculate_control_metrics(control_data)
            metrics.control_variance = control_metrics['control_variance']
            metrics.max_control = control_metrics['max_control']
            
        # Generate recommendations
        recommendations = self._generate_stability_recommendations(metrics)
        
        # Create plots
        plots = []
        if self.enable_plots:
            plot_path = self._plot_stability_analysis(time_data, response_data, control_data)
            if plot_path:
                plots.append(plot_path)
                
        result = AnalysisResult(
            analysis_type=AnalysisType.STABILITY_ANALYSIS,
            metrics=metrics,
            data={
                'time': time_data.tolist(),
                'response': response_data.tolist(),
                'control': control_data.tolist() if control_data is not None else None
            },
            recommendations=recommendations,
            plots=plots
        )
        
        self.analysis_history.append(result)
        return result
        
    def compare_configurations(self,
                             config1_data: Dict[str, np.ndarray],
                             config2_data: Dict[str, np.ndarray],
                             config1_name: str = "Configuration 1",
                             config2_name: str = "Configuration 2") -> AnalysisResult:
        """
        Compare performance between two configurations.
        
        Args:
            config1_data: First configuration data
            config2_data: Second configuration data
            config1_name: Name of first configuration
            config2_name: Name of second configuration
            
        Returns:
            Comparison analysis result
        """
        self.logger.info(f"Comparing configurations: {config1_name} vs {config2_name}")
        
        # Analyze both configurations
        result1 = self.analyze_step_response(
            config1_data['time'],
            config1_data['setpoint'],
            config1_data['response'],
            config1_data.get('control')
        )
        
        result2 = self.analyze_step_response(
            config2_data['time'],
            config2_data['setpoint'],
            config2_data['response'],
            config2_data.get('control')
        )
        
        # Compare metrics
        comparison_metrics = self._compare_metrics(result1.metrics, result2.metrics)
        
        # Generate comparison recommendations
        recommendations = self._generate_comparison_recommendations(
            result1.metrics, result2.metrics, config1_name, config2_name
        )
        
        # Create comparison plots
        plots = []
        if self.enable_plots:
            plot_path = self._plot_comparison(
                config1_data, config2_data, config1_name, config2_name
            )
            if plot_path:
                plots.append(plot_path)
                
        result = AnalysisResult(
            analysis_type=AnalysisType.PERFORMANCE_COMPARISON,
            metrics=comparison_metrics,
            data={
                'config1': result1.data,
                'config2': result2.data,
                'config1_name': config1_name,
                'config2_name': config2_name
            },
            recommendations=recommendations,
            plots=plots
        )
        
        self.analysis_history.append(result)
        return result
        
    def _calculate_time_domain_metrics(self,
                                     time_data: np.ndarray,
                                     setpoint_data: np.ndarray,
                                     response_data: np.ndarray) -> PerformanceMetrics:
        """Calculate time domain performance metrics"""
        metrics = PerformanceMetrics()
        
        # Find step location
        step_idx = np.where(np.diff(setpoint_data) != 0)[0]
        if len(step_idx) == 0:
            step_idx = [0]
        step_start = step_idx[0]
        
        # Final value
        final_value = np.mean(setpoint_data[-100:]) if len(setpoint_data) > 100 else setpoint_data[-1]
        initial_value = np.mean(response_data[:step_start+1]) if step_start > 0 else response_data[0]
        
        # Step response portion
        step_response = response_data[step_start:]
        step_time = time_data[step_start:]
        
        if len(step_response) < 10:
            return metrics
            
        # Rise time (10% to 90% of final value)
        value_range = final_value - initial_value
        if abs(value_range) > 1e-6:
            target_10 = initial_value + 0.1 * value_range
            target_90 = initial_value + 0.9 * value_range
            
            idx_10 = np.where(step_response >= target_10)[0]
            idx_90 = np.where(step_response >= target_90)[0]
            
            if len(idx_10) > 0 and len(idx_90) > 0:
                metrics.rise_time = step_time[idx_90[0]] - step_time[idx_10[0]]
                
        # Settling time (within 2% of final value)
        tolerance = 0.02 * abs(final_value)
        settled_indices = np.where(np.abs(step_response - final_value) <= tolerance)[0]
        
        if len(settled_indices) > 0:
            # Find first index where it stays settled
            for i in range(len(settled_indices) - 10):
                if np.all(np.abs(step_response[settled_indices[i]:settled_indices[i]+10] - final_value) <= tolerance):
                    metrics.settling_time = step_time[settled_indices[i]] - step_time[0]
                    break
                    
        # Overshoot and undershoot
        if abs(value_range) > 1e-6:
            max_response = np.max(step_response)
            min_response = np.min(step_response)
            
            if value_range > 0:  # Positive step
                metrics.overshoot = max(0, (max_response - final_value) / value_range * 100)
                metrics.undershoot = max(0, (initial_value - min_response) / value_range * 100)
            else:  # Negative step
                metrics.overshoot = max(0, (final_value - min_response) / abs(value_range) * 100)
                metrics.undershoot = max(0, (max_response - initial_value) / abs(value_range) * 100)
                
        # Steady-state error
        metrics.steady_state_error = abs(final_value - np.mean(step_response[-50:]))
        
        # Statistical metrics
        error_signal = setpoint_data - response_data
        metrics.rms_error = np.sqrt(np.mean(error_signal**2))
        metrics.max_error = np.max(np.abs(error_signal))
        metrics.mean_error = np.mean(error_signal)
        metrics.error_variance = np.var(error_signal)
        
        return metrics
        
    def _calculate_control_metrics(self, control_data: np.ndarray) -> Dict[str, float]:
        """Calculate control effort metrics"""
        return {
            'control_effort': np.sum(np.abs(control_data)) * self.dt,
            'max_control': np.max(np.abs(control_data)),
            'control_variance': np.var(control_data)
        }
        
    def _detect_oscillation_frequency(self, data: np.ndarray) -> float:
        """Detect dominant oscillation frequency using FFT"""
        if len(data) < 50:
            return 0.0
            
        # Remove DC component
        data_centered = data - np.mean(data)
        
        # Apply window
        window = np.hanning(len(data_centered))
        windowed_data = data_centered * window
        
        # FFT
        fft_result = np.fft.fft(windowed_data)
        frequencies = np.fft.fftfreq(len(windowed_data), self.dt)
        
        # Find peak frequency (excluding DC)
        magnitude = np.abs(fft_result[:len(fft_result)//2])
        positive_freqs = frequencies[:len(frequencies)//2]
        
        if len(magnitude) > 1:
            peak_idx = np.argmax(magnitude[1:]) + 1  # Skip DC
            return positive_freqs[peak_idx]
        else:
            return 0.0
            
    def _estimate_damping_ratio(self, response_data: np.ndarray) -> float:
        """Estimate damping ratio from response"""
        # Find peaks for oscillation analysis
        peaks, _ = signal.find_peaks(response_data, height=np.mean(response_data))
        
        if len(peaks) < 2:
            return 1.0  # Overdamped
            
        # Calculate logarithmic decrement
        peak_values = response_data[peaks]
        if len(peak_values) >= 2:
            log_dec = np.log(peak_values[0] / peak_values[-1]) / (len(peak_values) - 1)
            damping_ratio = log_dec / np.sqrt(4 * np.pi**2 + log_dec**2)
            return np.clip(damping_ratio, 0.0, 1.0)
        else:
            return 1.0
            
    def _compare_metrics(self, metrics1: PerformanceMetrics, metrics2: PerformanceMetrics) -> PerformanceMetrics:
        """Compare two sets of metrics"""
        # Create comparison metrics (percentage improvements)
        comparison = PerformanceMetrics()
        
        # Calculate percentage improvements (negative means config2 is worse)
        if metrics1.rise_time > 0:
            comparison.rise_time = (metrics1.rise_time - metrics2.rise_time) / metrics1.rise_time * 100
        if metrics1.settling_time > 0:
            comparison.settling_time = (metrics1.settling_time - metrics2.settling_time) / metrics1.settling_time * 100
        if metrics1.overshoot > 0:
            comparison.overshoot = (metrics1.overshoot - metrics2.overshoot) / metrics1.overshoot * 100
        if metrics1.rms_error > 0:
            comparison.rms_error = (metrics1.rms_error - metrics2.rms_error) / metrics1.rms_error * 100
        if metrics1.control_effort > 0:
            comparison.control_effort = (metrics1.control_effort - metrics2.control_effort) / metrics1.control_effort * 100
            
        return comparison
        
    def _generate_step_response_recommendations(self, metrics: PerformanceMetrics) -> List[str]:
        """Generate recommendations based on step response analysis"""
        recommendations = []
        
        # Rise time recommendations
        if metrics.rise_time > 2.0:
            recommendations.append("Rise time is slow. Consider increasing P gain.")
        elif metrics.rise_time < 0.1:
            recommendations.append("Rise time is very fast. Consider reducing P gain to avoid overshoot.")
            
        # Settling time recommendations
        if metrics.settling_time > 5.0:
            recommendations.append("Settling time is too long. Increase P gain or add D gain.")
        elif metrics.settling_time < 0.5:
            recommendations.append("Very fast settling. Good performance.")
            
        # Overshoot recommendations
        if metrics.overshoot > 20.0:
            recommendations.append("Excessive overshoot. Reduce P gain or increase D gain.")
        elif metrics.overshoot > 10.0:
            recommendations.append("Moderate overshoot. Consider increasing D gain.")
        elif metrics.overshoot < 2.0:
            recommendations.append("Low overshoot. Good damping characteristics.")
            
        # Steady-state error recommendations
        if metrics.steady_state_error > 0.1:
            recommendations.append("High steady-state error. Increase I gain.")
        elif metrics.steady_state_error < 0.01:
            recommendations.append("Excellent steady-state accuracy.")
            
        # Control effort recommendations
        if metrics.control_effort > 100.0:
            recommendations.append("High control effort. Consider reducing gains.")
        elif metrics.max_control > 0.95:
            recommendations.append("Control saturation detected. Reduce gains or increase limits.")
            
        return recommendations
        
    def _generate_frequency_response_recommendations(self, metrics: PerformanceMetrics) -> List[str]:
        """Generate recommendations based on frequency response analysis"""
        recommendations = []
        
        # Bandwidth recommendations
        if metrics.bandwidth < 5.0:
            recommendations.append("Low bandwidth. System may be slow to respond.")
        elif metrics.bandwidth > 50.0:
            recommendations.append("High bandwidth. May be sensitive to noise.")
            
        # Phase margin recommendations
        if metrics.phase_margin < 30.0:
            recommendations.append("Low phase margin. System may be unstable or oscillatory.")
        elif metrics.phase_margin > 60.0:
            recommendations.append("High phase margin. Good stability margins.")
            
        # Gain margin recommendations
        if metrics.gain_margin < 6.0:
            recommendations.append("Low gain margin. Reduce P gain for better stability.")
        elif metrics.gain_margin > 20.0:
            recommendations.append("High gain margin. Could increase P gain for better performance.")
            
        return recommendations
        
    def _generate_stability_recommendations(self, metrics: PerformanceMetrics) -> List[str]:
        """Generate recommendations based on stability analysis"""
        recommendations = []
        
        # Oscillation frequency recommendations
        if metrics.oscillation_frequency > 10.0:
            recommendations.append("High frequency oscillations detected. Reduce P gain or add filtering.")
        elif metrics.oscillation_frequency > 2.0:
            recommendations.append("Moderate oscillations. Consider adjusting PID gains.")
            
        # Damping ratio recommendations
        if metrics.damping_ratio < 0.3:
            recommendations.append("Underdamped system. Increase D gain or reduce P gain.")
        elif metrics.damping_ratio > 0.8:
            recommendations.append("Overdamped system. Could increase P gain for faster response.")
        elif 0.6 <= metrics.damping_ratio <= 0.8:
            recommendations.append("Good damping ratio. Optimal performance region.")
            
        return recommendations
        
    def _generate_comparison_recommendations(self,
                                           metrics1: PerformanceMetrics,
                                           metrics2: PerformanceMetrics,
                                           config1_name: str,
                                           config2_name: str) -> List[str]:
        """Generate recommendations based on configuration comparison"""
        recommendations = []
        
        # Rise time comparison
        if metrics2.rise_time < metrics1.rise_time * 0.8:
            recommendations.append(f"{config2_name} has significantly faster rise time.")
        elif metrics2.rise_time > metrics1.rise_time * 1.2:
            recommendations.append(f"{config1_name} has significantly faster rise time.")
            
        # Overshoot comparison
        if metrics2.overshoot < metrics1.overshoot * 0.8:
            recommendations.append(f"{config2_name} has better overshoot characteristics.")
        elif metrics2.overshoot > metrics1.overshoot * 1.2:
            recommendations.append(f"{config1_name} has better overshoot characteristics.")
            
        # Overall recommendation
        score1 = self._calculate_performance_score(metrics1)
        score2 = self._calculate_performance_score(metrics2)
        
        if score2 > score1 * 1.1:
            recommendations.append(f"Overall, {config2_name} shows better performance.")
        elif score1 > score2 * 1.1:
            recommendations.append(f"Overall, {config1_name} shows better performance.")
        else:
            recommendations.append("Both configurations show similar performance.")
            
        return recommendations
        
    def _calculate_performance_score(self, metrics: PerformanceMetrics) -> float:
        """Calculate overall performance score"""
        score = 0.0
        
        # Penalize slow rise time
        if metrics.rise_time > 0:
            score -= metrics.rise_time * 2
            
        # Penalize slow settling time
        if metrics.settling_time > 0:
            score -= metrics.settling_time
            
        # Penalize overshoot
        score -= metrics.overshoot * 0.5
        
        # Penalize steady-state error
        score -= metrics.steady_state_error * 10
        
        # Penalize high control effort
        if metrics.control_effort > 0:
            score -= metrics.control_effort * 0.01
            
        return score
        
    def _plot_step_response(self,
                           time_data: np.ndarray,
                           setpoint_data: np.ndarray,
                           response_data: np.ndarray,
                           control_data: Optional[np.ndarray] = None) -> Optional[str]:
        """Create step response plot"""
        try:
            fig, axes = plt.subplots(2, 1, figsize=(10, 8))
            
            # Response plot
            axes[0].plot(time_data, setpoint_data, 'r--', label='Setpoint', linewidth=2)
            axes[0].plot(time_data, response_data, 'b-', label='Response', linewidth=2)
            axes[0].set_xlabel('Time (s)')
            axes[0].set_ylabel('Response')
            axes[0].set_title('Step Response Analysis')
            axes[0].grid(True, alpha=0.3)
            axes[0].legend()
            
            # Control signal plot
            if control_data is not None:
                axes[1].plot(time_data, control_data, 'g-', label='Control Signal', linewidth=2)
                axes[1].set_xlabel('Time (s)')
                axes[1].set_ylabel('Control Output')
                axes[1].set_title('Control Signal')
                axes[1].grid(True, alpha=0.3)
                axes[1].legend()
            else:
                axes[1].axis('off')
                
            plt.tight_layout()
            
            # Save plot
            timestamp = int(time.time())
            plot_path = self.plot_directory / f"step_response_{timestamp}.png"
            plt.savefig(plot_path, dpi=300, bbox_inches='tight')
            plt.close()
            
            return str(plot_path)
            
        except Exception as e:
            self.logger.error(f"Failed to create step response plot: {e}")
            return None
            
    def _plot_frequency_response(self,
                               frequencies: np.ndarray,
                               magnitude_db: np.ndarray,
                               phase: np.ndarray) -> Optional[str]:
        """Create frequency response plot"""
        try:
            fig, axes = plt.subplots(2, 1, figsize=(10, 8))
            
            # Magnitude plot
            axes[0].semilogx(frequencies, magnitude_db, 'b-', linewidth=2)
            axes[0].set_xlabel('Frequency (Hz)')
            axes[0].set_ylabel('Magnitude (dB)')
            axes[0].set_title('Frequency Response - Magnitude')
            axes[0].grid(True, alpha=0.3)
            axes[0].axhline(-3, color='r', linestyle='--', alpha=0.7, label='-3dB')
            axes[0].legend()
            
            # Phase plot
            axes[1].semilogx(frequencies, phase, 'r-', linewidth=2)
            axes[1].set_xlabel('Frequency (Hz)')
            axes[1].set_ylabel('Phase (degrees)')
            axes[1].set_title('Frequency Response - Phase')
            axes[1].grid(True, alpha=0.3)
            axes[1].axhline(-180, color='r', linestyle='--', alpha=0.7, label='-180°')
            axes[1].legend()
            
            plt.tight_layout()
            
            # Save plot
            timestamp = int(time.time())
            plot_path = self.plot_directory / f"frequency_response_{timestamp}.png"
            plt.savefig(plot_path, dpi=300, bbox_inches='tight')
            plt.close()
            
            return str(plot_path)
            
        except Exception as e:
            self.logger.error(f"Failed to create frequency response plot: {e}")
            return None
            
    def _plot_stability_analysis(self,
                               time_data: np.ndarray,
                               response_data: np.ndarray,
                               control_data: Optional[np.ndarray] = None) -> Optional[str]:
        """Create stability analysis plot"""
        try:
            fig, axes = plt.subplots(2, 2, figsize=(12, 8))
            
            # Time series plot
            axes[0, 0].plot(time_data, response_data, 'b-', linewidth=2)
            axes[0, 0].set_xlabel('Time (s)')
            axes[0, 0].set_ylabel('Response')
            axes[0, 0].set_title('Response Time Series')
            axes[0, 0].grid(True, alpha=0.3)
            
            # FFT plot
            fft_result = np.fft.fft(response_data - np.mean(response_data))
            frequencies = np.fft.fftfreq(len(response_data), self.dt)
            magnitude = np.abs(fft_result[:len(fft_result)//2])
            pos_freqs = frequencies[:len(frequencies)//2]
            
            axes[0, 1].semilogy(pos_freqs, magnitude, 'r-', linewidth=2)
            axes[0, 1].set_xlabel('Frequency (Hz)')
            axes[0, 1].set_ylabel('Magnitude')
            axes[0, 1].set_title('Frequency Content')
            axes[0, 1].grid(True, alpha=0.3)
            
            # Phase portrait (if control data available)
            if control_data is not None:
                axes[1, 0].plot(response_data, control_data, 'g-', linewidth=2)
                axes[1, 0].set_xlabel('Response')
                axes[1, 0].set_ylabel('Control')
                axes[1, 0].set_title('Phase Portrait')
                axes[1, 0].grid(True, alpha=0.3)
                
                # Control signal
                axes[1, 1].plot(time_data, control_data, 'g-', linewidth=2)
                axes[1, 1].set_xlabel('Time (s)')
                axes[1, 1].set_ylabel('Control')
                axes[1, 1].set_title('Control Signal')
                axes[1, 1].grid(True, alpha=0.3)
            else:
                axes[1, 0].axis('off')
                axes[1, 1].axis('off')
                
            plt.tight_layout()
            
            # Save plot
            timestamp = int(time.time())
            plot_path = self.plot_directory / f"stability_analysis_{timestamp}.png"
            plt.savefig(plot_path, dpi=300, bbox_inches='tight')
            plt.close()
            
            return str(plot_path)
            
        except Exception as e:
            self.logger.error(f"Failed to create stability analysis plot: {e}")
            return None
            
    def _plot_comparison(self,
                        config1_data: Dict[str, np.ndarray],
                        config2_data: Dict[str, np.ndarray],
                        config1_name: str,
                        config2_name: str) -> Optional[str]:
        """Create comparison plot"""
        try:
            fig, axes = plt.subplots(2, 1, figsize=(12, 8))
            
            # Response comparison
            axes[0].plot(config1_data['time'], config1_data['setpoint'], 'k--', 
                        label='Setpoint', linewidth=2)
            axes[0].plot(config1_data['time'], config1_data['response'], 'b-', 
                        label=config1_name, linewidth=2)
            axes[0].plot(config2_data['time'], config2_data['response'], 'r-', 
                        label=config2_name, linewidth=2)
            axes[0].set_xlabel('Time (s)')
            axes[0].set_ylabel('Response')
            axes[0].set_title('Response Comparison')
            axes[0].grid(True, alpha=0.3)
            axes[0].legend()
            
            # Control comparison
            if 'control' in config1_data and 'control' in config2_data:
                axes[1].plot(config1_data['time'], config1_data['control'], 'b-', 
                            label=config1_name, linewidth=2)
                axes[1].plot(config2_data['time'], config2_data['control'], 'r-', 
                            label=config2_name, linewidth=2)
                axes[1].set_xlabel('Time (s)')
                axes[1].set_ylabel('Control Output')
                axes[1].set_title('Control Signal Comparison')
                axes[1].grid(True, alpha=0.3)
                axes[1].legend()
            else:
                axes[1].axis('off')
                
            plt.tight_layout()
            
            # Save plot
            timestamp = int(time.time())
            plot_path = self.plot_directory / f"comparison_{timestamp}.png"
            plt.savefig(plot_path, dpi=300, bbox_inches='tight')
            plt.close()
            
            return str(plot_path)
            
        except Exception as e:
            self.logger.error(f"Failed to create comparison plot: {e}")
            return None
            
    def generate_report(self, output_path: str = None) -> str:
        """Generate comprehensive analysis report"""
        if not self.analysis_history:
            return "No analysis data available for report generation."
            
        # Create HTML report
        report_html = """
        <!DOCTYPE html>
        <html>
        <head>
            <title>Tuning Analysis Report</title>
            <style>
                body { font-family: Arial, sans-serif; margin: 20px; }
                h1, h2 { color: #2c3e50; }
                table { border-collapse: collapse; width: 100%; }
                th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }
                th { background-color: #f2f2f2; }
                .metric { margin: 10px 0; }
                .recommendation { background-color: #e8f5e8; padding: 10px; margin: 5px 0; border-radius: 5px; }
                .plot { margin: 20px 0; text-align: center; }
            </style>
        </head>
        <body>
            <h1>Tuning Analysis Report</h1>
            <p>Generated: {timestamp}</p>
        """.format(timestamp=time.strftime("%Y-%m-%d %H:%M:%S"))
        
        # Add analysis results
        for i, result in enumerate(self.analysis_history):
            report_html += f"""
            <h2>Analysis {i+1}: {result.analysis_type.value.replace('_', ' ').title()}</h2>
            <div class="metric">
                <h3>Performance Metrics</h3>
                <table>
                    <tr><th>Metric</th><th>Value</th></tr>
            """
            
            # Add metrics to table
            metrics_dict = result.metrics.to_dict()
            for category, values in metrics_dict.items():
                for metric, value in values.items():
                    if isinstance(value, float):
                        report_html += f"<tr><td>{metric.replace('_', ' ').title()}</td><td>{value:.4f}</td></tr>"
                        
            report_html += """
                </table>
            </div>
            """
            
            # Add recommendations
            if result.recommendations:
                report_html += "<h3>Recommendations</h3>"
                for rec in result.recommendations:
                    report_html += f'<div class="recommendation">{rec}</div>'
                    
            # Add plots
            if result.plots:
                report_html += "<h3>Plots</h3>"
                for plot_path in result.plots:
                    report_html += f'<div class="plot"><img src="{plot_path}" style="max-width: 800px;"></div>'
                    
        report_html += """
        </body>
        </html>
        """
        
        # Save report
        if output_path is None:
            output_path = f"tuning_report_{int(time.time())}.html"
            
        try:
            with open(output_path, 'w') as f:
                f.write(report_html)
            self.logger.info(f"Report generated: {output_path}")
            return output_path
        except Exception as e:
            self.logger.error(f"Failed to generate report: {e}")
            return ""
            
    def clear_history(self):
        """Clear analysis history"""
        self.analysis_history.clear()
        self.logger.info("Analysis history cleared")
        
    def get_summary(self) -> Dict[str, Any]:
        """Get summary of analysis results"""
        if not self.analysis_history:
            return {'error': 'No analysis data available'}
            
        summary = {
            'total_analyses': len(self.analysis_history),
            'analysis_types': [result.analysis_type.value for result in self.analysis_history],
            'latest_analysis': self.analysis_history[-1].to_dict() if self.analysis_history else None
        }
        
        return summary
