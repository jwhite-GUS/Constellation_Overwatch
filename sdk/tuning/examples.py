"""
Constellation Overwatch SDK - Tuning Examples
Complete examples and workflows for autopilot tuning.
"""

import asyncio
import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, List, Optional
import logging

from .tuning_integrator import TuningIntegrator, IntegrationMode, TuningPhase
from .parameter_manager import ParameterManager, ParameterSet
from .tuning_analyzer import TuningAnalyzer, AnalysisType
from .pid_controller import PIDController, PIDGains
from .safety_monitor import SafetyMonitor, SafetyLevel

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class TuningExamples:
    """
    Complete examples and workflows for autopilot tuning.
    
    This class provides practical examples of how to use the tuning framework
    for different scenarios and vehicle types.
    """
    
    def __init__(self):
        self.integrator = TuningIntegrator()
        self.parameter_manager = ParameterManager()
        self.analyzer = TuningAnalyzer()
        self.safety_monitor = SafetyMonitor()
        
    async def example_basic_pid_tuning(self) -> str:
        """
        Basic PID tuning example for a quadcopter.
        
        This example shows how to:
        1. Set up a basic tuning session
        2. Define tuning objectives
        3. Run automated tuning
        4. Analyze results
        
        Returns:
            Session ID for tracking
        """
        logger.info("Starting basic PID tuning example")
        
        # Define vehicle configuration
        vehicle_config = {
            'vehicle_type': 'quadcopter',
            'mass': 1.5,  # kg
            'inertia': [0.02, 0.02, 0.04],  # kg⋅m²
            'motor_time_constant': 0.1,  # seconds
            'max_tilt_angle': 30,  # degrees
            'max_climb_rate': 5.0,  # m/s
        }
        
        # Define tuning objectives
        tuning_objectives = {
            'target_bandwidth': 10.0,  # Hz
            'max_overshoot': 15.0,  # %
            'settling_time': 2.0,  # seconds
            'phase_margin': 45.0,  # degrees
        }
        
        # Start tuning session
        session_id = await self.integrator.start_tuning_session(
            vehicle_config=vehicle_config,
            tuning_objectives=tuning_objectives,
            mode=IntegrationMode.SIMULATION_ONLY
        )
        
        logger.info(f"Basic PID tuning session started: {session_id}")
        return session_id
        
    async def example_advanced_multirotor_tuning(self) -> str:
        """
        Advanced multirotor tuning with comprehensive analysis.
        
        This example demonstrates:
        1. Multi-axis tuning coordination
        2. Disturbance rejection optimization
        3. Noise filtering integration
        4. Performance validation
        
        Returns:
            Session ID for tracking
        """
        logger.info("Starting advanced multirotor tuning example")
        
        # Advanced vehicle configuration
        vehicle_config = {
            'vehicle_type': 'hexacopter',
            'mass': 3.5,  # kg
            'inertia': [0.15, 0.15, 0.25],  # kg⋅m²
            'motor_time_constant': 0.08,
            'propeller_diameter': 0.254,  # meters
            'battery_cells': 6,
            'max_tilt_angle': 25,
            'max_climb_rate': 8.0,
            'has_gimbal': True,
            'payload_mass': 0.5,  # kg
        }
        
        # Comprehensive tuning objectives
        tuning_objectives = {
            'target_bandwidth': 15.0,
            'max_overshoot': 10.0,
            'settling_time': 1.5,
            'phase_margin': 50.0,
            'gain_margin': 10.0,
            'disturbance_rejection': 0.8,
            'noise_sensitivity': 0.2,
            'control_effort_limit': 0.8,
        }
        
        # Start advanced tuning session
        session_id = await self.integrator.start_tuning_session(
            vehicle_config=vehicle_config,
            tuning_objectives=tuning_objectives,
            mode=IntegrationMode.HYBRID
        )
        
        logger.info(f"Advanced multirotor tuning session started: {session_id}")
        return session_id
        
    async def example_fixed_wing_tuning(self) -> str:
        """
        Fixed-wing aircraft tuning example.
        
        This example shows:
        1. Longitudinal and lateral axis tuning
        2. Airspeed and altitude control
        3. Turn coordination
        4. Stall prevention
        
        Returns:
            Session ID for tracking
        """
        logger.info("Starting fixed-wing tuning example")
        
        # Fixed-wing configuration
        vehicle_config = {
            'vehicle_type': 'fixed_wing',
            'mass': 2.0,  # kg
            'wing_span': 1.5,  # meters
            'wing_area': 0.3,  # m²
            'cruise_speed': 15.0,  # m/s
            'stall_speed': 8.0,  # m/s
            'max_bank_angle': 60,  # degrees
            'service_ceiling': 200,  # meters
        }
        
        # Fixed-wing specific objectives
        tuning_objectives = {
            'airspeed_tracking': 0.5,  # m/s tolerance
            'altitude_tracking': 2.0,  # m tolerance
            'heading_tracking': 5.0,  # degrees tolerance
            'turn_coordination': 0.9,  # efficiency
            'spiral_stability': True,
            'phugoid_damping': 0.7,
        }
        
        session_id = await self.integrator.start_tuning_session(
            vehicle_config=vehicle_config,
            tuning_objectives=tuning_objectives,
            mode=IntegrationMode.SIMULATION_ONLY
        )
        
        logger.info(f"Fixed-wing tuning session started: {session_id}")
        return session_id
        
    async def example_parameter_comparison(self) -> Dict[str, float]:
        """
        Example of comparing different parameter sets.
        
        This demonstrates:
        1. Setting up multiple parameter configurations
        2. Running comparative analysis
        3. Generating performance metrics
        4. Selecting optimal parameters
        
        Returns:
            Comparison results
        """
        logger.info("Starting parameter comparison example")
        
        # Create two parameter sets to compare
        conservative_params = ParameterSet(
            name="conservative",
            description="Conservative tuning parameters"
        )
        conservative_params.add_parameter('pid_roll_p', 0.3, 0.1, 1.0, "Conservative P gain")
        conservative_params.add_parameter('pid_roll_i', 0.05, 0.0, 0.5, "Conservative I gain")
        conservative_params.add_parameter('pid_roll_d', 0.02, 0.0, 0.2, "Conservative D gain")
        
        aggressive_params = ParameterSet(
            name="aggressive",
            description="Aggressive tuning parameters"
        )
        aggressive_params.add_parameter('pid_roll_p', 0.8, 0.1, 1.0, "Aggressive P gain")
        aggressive_params.add_parameter('pid_roll_i', 0.2, 0.0, 0.5, "Aggressive I gain")
        aggressive_params.add_parameter('pid_roll_d', 0.1, 0.0, 0.2, "Aggressive D gain")
        
        # Generate test data for both configurations
        duration = 10.0
        sample_rate = 100.0
        dt = 1.0 / sample_rate
        time_data = np.arange(0, duration, dt)
        
        # Step input at t=1s
        setpoint = np.zeros_like(time_data)
        setpoint[time_data >= 1.0] = 30.0  # 30 degree roll command
        
        # Simulate both configurations
        conservative_data = await self._simulate_response(
            conservative_params, time_data, setpoint, sample_rate
        )
        
        aggressive_data = await self._simulate_response(
            aggressive_params, time_data, setpoint, sample_rate
        )
        
        # Compare configurations
        comparison_result = self.analyzer.compare_configurations(
            conservative_data, aggressive_data,
            "Conservative", "Aggressive"
        )
        
        logger.info("Parameter comparison completed")
        return {
            'conservative_score': self._calculate_score(conservative_data),
            'aggressive_score': self._calculate_score(aggressive_data),
            'comparison_metrics': comparison_result.metrics.to_dict()
        }
        
    async def example_safety_monitoring(self) -> Dict[str, bool]:
        """
        Example of safety monitoring during tuning.
        
        This demonstrates:
        1. Real-time safety monitoring
        2. Oscillation detection
        3. Automatic safety interventions
        4. Parameter rollback
        
        Returns:
            Safety monitoring results
        """
        logger.info("Starting safety monitoring example")
        
        # Configure safety monitor
        self.safety_monitor.configure_limits(
            max_roll_rate=120.0,  # deg/s
            max_pitch_rate=120.0,  # deg/s
            max_yaw_rate=90.0,  # deg/s
            max_acceleration=2.0,  # g
            oscillation_threshold=0.5,
            safety_timeout=5.0
        )
        
        # Create unstable parameters (for demonstration)
        unstable_params = ParameterSet(
            name="unstable",
            description="Intentionally unstable parameters"
        )
        unstable_params.add_parameter('pid_roll_p', 2.0, 0.1, 3.0, "High P gain")
        unstable_params.add_parameter('pid_roll_i', 0.5, 0.0, 1.0, "High I gain")
        unstable_params.add_parameter('pid_roll_d', 0.0, 0.0, 0.2, "No D gain")
        
        # Simulate unstable response
        duration = 5.0
        sample_rate = 100.0
        dt = 1.0 / sample_rate
        time_data = np.arange(0, duration, dt)
        setpoint = np.ones_like(time_data) * 10.0  # 10 degree step
        
        # Create PID controller with unstable parameters
        pid_gains = PIDGains(
            kp=unstable_params.get_parameter('pid_roll_p'),
            ki=unstable_params.get_parameter('pid_roll_i'),
            kd=unstable_params.get_parameter('pid_roll_d')
        )
        controller = PIDController(pid_gains, sample_rate)
        
        # Simulate with safety monitoring
        response = np.zeros_like(time_data)
        control = np.zeros_like(time_data)
        safety_violations = []
        
        for i in range(len(time_data)):
            if i > 0:
                # Simple unstable system
                response[i] = response[i-1] + control[i-1] * dt * 2.0
                
            control[i] = controller.update(setpoint[i], response[i], dt)
            
            # Safety monitoring
            self.safety_monitor.update_state({
                'roll_angle': response[i],
                'roll_rate': (response[i] - response[i-1]) / dt if i > 0 else 0.0,
                'control_output': control[i]
            })
            
            # Check for violations
            violations = self.safety_monitor.check_violations()
            if violations:
                safety_violations.extend(violations)
                logger.warning(f"Safety violation at t={time_data[i]:.2f}s: {violations}")
                
                # Trigger safety intervention
                if self.safety_monitor.safety_level == SafetyLevel.CRITICAL:
                    logger.error("Critical safety violation - emergency stop")
                    break
                    
        results = {
            'oscillation_detected': self.safety_monitor.oscillation_detector.is_oscillating(),
            'safety_violations': len(safety_violations) > 0,
            'max_response': np.max(np.abs(response)),
            'max_control': np.max(np.abs(control)),
            'safety_level': self.safety_monitor.safety_level.value
        }
        
        logger.info(f"Safety monitoring completed: {results}")
        return results
        
    async def example_frequency_analysis(self) -> Dict[str, np.ndarray]:
        """
        Example of frequency domain analysis.
        
        This demonstrates:
        1. Frequency sweep testing
        2. Bode plot generation
        3. Stability margin calculation
        4. Bandwidth optimization
        
        Returns:
            Frequency analysis results
        """
        logger.info("Starting frequency analysis example")
        
        # Create PID controller for analysis
        pid_gains = PIDGains(kp=0.5, ki=0.1, kd=0.05)
        controller = PIDController(pid_gains, 100.0)
        
        # Generate frequency sweep
        frequencies = np.logspace(-1, 2, 100)  # 0.1 to 100 Hz
        magnitude = np.zeros_like(frequencies)
        phase = np.zeros_like(frequencies)
        
        for i, freq in enumerate(frequencies):
            # Calculate frequency response
            omega = 2 * np.pi * freq
            s = 1j * omega
            
            # PID transfer function: K_p + K_i/s + K_d*s
            pid_tf = pid_gains.kp + pid_gains.ki / s + pid_gains.kd * s
            
            # Simple plant model: 1/(s^2 + 2*zeta*wn*s + wn^2)
            wn = 10.0  # Natural frequency
            zeta = 0.3  # Damping ratio
            plant_tf = wn**2 / (s**2 + 2*zeta*wn*s + wn**2)
            
            # Closed-loop transfer function
            open_loop = pid_tf * plant_tf
            closed_loop = open_loop / (1 + open_loop)
            
            magnitude[i] = abs(closed_loop)
            phase[i] = np.angle(closed_loop, deg=True)
            
        # Analyze frequency response
        analysis_result = self.analyzer.analyze_frequency_response(
            frequencies, magnitude, phase
        )
        
        logger.info("Frequency analysis completed")
        return {
            'frequencies': frequencies,
            'magnitude': magnitude,
            'phase': phase,
            'bandwidth': analysis_result.metrics.bandwidth,
            'phase_margin': analysis_result.metrics.phase_margin,
            'gain_margin': analysis_result.metrics.gain_margin
        }
        
    async def example_automated_workflow(self) -> str:
        """
        Complete automated tuning workflow example.
        
        This demonstrates:
        1. Automated session management
        2. Multi-phase tuning progression
        3. Real-time monitoring
        4. Result validation
        5. Report generation
        
        Returns:
            Final report path
        """
        logger.info("Starting automated workflow example")
        
        # Setup status monitoring
        status_updates = []
        
        async def status_callback(session_id: str, message: str, phase: TuningPhase):
            status_updates.append({
                'session_id': session_id,
                'message': message,
                'phase': phase.value,
                'timestamp': asyncio.get_event_loop().time()
            })
            logger.info(f"Status update: {message}")
            
        self.integrator.set_status_update_callback(status_callback)
        
        # Vehicle configuration
        vehicle_config = {
            'vehicle_type': 'quadcopter',
            'mass': 2.0,
            'inertia': [0.05, 0.05, 0.08],
            'motor_time_constant': 0.1,
            'max_tilt_angle': 35,
            'max_climb_rate': 6.0,
        }
        
        # Tuning objectives
        tuning_objectives = {
            'target_bandwidth': 12.0,
            'max_overshoot': 12.0,
            'settling_time': 1.8,
            'phase_margin': 48.0,
            'min_damping_ratio': 0.6,
        }
        
        # Start automated tuning session
        session_id = await self.integrator.start_tuning_session(
            vehicle_config=vehicle_config,
            tuning_objectives=tuning_objectives,
            mode=IntegrationMode.SIMULATION_ONLY
        )
        
        # Monitor session progress
        session_complete = False
        timeout = 60.0  # 60 second timeout
        start_time = asyncio.get_event_loop().time()
        
        while not session_complete and (asyncio.get_event_loop().time() - start_time) < timeout:
            await asyncio.sleep(1.0)
            
            session_status = self.integrator.get_session_status(session_id)
            if session_status and not session_status['is_active']:
                session_complete = True
                
        # Generate comprehensive report
        report_path = self.analyzer.generate_report(f"automated_tuning_report_{session_id}.html")
        
        logger.info(f"Automated workflow completed. Report: {report_path}")
        return report_path
        
    # Helper methods
    
    async def _simulate_response(self,
                                params: ParameterSet,
                                time_data: np.ndarray,
                                setpoint: np.ndarray,
                                sample_rate: float) -> Dict[str, np.ndarray]:
        """Simulate system response with given parameters"""
        # Create PID controller
        pid_gains = PIDGains(
            kp=params.get_parameter('pid_roll_p', 0.5),
            ki=params.get_parameter('pid_roll_i', 0.1),
            kd=params.get_parameter('pid_roll_d', 0.05)
        )
        controller = PIDController(pid_gains, sample_rate)
        
        # Simulate response
        response = np.zeros_like(time_data)
        control = np.zeros_like(time_data)
        dt = 1.0 / sample_rate
        
        for i in range(len(time_data)):
            if i > 0:
                # Second-order system approximation
                wn = 5.0  # Natural frequency
                zeta = 0.7  # Damping ratio
                
                # State-space simulation (simplified)
                acceleration = wn**2 * (control[i-1] - response[i-1]) - 2*zeta*wn*(response[i] - response[i-1])/dt
                velocity = (response[i-1] - response[i-2])/dt if i > 1 else 0.0
                velocity += acceleration * dt
                response[i] = response[i-1] + velocity * dt
                
            control[i] = controller.update(setpoint[i], response[i], dt)
            
        return {
            'time': time_data,
            'setpoint': setpoint,
            'response': response,
            'control': control
        }
        
    def _calculate_score(self, data: Dict[str, np.ndarray]) -> float:
        """Calculate performance score from response data"""
        error = data['setpoint'] - data['response']
        rms_error = np.sqrt(np.mean(error**2))
        max_error = np.max(np.abs(error))
        
        # Simple scoring function
        score = 100.0 / (1.0 + rms_error + 0.1 * max_error)
        return score
        
    def plot_comparison_results(self, 
                               data1: Dict[str, np.ndarray], 
                               data2: Dict[str, np.ndarray],
                               title1: str = "Configuration 1",
                               title2: str = "Configuration 2",
                               save_path: str = "comparison_plot.png"):
        """Create comparison plot of two configurations"""
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        
        # Response comparison
        ax1.plot(data1['time'], data1['setpoint'], 'k--', label='Setpoint', linewidth=2)
        ax1.plot(data1['time'], data1['response'], 'b-', label=title1, linewidth=2)
        ax1.plot(data2['time'], data2['response'], 'r-', label=title2, linewidth=2)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Response')
        ax1.set_title('Response Comparison')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        
        # Control comparison
        ax2.plot(data1['time'], data1['control'], 'b-', label=title1, linewidth=2)
        ax2.plot(data2['time'], data2['control'], 'r-', label=title2, linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Control Output')
        ax2.set_title('Control Signal Comparison')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info(f"Comparison plot saved: {save_path}")

# Example usage and demonstration
async def main():
    """Main demonstration of tuning examples"""
    print("Constellation Overwatch SDK - Tuning Examples")
    print("=" * 50)
    
    examples = TuningExamples()
    
    # Run basic PID tuning example
    print("\n1. Basic PID Tuning Example")
    print("-" * 30)
    basic_session = await examples.example_basic_pid_tuning()
    print(f"Session ID: {basic_session}")
    
    # Run parameter comparison example
    print("\n2. Parameter Comparison Example")
    print("-" * 30)
    comparison_results = await examples.example_parameter_comparison()
    print(f"Conservative Score: {comparison_results['conservative_score']:.2f}")
    print(f"Aggressive Score: {comparison_results['aggressive_score']:.2f}")
    
    # Run safety monitoring example
    print("\n3. Safety Monitoring Example")
    print("-" * 30)
    safety_results = await examples.example_safety_monitoring()
    print(f"Oscillation Detected: {safety_results['oscillation_detected']}")
    print(f"Safety Violations: {safety_results['safety_violations']}")
    print(f"Safety Level: {safety_results['safety_level']}")
    
    # Run frequency analysis example
    print("\n4. Frequency Analysis Example")
    print("-" * 30)
    freq_results = await examples.example_frequency_analysis()
    print(f"Bandwidth: {freq_results['bandwidth']:.2f} Hz")
    print(f"Phase Margin: {freq_results['phase_margin']:.1f} degrees")
    print(f"Gain Margin: {freq_results['gain_margin']:.1f} dB")
    
    print("\n5. Complete Automated Workflow")
    print("-" * 30)
    report_path = await examples.example_automated_workflow()
    print(f"Report generated: {report_path}")
    
    print("\nAll examples completed successfully!")

if __name__ == "__main__":
    asyncio.run(main())
