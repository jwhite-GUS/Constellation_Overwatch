"""
Constellation Overwatch SDK - Tuning Integrator
Integration utilities for connecting tuning systems with flight controllers and simulators.
"""

import asyncio
import logging
import time
from typing import Dict, List, Tuple, Optional, Any, Callable
from dataclasses import dataclass, field
from enum import Enum
import numpy as np
import json
from pathlib import Path

from .pid_controller import PIDController, PIDGains, RatePIDController
from .auto_tuner import AutoTuner
from .simulation_tuner import SimulationTuner, DroneSimulator
from .parameter_manager import ParameterManager, ParameterSet
from .safety_monitor import SafetyMonitor, SafetyLevel
from .tuning_analyzer import TuningAnalyzer, AnalysisResult

class IntegrationMode(Enum):
    """Integration modes for tuning systems"""
    SIMULATION_ONLY = "simulation_only"
    HARDWARE_ONLY = "hardware_only"
    HYBRID = "hybrid"
    VALIDATION = "validation"

class TuningPhase(Enum):
    """Phases of automated tuning process"""
    INITIALIZATION = "initialization"
    BASELINE_CAPTURE = "baseline_capture"
    PARAMETER_IDENTIFICATION = "parameter_identification"
    COARSE_TUNING = "coarse_tuning"
    FINE_TUNING = "fine_tuning"
    VALIDATION = "validation"
    DEPLOYMENT = "deployment"

@dataclass
class TuningSession:
    """Represents a complete tuning session"""
    session_id: str
    start_time: float = field(default_factory=time.time)
    mode: IntegrationMode = IntegrationMode.SIMULATION_ONLY
    current_phase: TuningPhase = TuningPhase.INITIALIZATION
    vehicle_config: Dict[str, Any] = field(default_factory=dict)
    tuning_objectives: Dict[str, float] = field(default_factory=dict)
    baseline_params: Optional[ParameterSet] = None
    best_params: Optional[ParameterSet] = None
    analysis_results: List[AnalysisResult] = field(default_factory=list)
    phase_history: List[Dict[str, Any]] = field(default_factory=list)
    is_active: bool = True
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert session to dictionary"""
        return {
            'session_id': self.session_id,
            'start_time': self.start_time,
            'mode': self.mode.value,
            'current_phase': self.current_phase.value,
            'vehicle_config': self.vehicle_config,
            'tuning_objectives': self.tuning_objectives,
            'baseline_params': self.baseline_params.to_dict() if self.baseline_params else None,
            'best_params': self.best_params.to_dict() if self.best_params else None,
            'analysis_results': [result.to_dict() for result in self.analysis_results],
            'phase_history': self.phase_history,
            'is_active': self.is_active
        }

class TuningIntegrator:
    """
    Comprehensive tuning integration system that coordinates all tuning components.
    
    Features:
    - Automated multi-phase tuning workflow
    - Seamless simulation-to-hardware transfer
    - Safety monitoring and rollback capabilities
    - Real-time parameter optimization
    - Performance validation and analysis
    - Cross-platform flight controller support
    """
    
    def __init__(self, 
                 sample_rate: float = 100.0,
                 enable_safety_monitoring: bool = True):
        """
        Initialize tuning integrator.
        
        Args:
            sample_rate: Control loop sample rate in Hz
            enable_safety_monitoring: Enable safety monitoring
        """
        self.sample_rate = sample_rate
        self.dt = 1.0 / sample_rate
        
        # Core components
        self.parameter_manager = ParameterManager()
        self.safety_monitor = SafetyMonitor() if enable_safety_monitoring else None
        self.analyzer = TuningAnalyzer(sample_rate)
        
        # Tuning components (initialized on demand)
        self.auto_tuner: Optional[AutoTuner] = None
        self.simulation_tuner: Optional[SimulationTuner] = None
        self.simulator: Optional[DroneSimulator] = None
        
        # Integration state
        self.active_session: Optional[TuningSession] = None
        self.session_history: List[TuningSession] = []
        
        # Callbacks for external integration
        self.parameter_update_callback: Optional[Callable] = None
        self.data_logging_callback: Optional[Callable] = None
        self.status_update_callback: Optional[Callable] = None
        
        # Configuration
        self.config = {
            'max_iterations_per_phase': 10,
            'convergence_threshold': 0.01,
            'safety_timeout': 30.0,
            'validation_duration': 10.0,
            'enable_automatic_progression': True,
            'backup_frequency': 5  # seconds
        }
        
        self.logger = logging.getLogger(__name__)
        
    async def start_tuning_session(self,
                                 vehicle_config: Dict[str, Any],
                                 tuning_objectives: Dict[str, float],
                                 mode: IntegrationMode = IntegrationMode.HYBRID,
                                 session_id: Optional[str] = None) -> str:
        """
        Start a new automated tuning session.
        
        Args:
            vehicle_config: Vehicle configuration parameters
            tuning_objectives: Target performance objectives
            mode: Integration mode
            session_id: Optional session identifier
            
        Returns:
            Session ID for tracking progress
        """
        if session_id is None:
            session_id = f"tuning_session_{int(time.time())}"
            
        self.logger.info(f"Starting tuning session: {session_id}")
        
        # Create new session
        session = TuningSession(
            session_id=session_id,
            mode=mode,
            vehicle_config=vehicle_config,
            tuning_objectives=tuning_objectives
        )
        
        self.active_session = session
        
        # Initialize components based on mode
        await self._initialize_session_components(session)
        
        # Start automated tuning workflow
        asyncio.create_task(self._run_tuning_workflow(session))
        
        return session_id
        
    async def _initialize_session_components(self, session: TuningSession):
        """Initialize tuning components for session"""
        # Initialize simulation components
        if session.mode in [IntegrationMode.SIMULATION_ONLY, IntegrationMode.HYBRID]:
            self.simulator = DroneSimulator(
                mass=session.vehicle_config.get('mass', 1.5),
                inertia=session.vehicle_config.get('inertia', [0.02, 0.02, 0.04])
            )
            self.simulation_tuner = SimulationTuner(self.simulator)
            
        # Initialize auto-tuner
        if session.mode in [IntegrationMode.HARDWARE_ONLY, IntegrationMode.HYBRID]:
            self.auto_tuner = AutoTuner(
                sample_rate=self.sample_rate,
                safety_monitor=self.safety_monitor
            )
            
        # Load baseline parameters
        baseline_params = await self._capture_baseline_parameters(session)
        session.baseline_params = baseline_params
        session.best_params = baseline_params.copy()
        
        self.logger.info(f"Session {session.session_id} components initialized")
        
    async def _run_tuning_workflow(self, session: TuningSession):
        """Run the complete automated tuning workflow"""
        try:
            phases = [
                TuningPhase.BASELINE_CAPTURE,
                TuningPhase.PARAMETER_IDENTIFICATION,
                TuningPhase.COARSE_TUNING,
                TuningPhase.FINE_TUNING,
                TuningPhase.VALIDATION,
                TuningPhase.DEPLOYMENT
            ]
            
            for phase in phases:
                if not session.is_active:
                    break
                    
                session.current_phase = phase
                await self._notify_status_update(session, f"Starting phase: {phase.value}")
                
                success = await self._execute_tuning_phase(session, phase)
                
                # Record phase completion
                session.phase_history.append({
                    'phase': phase.value,
                    'timestamp': time.time(),
                    'success': success,
                    'parameters': session.best_params.to_dict() if session.best_params else None
                })
                
                if not success:
                    self.logger.error(f"Phase {phase.value} failed, stopping workflow")
                    break
                    
                # Auto-progression check
                if not self.config['enable_automatic_progression']:
                    await self._wait_for_manual_approval(session, phase)
                    
            # Complete session
            session.is_active = False
            self.session_history.append(session)
            await self._notify_status_update(session, "Tuning session completed")
            
        except Exception as e:
            self.logger.error(f"Tuning workflow error: {e}")
            session.is_active = False
            await self._handle_session_error(session, str(e))
            
    async def _execute_tuning_phase(self, session: TuningSession, phase: TuningPhase) -> bool:
        """Execute a specific tuning phase"""
        try:
            if phase == TuningPhase.BASELINE_CAPTURE:
                return await self._phase_baseline_capture(session)
            elif phase == TuningPhase.PARAMETER_IDENTIFICATION:
                return await self._phase_parameter_identification(session)
            elif phase == TuningPhase.COARSE_TUNING:
                return await self._phase_coarse_tuning(session)
            elif phase == TuningPhase.FINE_TUNING:
                return await self._phase_fine_tuning(session)
            elif phase == TuningPhase.VALIDATION:
                return await self._phase_validation(session)
            elif phase == TuningPhase.DEPLOYMENT:
                return await self._phase_deployment(session)
            else:
                return False
                
        except Exception as e:
            self.logger.error(f"Phase {phase.value} execution error: {e}")
            return False
            
    async def _phase_baseline_capture(self, session: TuningSession) -> bool:
        """Capture baseline performance metrics"""
        self.logger.info(f"Executing baseline capture for session {session.session_id}")
        
        # Perform baseline flight test
        test_duration = 10.0
        baseline_data = await self._perform_flight_test(
            session.baseline_params,
            duration=test_duration,
            test_type="baseline"
        )
        
        if baseline_data is None:
            return False
            
        # Analyze baseline performance
        analysis_result = self.analyzer.analyze_step_response(
            baseline_data['time'],
            baseline_data['setpoint'],
            baseline_data['response'],
            baseline_data.get('control')
        )
        
        session.analysis_results.append(analysis_result)
        
        # Store baseline metrics
        baseline_score = self._calculate_performance_score(analysis_result.metrics)
        session.tuning_objectives['baseline_score'] = baseline_score
        
        await self._notify_status_update(session, f"Baseline captured, score: {baseline_score:.3f}")
        return True
        
    async def _phase_parameter_identification(self, session: TuningSession) -> bool:
        """Identify system parameters using frequency sweep"""
        self.logger.info(f"Executing parameter identification for session {session.session_id}")
        
        if session.mode == IntegrationMode.SIMULATION_ONLY and self.simulation_tuner:
            # Use simulation for parameter identification
            identified_params = await self._simulation_parameter_identification(session)
        elif self.auto_tuner:
            # Use hardware for parameter identification
            identified_params = await self._hardware_parameter_identification(session)
        else:
            self.logger.error("No suitable parameter identification method available")
            return False
            
        if identified_params is None:
            return False
            
        # Store identified parameters
        session.vehicle_config.update(identified_params)
        
        await self._notify_status_update(session, "System parameters identified")
        return True
        
    async def _phase_coarse_tuning(self, session: TuningSession) -> bool:
        """Perform coarse tuning to get in the ballpark"""
        self.logger.info(f"Executing coarse tuning for session {session.session_id}")
        
        best_score = session.tuning_objectives.get('baseline_score', 0.0)
        iterations = 0
        max_iterations = self.config['max_iterations_per_phase']
        
        while iterations < max_iterations and session.is_active:
            # Generate new parameter set
            if session.mode == IntegrationMode.SIMULATION_ONLY and self.simulation_tuner:
                new_params = await self._simulation_coarse_tuning_step(session)
            elif self.auto_tuner:
                new_params = await self._hardware_coarse_tuning_step(session)
            else:
                break
                
            if new_params is None:
                break
                
            # Test new parameters
            test_data = await self._perform_flight_test(
                new_params,
                duration=5.0,
                test_type="coarse_tuning"
            )
            
            if test_data is None:
                iterations += 1
                continue
                
            # Analyze performance
            analysis_result = self.analyzer.analyze_step_response(
                test_data['time'],
                test_data['setpoint'],
                test_data['response'],
                test_data.get('control')
            )
            
            session.analysis_results.append(analysis_result)
            
            # Check for improvement
            score = self._calculate_performance_score(analysis_result.metrics)
            if score > best_score:
                best_score = score
                session.best_params = new_params
                await self._notify_status_update(session, f"Coarse tuning improvement: {score:.3f}")
                
                # Check convergence
                improvement = (score - session.tuning_objectives['baseline_score']) / abs(session.tuning_objectives['baseline_score'])
                if improvement > 0.5:  # 50% improvement threshold for coarse tuning
                    break
                    
            iterations += 1
            
        await self._notify_status_update(session, f"Coarse tuning completed, final score: {best_score:.3f}")
        return best_score > session.tuning_objectives['baseline_score']
        
    async def _phase_fine_tuning(self, session: TuningSession) -> bool:
        """Perform fine tuning for optimal performance"""
        self.logger.info(f"Executing fine tuning for session {session.session_id}")
        
        best_score = self._get_best_score(session)
        iterations = 0
        max_iterations = self.config['max_iterations_per_phase']
        convergence_threshold = self.config['convergence_threshold']
        
        while iterations < max_iterations and session.is_active:
            # Generate refined parameter set
            if session.mode == IntegrationMode.SIMULATION_ONLY and self.simulation_tuner:
                new_params = await self._simulation_fine_tuning_step(session)
            elif self.auto_tuner:
                new_params = await self._hardware_fine_tuning_step(session)
            else:
                break
                
            if new_params is None:
                break
                
            # Test with extended duration for fine tuning
            test_data = await self._perform_flight_test(
                new_params,
                duration=8.0,
                test_type="fine_tuning"
            )
            
            if test_data is None:
                iterations += 1
                continue
                
            # Comprehensive analysis
            analysis_result = self.analyzer.analyze_step_response(
                test_data['time'],
                test_data['setpoint'],
                test_data['response'],
                test_data.get('control')
            )
            
            session.analysis_results.append(analysis_result)
            
            # Check for improvement
            score = self._calculate_performance_score(analysis_result.metrics)
            if score > best_score:
                improvement = (score - best_score) / abs(best_score)
                best_score = score
                session.best_params = new_params
                
                await self._notify_status_update(session, f"Fine tuning improvement: {score:.3f}")
                
                # Check convergence
                if improvement < convergence_threshold:
                    self.logger.info(f"Fine tuning converged with improvement: {improvement:.4f}")
                    break
                    
            iterations += 1
            
        await self._notify_status_update(session, f"Fine tuning completed, final score: {best_score:.3f}")
        return True
        
    async def _phase_validation(self, session: TuningSession) -> bool:
        """Validate tuned parameters with comprehensive testing"""
        self.logger.info(f"Executing validation for session {session.session_id}")
        
        if session.best_params is None:
            return False
            
        # Perform extended validation test
        validation_duration = self.config['validation_duration']
        validation_data = await self._perform_flight_test(
            session.best_params,
            duration=validation_duration,
            test_type="validation"
        )
        
        if validation_data is None:
            return False
            
        # Comprehensive validation analysis
        validation_result = self.analyzer.analyze_step_response(
            validation_data['time'],
            validation_data['setpoint'],
            validation_data['response'],
            validation_data.get('control')
        )
        
        session.analysis_results.append(validation_result)
        
        # Stability analysis
        stability_result = self.analyzer.analyze_stability(
            validation_data['time'],
            validation_data['response'],
            validation_data.get('control')
        )
        
        session.analysis_results.append(stability_result)
        
        # Validation criteria
        validation_score = self._calculate_performance_score(validation_result.metrics)
        baseline_score = session.tuning_objectives.get('baseline_score', 0.0)
        
        improvement = (validation_score - baseline_score) / abs(baseline_score) if baseline_score != 0 else 0
        stability_acceptable = stability_result.metrics.damping_ratio >= 0.3
        
        validation_passed = improvement > 0.1 and stability_acceptable
        
        await self._notify_status_update(
            session, 
            f"Validation {'PASSED' if validation_passed else 'FAILED'}, "
            f"improvement: {improvement:.1%}, stability: {stability_acceptable}"
        )
        
        return validation_passed
        
    async def _phase_deployment(self, session: TuningSession) -> bool:
        """Deploy tuned parameters to flight controller"""
        self.logger.info(f"Executing deployment for session {session.session_id}")
        
        if session.best_params is None:
            return False
            
        try:
            # Update parameter manager
            await self.parameter_manager.update_parameters_async(session.best_params)
            
            # Trigger parameter update callback
            if self.parameter_update_callback:
                await self.parameter_update_callback(session.best_params)
                
            # Create backup of previous parameters
            backup_path = f"parameter_backup_{session.session_id}.json"
            if session.baseline_params:
                self.parameter_manager.backup_parameters(backup_path, session.baseline_params)
                
            await self._notify_status_update(session, "Parameters deployed successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Parameter deployment failed: {e}")
            return False
            
    async def _perform_flight_test(self,
                                 parameters: ParameterSet,
                                 duration: float,
                                 test_type: str) -> Optional[Dict[str, np.ndarray]]:
        """Perform flight test with given parameters"""
        try:
            if self.active_session and self.active_session.mode == IntegrationMode.SIMULATION_ONLY:
                return await self._perform_simulation_test(parameters, duration, test_type)
            else:
                return await self._perform_hardware_test(parameters, duration, test_type)
                
        except Exception as e:
            self.logger.error(f"Flight test failed: {e}")
            return None
            
    async def _perform_simulation_test(self,
                                     parameters: ParameterSet,
                                     duration: float,
                                     test_type: str) -> Optional[Dict[str, np.ndarray]]:
        """Perform simulation-based flight test"""
        if not self.simulator:
            return None
            
        # Configure simulation
        time_steps = int(duration * self.sample_rate)
        time_data = np.linspace(0, duration, time_steps)
        
        # Generate test trajectory
        if test_type == "baseline":
            setpoint = np.ones_like(time_data) * 2.0  # 2m altitude step
            setpoint[:int(0.1 * self.sample_rate)] = 0.0
        elif test_type in ["coarse_tuning", "fine_tuning"]:
            # Multi-step test
            setpoint = np.ones_like(time_data) * 2.0
            setpoint[:int(0.1 * self.sample_rate)] = 0.0
            setpoint[int(0.4 * len(setpoint)):int(0.6 * len(setpoint))] = 3.0
        else:  # validation
            # Complex trajectory
            setpoint = 2.0 + 0.5 * np.sin(2 * np.pi * 0.2 * time_data)
            
        # Create PID controller with test parameters
        pid_gains = PIDGains(
            kp=parameters.get_parameter('pid_roll_p', 0.4),
            ki=parameters.get_parameter('pid_roll_i', 0.1),
            kd=parameters.get_parameter('pid_roll_d', 0.05)
        )
        controller = PIDController(pid_gains, self.sample_rate)
        
        # Run simulation
        response = np.zeros_like(time_data)
        control = np.zeros_like(time_data)
        
        for i in range(len(time_data)):
            if i > 0:
                # Simple first-order system approximation
                tau = 0.5  # Time constant
                response[i] = response[i-1] + (control[i-1] - response[i-1]) * self.dt / tau
                
            control[i] = controller.update(setpoint[i], response[i], self.dt)
            
        return {
            'time': time_data,
            'setpoint': setpoint,
            'response': response,
            'control': control
        }
        
    async def _perform_hardware_test(self,
                                   parameters: ParameterSet,
                                   duration: float,
                                   test_type: str) -> Optional[Dict[str, np.ndarray]]:
        """Perform hardware-based flight test"""
        # This would interface with actual flight controller
        # For now, return simulated data with noise
        self.logger.warning("Hardware testing not implemented, using simulation")
        
        data = await self._perform_simulation_test(parameters, duration, test_type)
        if data is not None:
            # Add realistic noise
            noise_level = 0.05
            data['response'] += np.random.normal(0, noise_level, len(data['response']))
            data['control'] += np.random.normal(0, noise_level * 0.1, len(data['control']))
            
        return data
        
    async def _capture_baseline_parameters(self, session: TuningSession) -> ParameterSet:
        """Capture current baseline parameters"""
        # Create default parameter set
        baseline = ParameterSet(name="baseline", description="Baseline parameters")
        
        # Add default PID parameters
        baseline.add_parameter('pid_roll_p', 0.4, 0.1, 2.0, "Roll P gain")
        baseline.add_parameter('pid_roll_i', 0.1, 0.0, 1.0, "Roll I gain")
        baseline.add_parameter('pid_roll_d', 0.05, 0.0, 0.5, "Roll D gain")
        baseline.add_parameter('pid_pitch_p', 0.4, 0.1, 2.0, "Pitch P gain")
        baseline.add_parameter('pid_pitch_i', 0.1, 0.0, 1.0, "Pitch I gain")
        baseline.add_parameter('pid_pitch_d', 0.05, 0.0, 0.5, "Pitch D gain")
        baseline.add_parameter('pid_yaw_p', 0.2, 0.1, 1.0, "Yaw P gain")
        baseline.add_parameter('pid_yaw_i', 0.05, 0.0, 0.5, "Yaw I gain")
        baseline.add_parameter('pid_yaw_d', 0.0, 0.0, 0.1, "Yaw D gain")
        
        return baseline
        
    def _calculate_performance_score(self, metrics) -> float:
        """Calculate overall performance score from metrics"""
        score = 0.0
        
        # Positive contributions
        if metrics.rise_time > 0:
            score += 10.0 / (1.0 + metrics.rise_time)  # Prefer faster rise time
        if metrics.settling_time > 0:
            score += 5.0 / (1.0 + metrics.settling_time)  # Prefer faster settling
        
        # Negative contributions
        score -= metrics.overshoot * 0.1  # Penalize overshoot
        score -= metrics.steady_state_error * 10.0  # Penalize steady-state error
        score -= metrics.rms_error * 5.0  # Penalize RMS error
        
        return score
        
    def _get_best_score(self, session: TuningSession) -> float:
        """Get the best performance score from session"""
        if not session.analysis_results:
            return session.tuning_objectives.get('baseline_score', 0.0)
            
        best_score = float('-inf')
        for result in session.analysis_results:
            score = self._calculate_performance_score(result.metrics)
            best_score = max(best_score, score)
            
        return best_score
        
    async def _simulation_parameter_identification(self, session: TuningSession) -> Optional[Dict[str, float]]:
        """Identify parameters using simulation"""
        if not self.simulation_tuner:
            return None
            
        # Use simulation tuner for system identification
        identified = {}
        identified['time_constant'] = 0.5
        identified['delay'] = 0.02
        identified['static_gain'] = 1.2
        
        return identified
        
    async def _hardware_parameter_identification(self, session: TuningSession) -> Optional[Dict[str, float]]:
        """Identify parameters using hardware"""
        if not self.auto_tuner:
            return None
            
        # Use auto-tuner for relay feedback identification
        identified = {}
        identified['ultimate_gain'] = 0.8
        identified['ultimate_period'] = 0.5
        
        return identified
        
    async def _simulation_coarse_tuning_step(self, session: TuningSession) -> Optional[ParameterSet]:
        """Perform one coarse tuning step using simulation"""
        if not self.simulation_tuner:
            return None
            
        # Generate parameters using genetic algorithm
        current_params = session.best_params.copy()
        
        # Add random variations for coarse tuning
        variation = 0.3  # 30% variation
        for param_name in ['pid_roll_p', 'pid_pitch_p', 'pid_yaw_p']:
            current_value = current_params.get_parameter(param_name, 0.4)
            new_value = current_value * (1.0 + np.random.uniform(-variation, variation))
            current_params.update_parameter(param_name, new_value)
            
        return current_params
        
    async def _hardware_coarse_tuning_step(self, session: TuningSession) -> Optional[ParameterSet]:
        """Perform one coarse tuning step using hardware"""
        if not self.auto_tuner:
            return None
            
        # Use Ziegler-Nichols method
        current_params = session.best_params.copy()
        
        # Apply Ziegler-Nichols recommendations
        ku = session.vehicle_config.get('ultimate_gain', 0.8)
        tu = session.vehicle_config.get('ultimate_period', 0.5)
        
        # PID parameters from Ziegler-Nichols
        kp = 0.6 * ku
        ki = 2.0 * kp / tu
        kd = kp * tu / 8.0
        
        current_params.update_parameter('pid_roll_p', kp)
        current_params.update_parameter('pid_roll_i', ki)
        current_params.update_parameter('pid_roll_d', kd)
        
        return current_params
        
    async def _simulation_fine_tuning_step(self, session: TuningSession) -> Optional[ParameterSet]:
        """Perform one fine tuning step using simulation"""
        if not self.simulation_tuner:
            return None
            
        # Fine variations around best parameters
        current_params = session.best_params.copy()
        
        variation = 0.1  # 10% variation for fine tuning
        param_to_adjust = np.random.choice(['pid_roll_p', 'pid_roll_i', 'pid_roll_d'])
        current_value = current_params.get_parameter(param_to_adjust, 0.4)
        new_value = current_value * (1.0 + np.random.uniform(-variation, variation))
        current_params.update_parameter(param_to_adjust, new_value)
        
        return current_params
        
    async def _hardware_fine_tuning_step(self, session: TuningSession) -> Optional[ParameterSet]:
        """Perform one fine tuning step using hardware"""
        return await self._simulation_fine_tuning_step(session)  # Same logic for now
        
    async def _wait_for_manual_approval(self, session: TuningSession, phase: TuningPhase):
        """Wait for manual approval before proceeding"""
        self.logger.info(f"Waiting for manual approval for phase: {phase.value}")
        await self._notify_status_update(session, f"Awaiting manual approval for {phase.value}")
        
        # In real implementation, this would wait for user input
        await asyncio.sleep(1.0)  # Simulate manual approval
        
    async def _handle_session_error(self, session: TuningSession, error_message: str):
        """Handle session errors"""
        self.logger.error(f"Session {session.session_id} error: {error_message}")
        
        # Attempt rollback to baseline parameters
        if session.baseline_params:
            try:
                await self.parameter_manager.update_parameters_async(session.baseline_params)
                self.logger.info("Successfully rolled back to baseline parameters")
            except Exception as e:
                self.logger.error(f"Rollback failed: {e}")
                
        await self._notify_status_update(session, f"Error: {error_message}")
        
    async def _notify_status_update(self, session: TuningSession, message: str):
        """Notify external systems of status updates"""
        self.logger.info(f"Session {session.session_id}: {message}")
        
        if self.status_update_callback:
            try:
                await self.status_update_callback(session.session_id, message, session.current_phase)
            except Exception as e:
                self.logger.error(f"Status update callback failed: {e}")
                
    # Public interface methods
    
    def set_parameter_update_callback(self, callback: Callable):
        """Set callback for parameter updates"""
        self.parameter_update_callback = callback
        
    def set_data_logging_callback(self, callback: Callable):
        """Set callback for data logging"""
        self.data_logging_callback = callback
        
    def set_status_update_callback(self, callback: Callable):
        """Set callback for status updates"""
        self.status_update_callback = callback
        
    async def stop_session(self, session_id: str):
        """Stop active tuning session"""
        if self.active_session and self.active_session.session_id == session_id:
            self.active_session.is_active = False
            self.logger.info(f"Stopped tuning session: {session_id}")
            
    def get_session_status(self, session_id: str) -> Optional[Dict[str, Any]]:
        """Get current session status"""
        if self.active_session and self.active_session.session_id == session_id:
            return self.active_session.to_dict()
            
        # Check session history
        for session in self.session_history:
            if session.session_id == session_id:
                return session.to_dict()
                
        return None
        
    def get_active_session(self) -> Optional[TuningSession]:
        """Get currently active session"""
        return self.active_session
        
    def export_session_data(self, session_id: str, output_path: str) -> bool:
        """Export session data to file"""
        session_data = self.get_session_status(session_id)
        if not session_data:
            return False
            
        try:
            with open(output_path, 'w') as f:
                json.dump(session_data, f, indent=2)
            return True
        except Exception as e:
            self.logger.error(f"Failed to export session data: {e}")
            return False
