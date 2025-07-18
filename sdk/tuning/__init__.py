"""
Constellation Overwatch SDK - Autopilot Tuning Framework

COPILOT: Autopilot tuning module - ensure safety-first approach in all tuning operations
COPILOT: This module handles critical flight control parameters - validate all changes thoroughly

Advanced tuning system for flight controller optimization and parameter management.
"""

from .pid_controller import PIDController, PIDGains, RatePIDController
from .auto_tuner import AutoTuner, TuningMethod, SystemIdentifier
from .simulation_tuner import SimulationTuner, DroneSimulator, OptimizationMethod
from .safety_monitor import SafetyMonitor, SafetyLevel, SafetyViolation, OscillationDetector
from .parameter_manager import ParameterManager, ParameterSet, Parameter
from .tuning_analyzer import TuningAnalyzer, AnalysisType, PerformanceMetrics, AnalysisResult
from .tuning_integrator import TuningIntegrator, IntegrationMode, TuningPhase, TuningSession
from .examples import TuningExamples

__version__ = "1.0.0"
__author__ = "Constellation Overwatch Team"

__all__ = [
    # PID Control
    'PIDController',
    'PIDGains',
    'RatePIDController',
    
    # Auto Tuning
    'AutoTuner',
    'TuningMethod',
    'SystemIdentifier',
    
    # Simulation
    'SimulationTuner',
    'DroneSimulator',
    'OptimizationMethod',
    
    # Safety
    'SafetyMonitor',
    'SafetyLevel',
    'SafetyViolation',
    'OscillationDetector',
    
    # Parameter Management
    'ParameterManager',
    'ParameterSet',
    'Parameter',
    
    # Analysis
    'TuningAnalyzer',
    'AnalysisType',
    'PerformanceMetrics',
    'AnalysisResult',
    
    # Integration
    'TuningIntegrator',
    'IntegrationMode',
    'TuningPhase',
    'TuningSession',
    
    # Examples
    'TuningExamples',
]
