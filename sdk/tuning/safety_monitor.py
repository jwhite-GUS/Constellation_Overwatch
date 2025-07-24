"""
Constellation Overwatch SDK - Safety Monitor
Real-time safety monitoring system for autonomous flight operations and tuning.
"""

import numpy as np
import logging
from typing import Dict, List, Optional, Callable
from dataclasses import dataclass, field
from enum import Enum
import time
import threading
from collections import deque


class SafetyLevel(Enum):
    """Safety severity levels"""

    SAFE = "safe"
    CAUTION = "caution"
    WARNING = "warning"
    CRITICAL = "critical"
    EMERGENCY = "emergency"


class SafetyViolationType(Enum):
    """Types of safety violations"""

    ATTITUDE_LIMIT = "attitude_limit"
    RATE_LIMIT = "rate_limit"
    ACCELERATION_LIMIT = "acceleration_limit"
    OSCILLATION = "oscillation"
    CONTROL_SATURATION = "control_saturation"
    SENSOR_FAILURE = "sensor_failure"
    COMMUNICATION_LOSS = "communication_loss"
    BATTERY_LOW = "battery_low"
    GEOFENCE_BREACH = "geofence_breach"


@dataclass
class SafetyLimits:
    """Safety limits for flight operations"""

    # Attitude limits (degrees)
    max_roll: float = 45.0
    max_pitch: float = 45.0
    max_yaw_rate: float = 180.0  # deg/s

    # Rate limits (deg/s)
    max_roll_rate: float = 200.0
    max_pitch_rate: float = 200.0
    max_yaw_rate_limit: float = 180.0

    # Acceleration limits (m/s²)
    max_acceleration: float = 10.0
    max_angular_accel: float = 1000.0  # deg/s²

    # Control limits
    max_control_output: float = 1.0
    control_saturation_threshold: float = 0.95

    # Oscillation detection
    oscillation_threshold: float = 5.0  # Hz
    oscillation_amplitude_threshold: float = 0.1

    # Sensor limits
    max_sensor_noise: float = 0.5
    sensor_timeout: float = 0.1  # seconds

    # Battery limits
    min_battery_voltage: float = 14.4  # V (for 4S LiPo)
    critical_battery_voltage: float = 13.2  # V

    # Geofence (meters from takeoff point)
    max_horizontal_distance: float = 1000.0
    max_altitude: float = 400.0  # AGL
    min_altitude: float = -10.0


@dataclass
class SafetyViolation:
    """Safety violation record"""

    timestamp: float
    violation_type: SafetyViolationType
    severity: SafetyLevel
    value: float
    limit: float
    message: str
    data: Dict = field(default_factory=dict)


class SafetyMonitor:
    """
    Real-time safety monitoring system for autonomous operations.

    Features:
    - Multi-level safety monitoring (attitude, rates, control outputs)
    - Oscillation detection and frequency analysis
    - Sensor health monitoring
    - Automatic safety responses
    - Real-time alerting and logging
    - Integration with auto-tuning systems
    """

    def __init__(
        self,
        limits: SafetyLimits = None,
        history_size: int = 1000,
        enable_auto_response: bool = True,
    ):
        """
        Initialize safety monitor.

        Args:
            limits: Safety limits configuration
            history_size: Number of historical samples to maintain
            enable_auto_response: Whether to enable automatic safety responses
        """
        self.limits = limits or SafetyLimits()
        self.history_size = history_size
        self.enable_auto_response = enable_auto_response

        # Safety state
        self.current_level = SafetyLevel.SAFE
        self.violations = deque(maxlen=100)  # Keep recent violations
        self.is_armed = False
        self.emergency_stop_triggered = False

        # Data history for analysis
        self.attitude_history = deque(maxlen=history_size)
        self.rate_history = deque(maxlen=history_size)
        self.control_history = deque(maxlen=history_size)
        self.timestamp_history = deque(maxlen=history_size)

        # Oscillation detection
        self.oscillation_detector = OscillationDetector()

        # Sensor monitoring
        self.last_sensor_update = time.time()
        self.sensor_health = {"imu": True, "gps": True, "baro": True, "mag": True}

        # Callbacks for safety responses
        self.emergency_callbacks: List[Callable] = []
        self.warning_callbacks: List[Callable] = []

        self.logger = logging.getLogger(__name__)

        # Start monitoring thread
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_running = True
        self._monitor_thread.start()

    def update_state(
        self,
        attitude: Dict[str, float],
        rates: Dict[str, float],
        control_outputs: Dict[str, float],
        sensor_data: Optional[Dict] = None,
    ):
        """
        Update safety monitor with current state.

        Args:
            attitude: {'roll': deg, 'pitch': deg, 'yaw': deg}
            rates: {'roll': deg/s, 'pitch': deg/s, 'yaw': deg/s}
            control_outputs: {'roll': output, 'pitch': output, 'yaw': output, 'throttle': output}
            sensor_data: Optional sensor health and data
        """
        current_time = time.time()

        # Update history
        self.attitude_history.append(attitude.copy())
        self.rate_history.append(rates.copy())
        self.control_history.append(control_outputs.copy())
        self.timestamp_history.append(current_time)

        # Update sensor health
        if sensor_data:
            self._update_sensor_health(sensor_data)

        # Check safety conditions
        self._check_attitude_limits(attitude)
        self._check_rate_limits(rates)
        self._check_control_limits(control_outputs)
        self._check_oscillations()

        # Update sensor monitoring
        self.last_sensor_update = current_time

    def is_safe(
        self,
        measurement: Optional[float] = None,
        control_output: Optional[float] = None,
        timestamp: Optional[float] = None,
    ) -> bool:
        """
        Quick safety check for single values (used during auto-tuning).

        Args:
            measurement: Current measurement value
            control_output: Current control output
            timestamp: Current timestamp

        Returns:
            True if safe to continue
        """
        if self.emergency_stop_triggered:
            return False

        if self.current_level in [SafetyLevel.CRITICAL, SafetyLevel.EMERGENCY]:
            return False

        # Check control output saturation
        if control_output is not None:
            if abs(control_output) > self.limits.max_control_output:
                self._add_violation(
                    SafetyViolationType.CONTROL_SATURATION,
                    SafetyLevel.WARNING,
                    abs(control_output),
                    self.limits.max_control_output,
                    f"Control output {control_output:.3f} exceeds limit",
                )
                return False

        # Check measurement bounds (simple range check)
        if measurement is not None:
            if abs(measurement) > 10.0:  # Generic large value check
                self._add_violation(
                    SafetyViolationType.SENSOR_FAILURE,
                    SafetyLevel.WARNING,
                    abs(measurement),
                    10.0,
                    f"Measurement {measurement:.3f} out of reasonable range",
                )
                return False

        return True

    def _check_attitude_limits(self, attitude: Dict[str, float]):
        """Check attitude limits"""
        roll = attitude.get("roll", 0.0)
        pitch = attitude.get("pitch", 0.0)

        # Roll limit check
        if abs(roll) > self.limits.max_roll:
            severity = (
                SafetyLevel.CRITICAL
                if abs(roll) > self.limits.max_roll * 1.2
                else SafetyLevel.WARNING
            )
            self._add_violation(
                SafetyViolationType.ATTITUDE_LIMIT,
                severity,
                abs(roll),
                self.limits.max_roll,
                f"Roll angle {roll:.1f}° exceeds limit {self.limits.max_roll}°",
            )

        # Pitch limit check
        if abs(pitch) > self.limits.max_pitch:
            severity = (
                SafetyLevel.CRITICAL
                if abs(pitch) > self.limits.max_pitch * 1.2
                else SafetyLevel.WARNING
            )
            self._add_violation(
                SafetyViolationType.ATTITUDE_LIMIT,
                severity,
                abs(pitch),
                self.limits.max_pitch,
                f"Pitch angle {pitch:.1f}° exceeds limit {self.limits.max_pitch}°",
            )

    def _check_rate_limits(self, rates: Dict[str, float]):
        """Check angular rate limits"""
        rate_checks = [
            ("roll", rates.get("roll", 0.0), self.limits.max_roll_rate),
            ("pitch", rates.get("pitch", 0.0), self.limits.max_pitch_rate),
            ("yaw", rates.get("yaw", 0.0), self.limits.max_yaw_rate_limit),
        ]

        for axis, rate, limit in rate_checks:
            if abs(rate) > limit:
                severity = (
                    SafetyLevel.CRITICAL
                    if abs(rate) > limit * 1.5
                    else SafetyLevel.WARNING
                )
                self._add_violation(
                    SafetyViolationType.RATE_LIMIT,
                    severity,
                    abs(rate),
                    limit,
                    f"{axis.title()} rate {rate:.1f}°/s exceeds limit {limit}°/s",
                )

    def _check_control_limits(self, control_outputs: Dict[str, float]):
        """Check control output limits"""
        saturation_count = 0

        for axis, output in control_outputs.items():
            if abs(output) > self.limits.control_saturation_threshold:
                saturation_count += 1

            if abs(output) > self.limits.max_control_output:
                self._add_violation(
                    SafetyViolationType.CONTROL_SATURATION,
                    SafetyLevel.WARNING,
                    abs(output),
                    self.limits.max_control_output,
                    f"{axis.title()} control output {output:.3f} exceeds limit",
                )

        # Check for simultaneous saturation (indicates control issues)
        if saturation_count >= 3:
            self._add_violation(
                SafetyViolationType.CONTROL_SATURATION,
                SafetyLevel.CRITICAL,
                saturation_count,
                2,
                f"Multiple control outputs saturated ({saturation_count} axes)",
            )

    def _check_oscillations(self):
        """Check for dangerous oscillations"""
        if len(self.attitude_history) < 100:  # Need sufficient data
            return

        # Use oscillation detector
        recent_attitudes = list(self.attitude_history)[-100:]  # Last 100 samples

        for axis in ["roll", "pitch", "yaw"]:
            axis_data = [att.get(axis, 0.0) for att in recent_attitudes]
            oscillation_freq = self.oscillation_detector.detect_oscillation_frequency(
                axis_data
            )

            if oscillation_freq > self.limits.oscillation_threshold:
                # Check amplitude
                amplitude = np.std(axis_data)
                if amplitude > self.limits.oscillation_amplitude_threshold:
                    self._add_violation(
                        SafetyViolationType.OSCILLATION,
                        SafetyLevel.CRITICAL,
                        oscillation_freq,
                        self.limits.oscillation_threshold,
                        f"{axis.title()} oscillating at {oscillation_freq:.1f} Hz (amplitude {amplitude:.3f})",
                    )

    def _update_sensor_health(self, sensor_data: Dict):
        """Update sensor health monitoring"""
        current_time = time.time()

        # Check for sensor timeouts
        for sensor in self.sensor_health:
            if sensor in sensor_data:
                last_update = sensor_data[sensor].get("timestamp", current_time)
                if current_time - last_update > self.limits.sensor_timeout:
                    self.sensor_health[sensor] = False
                    self._add_violation(
                        SafetyViolationType.SENSOR_FAILURE,
                        SafetyLevel.WARNING,
                        current_time - last_update,
                        self.limits.sensor_timeout,
                        f"{sensor.upper()} sensor timeout",
                    )
                else:
                    self.sensor_health[sensor] = True

        # Check sensor noise levels
        if "imu" in sensor_data:
            imu_data = sensor_data["imu"]
            for axis in ["gx", "gy", "gz", "ax", "ay", "az"]:
                if axis in imu_data:
                    noise_level = abs(imu_data[axis])
                    if noise_level > self.limits.max_sensor_noise:
                        self._add_violation(
                            SafetyViolationType.SENSOR_FAILURE,
                            SafetyLevel.CAUTION,
                            noise_level,
                            self.limits.max_sensor_noise,
                            f"IMU {axis} high noise level: {noise_level:.3f}",
                        )

    def _add_violation(
        self,
        violation_type: SafetyViolationType,
        severity: SafetyLevel,
        value: float,
        limit: float,
        message: str,
        data: Dict = None,
    ):
        """Add safety violation"""
        violation = SafetyViolation(
            timestamp=time.time(),
            violation_type=violation_type,
            severity=severity,
            value=value,
            limit=limit,
            message=message,
            data=data or {},
        )

        self.violations.append(violation)

        # Update current safety level
        if severity.value > self.current_level.value:
            self.current_level = severity

        # Log violation
        log_level = {
            SafetyLevel.SAFE: logging.INFO,
            SafetyLevel.CAUTION: logging.INFO,
            SafetyLevel.WARNING: logging.WARNING,
            SafetyLevel.CRITICAL: logging.ERROR,
            SafetyLevel.EMERGENCY: logging.CRITICAL,
        }.get(severity, logging.WARNING)

        self.logger.log(log_level, f"Safety violation ({severity.value}): {message}")

        # Trigger callbacks
        if severity in [SafetyLevel.CRITICAL, SafetyLevel.EMERGENCY]:
            for callback in self.emergency_callbacks:
                try:
                    callback(violation)
                except Exception as e:
                    self.logger.error(f"Emergency callback failed: {e}")

        elif severity == SafetyLevel.WARNING:
            for callback in self.warning_callbacks:
                try:
                    callback(violation)
                except Exception as e:
                    self.logger.error(f"Warning callback failed: {e}")

        # Auto-response
        if self.enable_auto_response and severity == SafetyLevel.EMERGENCY:
            self.trigger_emergency_stop()

    def trigger_emergency_stop(self):
        """Trigger emergency stop"""
        if not self.emergency_stop_triggered:
            self.emergency_stop_triggered = True
            self.current_level = SafetyLevel.EMERGENCY
            self.logger.critical("EMERGENCY STOP TRIGGERED")

            # Execute emergency callbacks
            for callback in self.emergency_callbacks:
                try:
                    callback(None)  # None indicates emergency stop
                except Exception as e:
                    self.logger.error(f"Emergency stop callback failed: {e}")

    def reset_emergency(self):
        """Reset emergency stop (requires manual intervention)"""
        self.emergency_stop_triggered = False
        self.current_level = SafetyLevel.SAFE
        self.violations.clear()
        self.logger.info("Emergency reset - safety monitor cleared")

    def add_emergency_callback(self, callback: Callable):
        """Add emergency response callback"""
        self.emergency_callbacks.append(callback)

    def add_warning_callback(self, callback: Callable):
        """Add warning response callback"""
        self.warning_callbacks.append(callback)

    def get_safety_status(self) -> Dict:
        """Get current safety status"""
        return {
            "level": self.current_level.value,
            "emergency_stop": self.emergency_stop_triggered,
            "recent_violations": len(
                [v for v in self.violations if time.time() - v.timestamp < 10.0]
            ),
            "sensor_health": self.sensor_health.copy(),
            "is_armed": self.is_armed,
        }

    def get_recent_violations(self, time_window: float = 60.0) -> List[SafetyViolation]:
        """Get recent safety violations"""
        current_time = time.time()
        return [v for v in self.violations if current_time - v.timestamp <= time_window]

    def _monitor_loop(self):
        """Background monitoring loop"""
        while self._monitor_running:
            try:
                # Periodic checks
                current_time = time.time()

                # Check for sensor timeouts
                if (
                    current_time - self.last_sensor_update
                    > self.limits.sensor_timeout * 2
                ):
                    self._add_violation(
                        SafetyViolationType.SENSOR_FAILURE,
                        SafetyLevel.CRITICAL,
                        current_time - self.last_sensor_update,
                        self.limits.sensor_timeout * 2,
                        "Global sensor timeout - no updates received",
                    )

                # Auto-decay safety level if no recent violations
                if self.current_level != SafetyLevel.SAFE:
                    recent_violations = self.get_recent_violations(5.0)
                    if not recent_violations:
                        # Decay safety level
                        if self.current_level == SafetyLevel.CRITICAL:
                            self.current_level = SafetyLevel.WARNING
                        elif self.current_level == SafetyLevel.WARNING:
                            self.current_level = SafetyLevel.CAUTION
                        elif self.current_level == SafetyLevel.CAUTION:
                            self.current_level = SafetyLevel.SAFE

                time.sleep(0.1)  # 10 Hz monitoring

            except Exception as e:
                self.logger.error(f"Safety monitor loop error: {e}")
                time.sleep(1.0)

    def shutdown(self):
        """Shutdown safety monitor"""
        self._monitor_running = False
        if self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=1.0)


class OscillationDetector:
    """Oscillation detection using frequency analysis"""

    def __init__(self, sample_rate: float = 100.0):
        """
        Initialize oscillation detector.

        Args:
            sample_rate: Data sampling rate in Hz
        """
        self.sample_rate = sample_rate

    def detect_oscillation_frequency(self, data: List[float]) -> float:
        """
        Detect dominant oscillation frequency in data.

        Args:
            data: Time series data

        Returns:
            Dominant frequency in Hz
        """
        if len(data) < 20:
            return 0.0

        # Remove DC component
        data_array = np.array(data)
        data_centered = data_array - np.mean(data_array)

        # Apply window to reduce spectral leakage
        window = np.hanning(len(data_centered))
        windowed_data = data_centered * window

        # FFT
        fft_result = np.fft.fft(windowed_data)
        frequencies = np.fft.fftfreq(len(windowed_data), 1.0 / self.sample_rate)

        # Only positive frequencies
        positive_freqs = frequencies[: len(frequencies) // 2]
        magnitude = np.abs(fft_result[: len(fft_result) // 2])

        # Find peak frequency
        if len(magnitude) > 1:
            peak_idx = np.argmax(magnitude[1:]) + 1  # Skip DC component
            peak_frequency = positive_freqs[peak_idx]
            return abs(peak_frequency)
        else:
            return 0.0

    def is_oscillating(self, data: List[float], threshold_freq: float = 2.0) -> bool:
        """
        Check if data shows oscillation above threshold frequency.

        Args:
            data: Time series data
            threshold_freq: Frequency threshold in Hz

        Returns:
            True if oscillating above threshold
        """
        dominant_freq = self.detect_oscillation_frequency(data)
        return dominant_freq > threshold_freq
