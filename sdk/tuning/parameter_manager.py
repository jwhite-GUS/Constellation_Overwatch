"""
Constellation Overwatch SDK - Parameter Manager
Advanced parameter management system for flight controllers with real-time tuning capabilities.
"""

import numpy as np
import logging
import json
import pickle
from typing import Dict, List, Tuple, Optional, Any, Union
from dataclasses import dataclass, field, asdict
from enum import Enum
from pathlib import Path
import time
import threading
from copy import deepcopy

from .pid_controller import PIDGains


class ParameterType(Enum):
    """Parameter data types"""

    FLOAT = "float"
    INT = "int"
    BOOL = "bool"
    STRING = "string"
    ARRAY = "array"


class ParameterCategory(Enum):
    """Parameter categories"""

    CONTROL = "control"
    SAFETY = "safety"
    ESTIMATION = "estimation"
    MISSION = "mission"
    HARDWARE = "hardware"
    TUNING = "tuning"
    SYSTEM = "system"


@dataclass
class Parameter:
    """Individual parameter definition"""

    name: str
    value: Any
    param_type: ParameterType
    category: ParameterCategory
    description: str = ""
    min_value: Optional[float] = None
    max_value: Optional[float] = None
    units: str = ""
    requires_restart: bool = False
    read_only: bool = False
    default_value: Any = None

    def __post_init__(self):
        if self.default_value is None:
            self.default_value = self.value

    def validate(self, value: Any) -> bool:
        """Validate parameter value"""
        if self.read_only:
            return value == self.value

        # Type checking
        if self.param_type == ParameterType.FLOAT:
            if not isinstance(value, (int, float)):
                return False
            if self.min_value is not None and value < self.min_value:
                return False
            if self.max_value is not None and value > self.max_value:
                return False

        elif self.param_type == ParameterType.INT:
            if not isinstance(value, int):
                return False
            if self.min_value is not None and value < self.min_value:
                return False
            if self.max_value is not None and value > self.max_value:
                return False

        elif self.param_type == ParameterType.BOOL:
            if not isinstance(value, bool):
                return False

        elif self.param_type == ParameterType.STRING:
            if not isinstance(value, str):
                return False

        elif self.param_type == ParameterType.ARRAY:
            if not isinstance(value, (list, np.ndarray)):
                return False

        return True

    def to_dict(self) -> Dict:
        """Convert parameter to dictionary"""
        return {
            "name": self.name,
            "value": self.value,
            "type": self.param_type.value,
            "category": self.category.value,
            "description": self.description,
            "min_value": self.min_value,
            "max_value": self.max_value,
            "units": self.units,
            "requires_restart": self.requires_restart,
            "read_only": self.read_only,
            "default_value": self.default_value,
        }


@dataclass
class ParameterSet:
    """Collection of related parameters"""

    name: str
    description: str
    parameters: Dict[str, Parameter] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(default_factory=dict)

    def add_parameter(self, param: Parameter):
        """Add parameter to set"""
        self.parameters[param.name] = param

    def get_parameter(self, name: str) -> Optional[Parameter]:
        """Get parameter by name"""
        return self.parameters.get(name)

    def update_parameter(self, name: str, value: Any) -> bool:
        """Update parameter value with validation"""
        if name not in self.parameters:
            return False

        param = self.parameters[name]
        if param.validate(value):
            param.value = value
            return True
        return False

    def to_dict(self) -> Dict:
        """Convert parameter set to dictionary"""
        return {
            "name": self.name,
            "description": self.description,
            "parameters": {
                name: param.to_dict() for name, param in self.parameters.items()
            },
            "metadata": self.metadata,
        }


class ParameterManager:
    """
    Advanced parameter management system for flight controllers.

    Features:
    - Real-time parameter updates
    - Parameter validation and bounds checking
    - Automatic parameter backup and restore
    - Parameter sets and configurations
    - Integration with tuning systems
    - Cross-platform parameter files
    - Parameter change notifications
    """

    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize parameter manager.

        Args:
            config_path: Path to parameter configuration file
        """
        self.config_path = Path(config_path) if config_path else None
        self.parameter_sets: Dict[str, ParameterSet] = {}
        self.active_set_name: Optional[str] = None

        # Parameter change tracking
        self.change_history: List[Dict] = []
        self.change_callbacks: List[callable] = []

        # Thread safety
        self._lock = threading.RLock()

        # Auto-save configuration
        self.auto_save_enabled = True
        self.auto_save_interval = 60.0  # seconds
        self._auto_save_thread = None

        self.logger = logging.getLogger(__name__)

        # Initialize default parameter sets
        self._initialize_default_parameters()

        # Load configuration if provided
        if self.config_path and self.config_path.exists():
            self.load_configuration(self.config_path)

        # Start auto-save thread
        if self.auto_save_enabled:
            self._start_auto_save()

    def _initialize_default_parameters(self):
        """Initialize default parameter sets"""

        # Control parameters
        control_set = ParameterSet(
            name="flight_control", description="Primary flight control parameters"
        )

        # Rate controller parameters
        axes = ["roll", "pitch", "yaw"]
        for axis in axes:
            # Default gains based on research
            default_p = 0.08 if axis != "yaw" else 0.15
            default_i = 0.04 if axis != "yaw" else 0.08
            default_d = 0.006 if axis != "yaw" else 0.0

            control_set.add_parameter(
                Parameter(
                    name=f"ATC_RAT_{axis.upper()}_P",
                    value=default_p,
                    param_type=ParameterType.FLOAT,
                    category=ParameterCategory.CONTROL,
                    description=f"{axis.title()} rate controller P gain",
                    min_value=0.0,
                    max_value=2.0,
                    units="dimensionless",
                )
            )

            control_set.add_parameter(
                Parameter(
                    name=f"ATC_RAT_{axis.upper()}_I",
                    value=default_i,
                    param_type=ParameterType.FLOAT,
                    category=ParameterCategory.CONTROL,
                    description=f"{axis.title()} rate controller I gain",
                    min_value=0.0,
                    max_value=2.0,
                    units="dimensionless",
                )
            )

            control_set.add_parameter(
                Parameter(
                    name=f"ATC_RAT_{axis.upper()}_D",
                    value=default_d,
                    param_type=ParameterType.FLOAT,
                    category=ParameterCategory.CONTROL,
                    description=f"{axis.title()} rate controller D gain",
                    min_value=0.0,
                    max_value=0.5,
                    units="dimensionless",
                )
            )

            # Rate limits
            max_rate = 200.0 if axis != "yaw" else 90.0
            control_set.add_parameter(
                Parameter(
                    name=f"ATC_RAT_{axis.upper()}_MAX",
                    value=max_rate,
                    param_type=ParameterType.FLOAT,
                    category=ParameterCategory.CONTROL,
                    description=f"Maximum {axis} rate",
                    min_value=10.0,
                    max_value=500.0,
                    units="deg/s",
                )
            )

        # Attitude controller parameters
        for axis in axes:
            default_p = 6.0 if axis != "yaw" else 4.5

            control_set.add_parameter(
                Parameter(
                    name=f"ATC_ANG_{axis.upper()}_P",
                    value=default_p,
                    param_type=ParameterType.FLOAT,
                    category=ParameterCategory.CONTROL,
                    description=f"{axis.title()} attitude controller P gain",
                    min_value=0.0,
                    max_value=20.0,
                    units="dimensionless",
                )
            )

        # Add control set
        self.parameter_sets["flight_control"] = control_set

        # Safety parameters
        safety_set = ParameterSet(
            name="safety", description="Safety and limit parameters"
        )

        safety_params = [
            ("SAFETY_ANGLE_MAX", 45.0, "Maximum attitude angle", "deg", 10.0, 90.0),
            ("SAFETY_RATE_MAX", 200.0, "Maximum angular rate", "deg/s", 50.0, 500.0),
            ("SAFETY_ACCEL_MAX", 10.0, "Maximum acceleration", "m/s²", 2.0, 20.0),
            ("SAFETY_GEOFENCE_RADIUS", 1000.0, "Geofence radius", "m", 10.0, 5000.0),
            ("SAFETY_ALTITUDE_MAX", 400.0, "Maximum altitude", "m", 5.0, 1000.0),
            ("SAFETY_BATTERY_MIN", 14.4, "Minimum battery voltage", "V", 10.0, 20.0),
            (
                "SAFETY_OSCILLATION_FREQ",
                5.0,
                "Oscillation detection frequency",
                "Hz",
                1.0,
                20.0,
            ),
        ]

        for name, value, desc, units, min_val, max_val in safety_params:
            safety_set.add_parameter(
                Parameter(
                    name=name,
                    value=value,
                    param_type=ParameterType.FLOAT,
                    category=ParameterCategory.SAFETY,
                    description=desc,
                    min_value=min_val,
                    max_value=max_val,
                    units=units,
                )
            )

        self.parameter_sets["safety"] = safety_set

        # Tuning parameters
        tuning_set = ParameterSet(
            name="tuning", description="Auto-tuning configuration parameters"
        )

        tuning_params = [
            (
                "TUNE_AGGR",
                0.075,
                ParameterType.FLOAT,
                "Tuning aggressiveness",
                0.05,
                0.15,
            ),
            (
                "TUNE_METHOD",
                "relay_feedback",
                ParameterType.STRING,
                "Tuning method",
                None,
                None,
            ),
            ("TUNE_TIMEOUT", 60.0, ParameterType.FLOAT, "Tuning timeout", 10.0, 300.0),
            (
                "TUNE_ENABLE_ROLL",
                True,
                ParameterType.BOOL,
                "Enable roll tuning",
                None,
                None,
            ),
            (
                "TUNE_ENABLE_PITCH",
                True,
                ParameterType.BOOL,
                "Enable pitch tuning",
                None,
                None,
            ),
            (
                "TUNE_ENABLE_YAW",
                True,
                ParameterType.BOOL,
                "Enable yaw tuning",
                None,
                None,
            ),
        ]

        for name, value, param_type, desc, min_val, max_val in tuning_params:
            tuning_set.add_parameter(
                Parameter(
                    name=name,
                    value=value,
                    param_type=param_type,
                    category=ParameterCategory.TUNING,
                    description=desc,
                    min_value=min_val,
                    max_value=max_val,
                    units="dimensionless",
                )
            )

        self.parameter_sets["tuning"] = tuning_set

        # Set default active set
        self.active_set_name = "flight_control"

    def create_parameter_set(self, name: str, description: str = "") -> ParameterSet:
        """Create new parameter set"""
        with self._lock:
            param_set = ParameterSet(name=name, description=description)
            self.parameter_sets[name] = param_set
            self.logger.info(f"Created parameter set: {name}")
            return param_set

    def get_parameter_set(self, name: str) -> Optional[ParameterSet]:
        """Get parameter set by name"""
        return self.parameter_sets.get(name)

    def set_active_parameter_set(self, name: str) -> bool:
        """Set active parameter set"""
        if name not in self.parameter_sets:
            return False

        with self._lock:
            self.active_set_name = name
            self.logger.info(f"Active parameter set changed to: {name}")
            self._notify_change("active_set_changed", name)
            return True

    def get_parameter(
        self, name: str, set_name: Optional[str] = None
    ) -> Optional[Parameter]:
        """Get parameter by name"""
        target_set = set_name or self.active_set_name
        if target_set not in self.parameter_sets:
            return None

        return self.parameter_sets[target_set].get_parameter(name)

    def set_parameter(
        self, name: str, value: Any, set_name: Optional[str] = None
    ) -> bool:
        """Set parameter value with validation"""
        target_set = set_name or self.active_set_name
        if target_set not in self.parameter_sets:
            return False

        with self._lock:
            param_set = self.parameter_sets[target_set]
            if param_set.update_parameter(name, value):
                # Record change
                self.change_history.append(
                    {
                        "timestamp": time.time(),
                        "parameter": name,
                        "value": value,
                        "set": target_set,
                    }
                )

                # Notify callbacks
                self._notify_change("parameter_changed", name, value)

                self.logger.info(f"Parameter {name} updated to {value}")
                return True
            else:
                self.logger.warning(f"Failed to update parameter {name} = {value}")
                return False

    def get_parameter_value(self, name: str, set_name: Optional[str] = None) -> Any:
        """Get parameter value"""
        param = self.get_parameter(name, set_name)
        return param.value if param else None

    def load_gains_from_pid(
        self, gains: Dict[str, PIDGains], set_name: Optional[str] = None
    ):
        """Load PID gains into parameter set"""
        target_set = set_name or self.active_set_name

        for axis, gain in gains.items():
            axis_upper = axis.upper()
            self.set_parameter(f"ATC_RAT_{axis_upper}_P", gain.P, target_set)
            self.set_parameter(f"ATC_RAT_{axis_upper}_I", gain.I, target_set)
            self.set_parameter(f"ATC_RAT_{axis_upper}_D", gain.D, target_set)

    def export_gains_to_pid(
        self, set_name: Optional[str] = None
    ) -> Dict[str, PIDGains]:
        """Export parameters to PID gains format"""
        target_set = set_name or self.active_set_name
        gains = {}

        for axis in ["roll", "pitch", "yaw"]:
            axis_upper = axis.upper()

            p_gain = self.get_parameter_value(f"ATC_RAT_{axis_upper}_P", target_set)
            i_gain = self.get_parameter_value(f"ATC_RAT_{axis_upper}_I", target_set)
            d_gain = self.get_parameter_value(f"ATC_RAT_{axis_upper}_D", target_set)

            if p_gain is not None and i_gain is not None and d_gain is not None:
                gains[axis] = PIDGains(P=p_gain, I=i_gain, D=d_gain)

        return gains

    def backup_parameters(self, backup_name: str = None) -> str:
        """Create backup of current parameters"""
        if backup_name is None:
            backup_name = f"backup_{int(time.time())}"

        backup_data = {
            "timestamp": time.time(),
            "active_set": self.active_set_name,
            "parameter_sets": {
                name: param_set.to_dict()
                for name, param_set in self.parameter_sets.items()
            },
        }

        # Save to file
        backup_path = Path(f"{backup_name}.json")
        try:
            with open(backup_path, "w") as f:
                json.dump(backup_data, f, indent=2)
            self.logger.info(f"Parameters backed up to {backup_path}")
            return str(backup_path)
        except Exception as e:
            self.logger.error(f"Failed to backup parameters: {e}")
            return ""

    def restore_parameters(self, backup_path: str) -> bool:
        """Restore parameters from backup"""
        try:
            with open(backup_path, "r") as f:
                backup_data = json.load(f)

            # Clear current parameters
            self.parameter_sets.clear()

            # Restore parameter sets
            for set_name, set_data in backup_data["parameter_sets"].items():
                param_set = ParameterSet(
                    name=set_data["name"], description=set_data["description"]
                )

                for param_name, param_data in set_data["parameters"].items():
                    param = Parameter(
                        name=param_data["name"],
                        value=param_data["value"],
                        param_type=ParameterType(param_data["type"]),
                        category=ParameterCategory(param_data["category"]),
                        description=param_data["description"],
                        min_value=param_data["min_value"],
                        max_value=param_data["max_value"],
                        units=param_data["units"],
                        requires_restart=param_data["requires_restart"],
                        read_only=param_data["read_only"],
                        default_value=param_data["default_value"],
                    )
                    param_set.add_parameter(param)

                self.parameter_sets[set_name] = param_set

            # Restore active set
            self.active_set_name = backup_data["active_set"]

            self.logger.info(f"Parameters restored from {backup_path}")
            self._notify_change("parameters_restored", backup_path)
            return True

        except Exception as e:
            self.logger.error(f"Failed to restore parameters: {e}")
            return False

    def save_configuration(self, config_path: Optional[str] = None):
        """Save current configuration to file"""
        if config_path is None:
            config_path = self.config_path

        if config_path is None:
            self.logger.warning("No configuration path specified")
            return

        try:
            config_data = {
                "version": "1.0",
                "timestamp": time.time(),
                "active_set": self.active_set_name,
                "parameter_sets": {
                    name: param_set.to_dict()
                    for name, param_set in self.parameter_sets.items()
                },
            }

            with open(config_path, "w") as f:
                json.dump(config_data, f, indent=2)

            self.logger.info(f"Configuration saved to {config_path}")

        except Exception as e:
            self.logger.error(f"Failed to save configuration: {e}")

    def load_configuration(self, config_path: str):
        """Load configuration from file"""
        try:
            with open(config_path, "r") as f:
                config_data = json.load(f)

            # Validate version
            if config_data.get("version") != "1.0":
                self.logger.warning(
                    f"Unknown configuration version: {config_data.get('version')}"
                )

            # Clear current parameters
            self.parameter_sets.clear()

            # Load parameter sets
            for set_name, set_data in config_data["parameter_sets"].items():
                param_set = ParameterSet(
                    name=set_data["name"],
                    description=set_data["description"],
                    metadata=set_data.get("metadata", {}),
                )

                for param_name, param_data in set_data["parameters"].items():
                    param = Parameter(
                        name=param_data["name"],
                        value=param_data["value"],
                        param_type=ParameterType(param_data["type"]),
                        category=ParameterCategory(param_data["category"]),
                        description=param_data["description"],
                        min_value=param_data["min_value"],
                        max_value=param_data["max_value"],
                        units=param_data["units"],
                        requires_restart=param_data["requires_restart"],
                        read_only=param_data["read_only"],
                        default_value=param_data["default_value"],
                    )
                    param_set.add_parameter(param)

                self.parameter_sets[set_name] = param_set

            # Set active set
            self.active_set_name = config_data["active_set"]

            self.logger.info(f"Configuration loaded from {config_path}")

        except Exception as e:
            self.logger.error(f"Failed to load configuration: {e}")

    def add_change_callback(self, callback: callable):
        """Add callback for parameter changes"""
        self.change_callbacks.append(callback)

    def _notify_change(self, change_type: str, *args):
        """Notify registered callbacks of parameter changes"""
        for callback in self.change_callbacks:
            try:
                callback(change_type, *args)
            except Exception as e:
                self.logger.error(f"Change callback failed: {e}")

    def _start_auto_save(self):
        """Start auto-save thread"""

        def auto_save_loop():
            while self.auto_save_enabled:
                time.sleep(self.auto_save_interval)
                if self.config_path:
                    self.save_configuration()

        self._auto_save_thread = threading.Thread(target=auto_save_loop, daemon=True)
        self._auto_save_thread.start()

    def get_parameters_by_category(
        self, category: ParameterCategory, set_name: Optional[str] = None
    ) -> Dict[str, Parameter]:
        """Get all parameters in a specific category"""
        target_set = set_name or self.active_set_name
        if target_set not in self.parameter_sets:
            return {}

        param_set = self.parameter_sets[target_set]
        return {
            name: param
            for name, param in param_set.parameters.items()
            if param.category == category
        }

    def search_parameters(
        self, search_term: str, set_name: Optional[str] = None
    ) -> Dict[str, Parameter]:
        """Search parameters by name or description"""
        target_set = set_name or self.active_set_name
        if target_set not in self.parameter_sets:
            return {}

        param_set = self.parameter_sets[target_set]
        results = {}

        for name, param in param_set.parameters.items():
            if (
                search_term.lower() in name.lower()
                or search_term.lower() in param.description.lower()
            ):
                results[name] = param

        return results

    def validate_all_parameters(
        self, set_name: Optional[str] = None
    ) -> Dict[str, List[str]]:
        """Validate all parameters in a set"""
        target_set = set_name or self.active_set_name
        if target_set not in self.parameter_sets:
            return {}

        param_set = self.parameter_sets[target_set]
        validation_results = {}

        for name, param in param_set.parameters.items():
            errors = []

            # Type validation
            if not param.validate(param.value):
                errors.append("Value validation failed")

            # Range validation
            if param.param_type == ParameterType.FLOAT:
                if param.min_value is not None and param.value < param.min_value:
                    errors.append(
                        f"Value {param.value} below minimum {param.min_value}"
                    )
                if param.max_value is not None and param.value > param.max_value:
                    errors.append(
                        f"Value {param.value} above maximum {param.max_value}"
                    )

            if errors:
                validation_results[name] = errors

        return validation_results

    def reset_to_defaults(self, set_name: Optional[str] = None):
        """Reset parameters to default values"""
        target_set = set_name or self.active_set_name
        if target_set not in self.parameter_sets:
            return

        param_set = self.parameter_sets[target_set]

        with self._lock:
            for param in param_set.parameters.values():
                if not param.read_only:
                    param.value = param.default_value

            self.logger.info(f"Reset parameters to defaults for set: {target_set}")
            self._notify_change("parameters_reset", target_set)

    def get_status(self) -> Dict:
        """Get parameter manager status"""
        return {
            "active_set": self.active_set_name,
            "parameter_sets": list(self.parameter_sets.keys()),
            "total_parameters": sum(
                len(ps.parameters) for ps in self.parameter_sets.values()
            ),
            "recent_changes": len(
                [c for c in self.change_history if time.time() - c["timestamp"] < 3600]
            ),
            "auto_save_enabled": self.auto_save_enabled,
            "config_path": str(self.config_path) if self.config_path else None,
        }

    def shutdown(self):
        """Shutdown parameter manager"""
        self.auto_save_enabled = False
        if self._auto_save_thread and self._auto_save_thread.is_alive():
            self._auto_save_thread.join(timeout=1.0)

        # Final save
        if self.config_path:
            self.save_configuration()

        self.logger.info("Parameter manager shutdown complete")
