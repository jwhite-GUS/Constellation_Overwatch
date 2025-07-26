"""
Constellation Overwatch SDK - Simple Sensor Plugin Example

<!-- DEVTEAM: Example sensor plugin demonstrating plugin development patterns -->
<!-- DEVTEAM: Serves as template for community-contributed sensor plugins -->

Example sensor plugin that demonstrates how to create a sensor plugin
for the Constellation Overwatch plugin system.
"""

from typing import Dict, Any, List
import random
import time
import logging

from ..base import SensorPlugin, PluginMetadata, create_plugin_metadata
from ..base import PluginStatus

logger = logging.getLogger(__name__)


class SimpleSensorPlugin(SensorPlugin):
    """
    Example sensor plugin that simulates a simple environmental sensor
    
    This plugin demonstrates the basic plugin architecture and can be used
    as a template for developing actual sensor plugins.
    """
    
    def __init__(self, config: Dict[str, Any] = None):
        """Initialize the simple sensor plugin"""
        super().__init__(config)
        
        # Plugin-specific configuration
        self.sensor_name = self.config.get("sensor_name", "Environmental Sensor")
        self.update_rate = self.config.get("update_rate", 1.0)  # Hz
        self.noise_level = self.config.get("noise_level", 0.1)
        
        # Sensor state
        self._is_calibrated = False
        self._last_reading_time = 0
        self._reading_count = 0
        
        logger.info(f"Simple sensor plugin initialized: {self.sensor_name}")
    
    def get_metadata(self) -> PluginMetadata:
        """Get plugin metadata"""
        return create_plugin_metadata(
            name="simple_sensor",
            version="1.0.0",
            description="Example environmental sensor plugin for demonstration",
            author="Constellation Overwatch Team", 
            plugin_type=None,  # Will be set by import
            dependencies=[],
            supports_hot_reload=True,
            website="https://github.com/jwhite-GUS/Constellation_Overwatch"
        )
    
    def initialize(self) -> bool:
        """Initialize the sensor plugin"""
        try:
            logger.info(f"Initializing {self.sensor_name}")
            
            # Simulate sensor initialization
            time.sleep(0.1)  # Simulate hardware initialization delay
            
            # Reset counters
            self._reading_count = 0
            self._last_reading_time = time.time()
            
            self.set_status(PluginStatus.LOADED)
            logger.info(f"{self.sensor_name} initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize {self.sensor_name}: {e}")
            self.set_status(PluginStatus.ERROR)
            return False
    
    def shutdown(self) -> bool:
        """Shutdown the sensor plugin"""
        try:
            logger.info(f"Shutting down {self.sensor_name}")
            
            # Simulate cleanup
            self._is_calibrated = False
            
            self.set_status(PluginStatus.UNLOADED)
            logger.info(f"{self.sensor_name} shut down successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to shutdown {self.sensor_name}: {e}")
            return False
    
    def read_sensor_data(self) -> Dict[str, Any]:
        """Read simulated sensor data"""
        if self.get_status() != PluginStatus.ACTIVE:
            logger.warning("Sensor not active, cannot read data")
            return {}
        
        current_time = time.time()
        
        # Simulate sensor readings with some noise
        base_temperature = 22.0  # Celsius
        base_humidity = 45.0     # Percentage
        base_pressure = 1013.25  # hPa
        
        # Add some realistic variation
        temp_noise = random.uniform(-self.noise_level, self.noise_level) * 5
        humidity_noise = random.uniform(-self.noise_level, self.noise_level) * 10
        pressure_noise = random.uniform(-self.noise_level, self.noise_level) * 10
        
        sensor_data = {
            "timestamp": current_time,
            "sensor_name": self.sensor_name,
            "reading_id": self._reading_count,
            "data": {
                "temperature_celsius": round(base_temperature + temp_noise, 2),
                "humidity_percent": round(max(0, min(100, base_humidity + humidity_noise)), 2),
                "pressure_hpa": round(base_pressure + pressure_noise, 2)
            },
            "metadata": {
                "calibrated": self._is_calibrated,
                "noise_level": self.noise_level,
                "quality": "good" if self._is_calibrated else "uncalibrated"
            }
        }
        
        self._reading_count += 1
        self._last_reading_time = current_time
        
        logger.debug(f"Sensor reading {self._reading_count}: {sensor_data['data']}")
        return sensor_data
    
    def calibrate_sensor(self) -> bool:
        """Calibrate the simulated sensor"""
        try:
            logger.info(f"Calibrating {self.sensor_name}")
            
            # Simulate calibration process
            time.sleep(0.5)  # Simulate calibration time
            
            self._is_calibrated = True
            logger.info(f"{self.sensor_name} calibration complete")
            return True
            
        except Exception as e:
            logger.error(f"Calibration failed for {self.sensor_name}: {e}")
            return False
    
    def get_sensor_info(self) -> Dict[str, Any]:
        """Get sensor information and status"""
        return {
            "sensor_name": self.sensor_name,
            "sensor_type": "environmental",
            "manufacturer": "Constellation Overwatch (Simulated)",
            "model": "ENV-SIM-001",
            "capabilities": ["temperature", "humidity", "pressure"],
            "units": {
                "temperature": "celsius", 
                "humidity": "percent",
                "pressure": "hpa"
            },
            "status": {
                "calibrated": self._is_calibrated,
                "reading_count": self._reading_count,
                "last_reading_time": self._last_reading_time,
                "update_rate_hz": self.update_rate
            },
            "configuration": {
                "noise_level": self.noise_level,
                "update_rate": self.update_rate
            }
        }
    
    def get_capabilities(self) -> List[str]:
        """Get list of sensor capabilities"""
        return [
            "temperature_reading",
            "humidity_reading", 
            "pressure_reading",
            "calibration",
            "status_monitoring"
        ]
    
    def activate(self) -> bool:
        """Activate the sensor for data collection"""
        if self.get_status() != PluginStatus.LOADED:
            logger.warning("Cannot activate sensor - not properly loaded")
            return False
        
        logger.info(f"Activating {self.sensor_name}")
        self.set_status(PluginStatus.ACTIVE)
        return True
    
    def deactivate(self) -> bool:
        """Deactivate the sensor"""
        logger.info(f"Deactivating {self.sensor_name}")
        self.set_status(PluginStatus.INACTIVE)
        return True
    
    def get_health_status(self) -> Dict[str, Any]:
        """Get sensor health status"""
        health_info = super().get_health_status()
        
        # Add sensor-specific health information
        health_info.update({
            "sensor_specific": {
                "calibrated": self._is_calibrated,
                "readings_collected": self._reading_count,
                "time_since_last_reading": time.time() - self._last_reading_time,
                "configured_properly": bool(self.sensor_name and self.update_rate > 0)
            }
        })
        
        return health_info
    
    def handle_event(self, event_type: str, event_data: Any) -> bool:
        """Handle system events"""
        if event_type == "calibration_request":
            return self.calibrate_sensor()
        elif event_type == "reading_request":
            # Could store the reading in event_data or trigger callback
            reading = self.read_sensor_data()
            return bool(reading)
        else:
            return super().handle_event(event_type, event_data)


# Plugin factory function (optional convenience function)
def create_plugin(config: Dict[str, Any] = None) -> SimpleSensorPlugin:
    """
    Factory function to create plugin instance
    
    Args:
        config: Plugin configuration
        
    Returns:
        SimpleSensorPlugin instance
    """
    return SimpleSensorPlugin(config)


# Plugin registration information
PLUGIN_CLASS = SimpleSensorPlugin
PLUGIN_NAME = "simple_sensor"
PLUGIN_VERSION = "1.0.0" 