"""
Constellation Overwatch SDK - Base Plugin Classes

<!-- DEVTEAM: Core plugin base classes and interfaces -->
<!-- DEVTEAM: Defines plugin contract and lifecycle management -->
"""

from typing import Dict, List, Optional, Any
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class PluginStatus(Enum):
    """Plugin lifecycle status states"""
    
    DISCOVERED = "discovered"
    VALIDATED = "validated"
    LOADED = "loaded"
    ACTIVE = "active"
    INACTIVE = "inactive"
    ERROR = "error"
    UNLOADED = "unloaded"


@dataclass
class PluginMetadata:
    """Plugin metadata and information"""
    
    name: str
    version: str
    description: str
    author: str
    plugin_type: 'PluginType'
    dependencies: List[str] = None
    api_version: str = "1.0"
    license: str = "Apache-2.0"
    website: Optional[str] = None
    supports_hot_reload: bool = False
    
    def __post_init__(self):
        if self.dependencies is None:
            self.dependencies = []


class BasePlugin(ABC):
    """
    Base class for all Constellation Overwatch plugins
    
    All plugins must inherit from this class and implement the required
    abstract methods to participate in the plugin system.
    """
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize the plugin
        
        Args:
            config: Plugin-specific configuration
        """
        self.config = config or {}
        self._status = PluginStatus.DISCOVERED
        self._logger = logging.getLogger(f"plugin.{self.get_name()}")
        self._events = []
        
    @abstractmethod
    def get_metadata(self) -> PluginMetadata:
        """
        Get plugin metadata
        
        Returns:
            Plugin metadata object
        """
        pass
    
    @abstractmethod
    def initialize(self) -> bool:
        """
        Initialize the plugin
        
        Called when the plugin is loaded. Should perform any setup
        required for the plugin to function.
        
        Returns:
            True if initialization succeeded
        """
        pass
    
    @abstractmethod
    def shutdown(self) -> bool:
        """
        Shutdown the plugin
        
        Called when the plugin is being unloaded. Should clean up
        any resources used by the plugin.
        
        Returns:
            True if shutdown succeeded
        """
        pass
    
    def get_name(self) -> str:
        """Get plugin name from metadata"""
        return self.get_metadata().name
    
    def get_version(self) -> str:
        """Get plugin version from metadata"""
        return self.get_metadata().version
    
    def get_status(self) -> PluginStatus:
        """Get current plugin status"""
        return self._status
    
    def set_status(self, status: PluginStatus):
        """Set plugin status"""
        old_status = self._status
        self._status = status
        self._logger.debug(f"Status changed from {old_status.value} to {status.value}")
    
    def activate(self) -> bool:
        """
        Activate the plugin
        
        Called to make the plugin active and ready for use.
        Override if plugin needs activation logic.
        
        Returns:
            True if activation succeeded
        """
        self.set_status(PluginStatus.ACTIVE)
        return True
    
    def deactivate(self) -> bool:
        """
        Deactivate the plugin
        
        Called to make the plugin inactive but keep it loaded.
        Override if plugin needs deactivation logic.
        
        Returns:
            True if deactivation succeeded
        """
        self.set_status(PluginStatus.INACTIVE)
        return True
    
    def get_capabilities(self) -> List[str]:
        """
        Get list of capabilities provided by this plugin
        
        Returns:
            List of capability names
        """
        return []
    
    def handle_event(self, event_type: str, event_data: Any) -> bool:
        """
        Handle system events
        
        Args:
            event_type: Type of event
            event_data: Event-specific data
            
        Returns:
            True if event was handled
        """
        return False
    
    def get_configuration_schema(self) -> Dict[str, Any]:
        """
        Get configuration schema for this plugin
        
        Returns:
            JSON schema for plugin configuration
        """
        return {}
    
    def validate_configuration(self, config: Dict[str, Any]) -> bool:
        """
        Validate plugin configuration
        
        Args:
            config: Configuration to validate
            
        Returns:
            True if configuration is valid
        """
        return True
    
    def get_health_status(self) -> Dict[str, Any]:
        """
        Get plugin health status
        
        Returns:
            Health status information
        """
        return {
            "status": self._status.value,
            "healthy": self._status in [PluginStatus.ACTIVE, PluginStatus.INACTIVE],
            "last_error": None
        }


class VehicleInterfacePlugin(BasePlugin):
    """Base class for vehicle interface plugins"""
    
    @abstractmethod
    def connect_vehicle(self, connection_params: Dict[str, Any]) -> bool:
        """Connect to a vehicle"""
        pass
    
    @abstractmethod
    def disconnect_vehicle(self) -> bool:
        """Disconnect from vehicle"""
        pass
    
    @abstractmethod
    def send_command(self, command: Dict[str, Any]) -> bool:
        """Send command to vehicle"""
        pass
    
    @abstractmethod
    def get_telemetry(self) -> Dict[str, Any]:
        """Get current vehicle telemetry"""
        pass


class SensorPlugin(BasePlugin):
    """Base class for sensor plugins"""
    
    @abstractmethod
    def read_sensor_data(self) -> Dict[str, Any]:
        """Read data from sensor"""
        pass
    
    @abstractmethod
    def calibrate_sensor(self) -> bool:
        """Calibrate the sensor"""
        pass
    
    @abstractmethod
    def get_sensor_info(self) -> Dict[str, Any]:
        """Get sensor information"""
        pass


class ComputerVisionPlugin(BasePlugin):
    """Base class for computer vision plugins"""
    
    @abstractmethod
    def process_image(self, image_data: Any) -> Dict[str, Any]:
        """Process image data"""
        pass
    
    @abstractmethod
    def get_supported_formats(self) -> List[str]:
        """Get supported image formats"""
        pass


class CommunicationPlugin(BasePlugin):
    """Base class for communication plugins"""
    
    @abstractmethod
    def establish_connection(self, endpoint: str) -> bool:
        """Establish communication connection"""
        pass
    
    @abstractmethod
    def send_message(self, message: Dict[str, Any]) -> bool:
        """Send message"""
        pass
    
    @abstractmethod
    def receive_message(self) -> Optional[Dict[str, Any]]:
        """Receive message"""
        pass


def create_plugin_metadata(
    name: str,
    version: str,
    description: str,
    author: str,
    plugin_type: 'PluginType',
    **kwargs
) -> PluginMetadata:
    """
    Helper function to create plugin metadata
    
    Args:
        name: Plugin name
        version: Plugin version
        description: Plugin description
        author: Plugin author
        plugin_type: Type of plugin
        **kwargs: Additional metadata fields
        
    Returns:
        PluginMetadata instance
    """
    return PluginMetadata(
        name=name,
        version=version,
        description=description,
        author=author,
        plugin_type=plugin_type,
        **kwargs
    ) 