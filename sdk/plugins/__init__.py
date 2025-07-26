"""
Constellation Overwatch SDK - Plugin System

<!-- DEVTEAM: Extensible plugin architecture for third-party integrations -->
<!-- DEVTEAM: Provides secure, sandboxed environment for community contributions -->

Plugin system for extending Constellation Overwatch functionality through
third-party modules and community-contributed capabilities.
"""

from typing import Dict, List, Optional, Any, Type, Callable
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
import logging
import importlib
import inspect
import os
from pathlib import Path

logger = logging.getLogger(__name__)

# Core plugin components
from .base import BasePlugin, PluginMetadata, PluginStatus
from .manager import PluginManager
from .registry import PluginRegistry
from .loader import PluginLoader
from .validator import PluginValidator

__all__ = [
    'BasePlugin',
    'PluginMetadata', 
    'PluginStatus',
    'PluginManager',
    'PluginRegistry',
    'PluginLoader',
    'PluginValidator',
    'PluginType',
    'PluginInterface',
    'create_plugin_manager'
]


class PluginType(Enum):
    """Types of plugins supported by the system"""
    
    # Core System Plugins
    VEHICLE_INTERFACE = "vehicle_interface"
    COMMUNICATION = "communication"
    NAVIGATION = "navigation"
    SENSOR = "sensor"
    
    # AI/ML Plugins
    COMPUTER_VISION = "computer_vision"
    DECISION_MAKING = "decision_making"
    NATURAL_LANGUAGE = "natural_language"
    MACHINE_LEARNING = "machine_learning"
    
    # Integration Plugins
    HARDWARE_DRIVER = "hardware_driver"
    PROTOCOL_ADAPTER = "protocol_adapter"
    DATA_PROCESSOR = "data_processor"
    VISUALIZATION = "visualization"
    
    # Utility Plugins
    SECURITY = "security"
    MONITORING = "monitoring"
    TESTING = "testing"
    LOGGING = "logging"
    
    # Custom
    CUSTOM = "custom"


class PluginInterface:
    """
    Main interface for plugin system operations
    
    Provides unified access to plugin management, loading, and execution
    capabilities for the Constellation Overwatch SDK.
    """
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize plugin interface
        
        Args:
            config: Plugin system configuration
        """
        self.config = config or {}
        self.manager = PluginManager(self.config)
        self.registry = PluginRegistry()
        self.loader = PluginLoader(self.config)
        self.validator = PluginValidator()
        
        self._initialize_system()
        logger.info("Plugin interface initialized")
    
    def _initialize_system(self):
        """Initialize the plugin system"""
        try:
            # Load built-in plugins
            self._load_builtin_plugins()
            
            # Scan for external plugins
            self._scan_external_plugins()
            
            logger.info(f"Plugin system initialized with {len(self.get_available_plugins())} plugins")
            
        except Exception as e:
            logger.error(f"Failed to initialize plugin system: {e}")
            raise
    
    def _load_builtin_plugins(self):
        """Load built-in system plugins"""
        builtin_path = Path(__file__).parent / "builtin"
        if builtin_path.exists():
            self.loader.load_plugins_from_directory(builtin_path, trusted=True)
    
    def _scan_external_plugins(self):
        """Scan for external plugins in configured directories"""
        plugin_dirs = self.config.get("plugin_directories", [])
        
        for plugin_dir in plugin_dirs:
            if os.path.exists(plugin_dir):
                self.loader.load_plugins_from_directory(plugin_dir, trusted=False)
    
    def get_available_plugins(self) -> List[PluginMetadata]:
        """
        Get list of all available plugins
        
        Returns:
            List of plugin metadata
        """
        return self.registry.get_all_plugins()
    
    def get_plugins_by_type(self, plugin_type: PluginType) -> List[PluginMetadata]:
        """
        Get plugins filtered by type
        
        Args:
            plugin_type: Type of plugins to retrieve
            
        Returns:
            List of matching plugin metadata
        """
        return self.registry.get_plugins_by_type(plugin_type)
    
    def load_plugin(self, plugin_name: str) -> bool:
        """
        Load a specific plugin
        
        Args:
            plugin_name: Name of plugin to load
            
        Returns:
            True if loaded successfully
        """
        try:
            return self.manager.load_plugin(plugin_name)
        except Exception as e:
            logger.error(f"Failed to load plugin {plugin_name}: {e}")
            return False
    
    def unload_plugin(self, plugin_name: str) -> bool:
        """
        Unload a specific plugin
        
        Args:
            plugin_name: Name of plugin to unload
            
        Returns:
            True if unloaded successfully
        """
        try:
            return self.manager.unload_plugin(plugin_name)
        except Exception as e:
            logger.error(f"Failed to unload plugin {plugin_name}: {e}")
            return False
    
    def get_plugin_instance(self, plugin_name: str) -> Optional[BasePlugin]:
        """
        Get instance of a loaded plugin
        
        Args:
            plugin_name: Name of plugin
            
        Returns:
            Plugin instance or None if not loaded
        """
        return self.manager.get_plugin_instance(plugin_name)
    
    def execute_plugin_method(
        self, 
        plugin_name: str, 
        method_name: str, 
        *args, 
        **kwargs
    ) -> Any:
        """
        Execute a method on a loaded plugin
        
        Args:
            plugin_name: Name of plugin
            method_name: Method to execute
            *args: Positional arguments
            **kwargs: Keyword arguments
            
        Returns:
            Method result
        """
        plugin = self.get_plugin_instance(plugin_name)
        if plugin is None:
            raise ValueError(f"Plugin {plugin_name} not loaded")
        
        if not hasattr(plugin, method_name):
            raise AttributeError(f"Plugin {plugin_name} has no method {method_name}")
        
        method = getattr(plugin, method_name)
        return method(*args, **kwargs)
    
    def validate_plugin(self, plugin_path: str) -> bool:
        """
        Validate a plugin before loading
        
        Args:
            plugin_path: Path to plugin file
            
        Returns:
            True if plugin is valid
        """
        return self.validator.validate_plugin(plugin_path)
    
    def get_plugin_status(self, plugin_name: str) -> PluginStatus:
        """
        Get current status of a plugin
        
        Args:
            plugin_name: Name of plugin
            
        Returns:
            Plugin status
        """
        return self.manager.get_plugin_status(plugin_name)
    
    def get_system_statistics(self) -> Dict[str, Any]:
        """Get plugin system statistics"""
        all_plugins = self.get_available_plugins()
        
        status_counts = {}
        type_counts = {}
        
        for plugin in all_plugins:
            # Count by status
            status = self.get_plugin_status(plugin.name)
            status_counts[status.value] = status_counts.get(status.value, 0) + 1
            
            # Count by type
            type_counts[plugin.plugin_type.value] = type_counts.get(plugin.plugin_type.value, 0) + 1
        
        return {
            "total_plugins": len(all_plugins),
            "status_distribution": status_counts,
            "type_distribution": type_counts,
            "loaded_plugins": len([p for p in all_plugins if self.get_plugin_status(p.name) == PluginStatus.LOADED])
        }


def create_plugin_manager(config: Optional[Dict[str, Any]] = None) -> PluginInterface:
    """
    Create a plugin interface with common configuration
    
    Args:
        config: Optional configuration dictionary
        
    Returns:
        Configured PluginInterface instance
    """
    default_config = {
        "plugin_directories": [
            "./plugins",
            os.path.expanduser("~/.constellation/plugins"),
            "/usr/local/lib/constellation/plugins"
        ],
        "security_mode": "strict",
        "auto_load_builtin": True,
        "validate_signatures": True
    }
    
    if config:
        default_config.update(config)
    
    return PluginInterface(default_config)


# Plugin discovery utilities
def discover_plugins(directory: str) -> List[str]:
    """
    Discover plugin files in a directory
    
    Args:
        directory: Directory to search
        
    Returns:
        List of plugin file paths
    """
    plugin_files = []
    
    if not os.path.exists(directory):
        return plugin_files
    
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith('.py') and not file.startswith('__'):
                plugin_files.append(os.path.join(root, file))
    
    return plugin_files


def get_plugin_info(plugin_path: str) -> Optional[Dict[str, Any]]:
    """
    Extract plugin information from a plugin file
    
    Args:
        plugin_path: Path to plugin file
        
    Returns:
        Plugin information dictionary or None
    """
    try:
        # Import the plugin module
        spec = importlib.util.spec_from_file_location("plugin_module", plugin_path)
        if spec is None or spec.loader is None:
            return None
        
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        
        # Look for plugin class
        plugin_class = None
        for name, obj in inspect.getmembers(module):
            if (inspect.isclass(obj) and 
                issubclass(obj, BasePlugin) and 
                obj != BasePlugin):
                plugin_class = obj
                break
        
        if plugin_class is None:
            return None
        
        # Extract metadata
        if hasattr(plugin_class, 'get_metadata'):
            metadata = plugin_class.get_metadata()
            return {
                "name": metadata.name,
                "version": metadata.version,
                "description": metadata.description,
                "plugin_type": metadata.plugin_type.value,
                "author": metadata.author,
                "class_name": plugin_class.__name__
            }
        
        return None
        
    except Exception as e:
        logger.error(f"Failed to extract plugin info from {plugin_path}: {e}")
        return None 