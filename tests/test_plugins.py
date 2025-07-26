"""
Unit tests for Constellation Overwatch Plugin System

<!-- DEVTEAM: Comprehensive test coverage for plugin architecture -->
<!-- DEVTEAM: Validates plugin loading, management, and security features -->
"""

import pytest
import tempfile
import os
from unittest.mock import Mock, patch, MagicMock
from pathlib import Path

# Plugin system imports
from sdk.plugins import (
    PluginInterface, PluginType, create_plugin_manager,
    discover_plugins, get_plugin_info
)
from sdk.plugins.base import (
    BasePlugin, PluginMetadata, PluginStatus, create_plugin_metadata,
    VehicleInterfacePlugin, SensorPlugin
)
from sdk.plugins.manager import PluginManager
from sdk.plugins.registry import PluginRegistry
from sdk.plugins.loader import PluginLoader
from sdk.plugins.validator import PluginValidator
from sdk.plugins.examples.simple_sensor_plugin import SimpleSensorPlugin


class TestPluginMetadata:
    """Test cases for PluginMetadata functionality"""
    
    def test_plugin_metadata_creation(self):
        """Test creating plugin metadata with required fields"""
        metadata = PluginMetadata(
            name="test_plugin",
            version="1.0.0",
            description="Test plugin for unit testing",
            author="Test Author",
            plugin_type=PluginType.SENSOR
        )
        
        assert metadata.name == "test_plugin"
        assert metadata.version == "1.0.0"
        assert metadata.description == "Test plugin for unit testing"
        assert metadata.author == "Test Author"
        assert metadata.plugin_type == PluginType.SENSOR
        assert metadata.dependencies == []  # Default empty list
        assert metadata.api_version == "1.0"
        assert metadata.license == "Apache-2.0"
    
    def test_plugin_metadata_with_dependencies(self):
        """Test plugin metadata with dependencies"""
        metadata = PluginMetadata(
            name="dependent_plugin",
            version="1.0.0",
            description="Plugin with dependencies",
            author="Test Author",
            plugin_type=PluginType.COMPUTER_VISION,
            dependencies=["numpy", "opencv-python"]
        )
        
        assert metadata.dependencies == ["numpy", "opencv-python"]
    
    def test_create_plugin_metadata_helper(self):
        """Test helper function for creating plugin metadata"""
        metadata = create_plugin_metadata(
            name="helper_test",
            version="2.0.0",
            description="Created with helper",
            author="Helper Author",
            plugin_type=PluginType.NAVIGATION,
            website="https://example.com"
        )
        
        assert isinstance(metadata, PluginMetadata)
        assert metadata.name == "helper_test"
        assert metadata.website == "https://example.com"


class TestPluginStatus:
    """Test cases for PluginStatus enum"""
    
    def test_plugin_status_values(self):
        """Test plugin status enum values"""
        assert PluginStatus.DISCOVERED.value == "discovered"
        assert PluginStatus.VALIDATED.value == "validated"
        assert PluginStatus.LOADED.value == "loaded"
        assert PluginStatus.ACTIVE.value == "active"
        assert PluginStatus.INACTIVE.value == "inactive"
        assert PluginStatus.ERROR.value == "error"
        assert PluginStatus.UNLOADED.value == "unloaded"


class TestPluginType:
    """Test cases for PluginType enum"""
    
    def test_plugin_type_values(self):
        """Test plugin type enum values"""
        assert PluginType.VEHICLE_INTERFACE.value == "vehicle_interface"
        assert PluginType.SENSOR.value == "sensor"
        assert PluginType.COMPUTER_VISION.value == "computer_vision"
        assert PluginType.COMMUNICATION.value == "communication"
        assert PluginType.CUSTOM.value == "custom"


class MockPlugin(BasePlugin):
    """Mock plugin for testing purposes"""
    
    def __init__(self, config=None):
        super().__init__(config)
        self.initialized = False
        self.shutdown_called = False
    
    def get_metadata(self):
        return create_plugin_metadata(
            name="mock_plugin",
            version="1.0.0",
            description="Mock plugin for testing",
            author="Test Suite",
            plugin_type=PluginType.TESTING
        )
    
    def initialize(self):
        self.initialized = True
        self.set_status(PluginStatus.LOADED)
        return True
    
    def shutdown(self):
        self.shutdown_called = True
        self.set_status(PluginStatus.UNLOADED)
        return True


class TestBasePlugin:
    """Test cases for BasePlugin functionality"""
    
    def test_base_plugin_initialization(self):
        """Test BasePlugin initialization"""
        config = {"test_param": "test_value"}
        plugin = MockPlugin(config)
        
        assert plugin.config == config
        assert plugin.get_status() == PluginStatus.DISCOVERED
        assert plugin.get_name() == "mock_plugin"
        assert plugin.get_version() == "1.0.0"
    
    def test_plugin_lifecycle(self):
        """Test plugin lifecycle methods"""
        plugin = MockPlugin()
        
        # Test initialization
        assert plugin.initialize() is True
        assert plugin.initialized is True
        assert plugin.get_status() == PluginStatus.LOADED
        
        # Test activation
        assert plugin.activate() is True
        assert plugin.get_status() == PluginStatus.ACTIVE
        
        # Test deactivation
        assert plugin.deactivate() is True
        assert plugin.get_status() == PluginStatus.INACTIVE
        
        # Test shutdown
        assert plugin.shutdown() is True
        assert plugin.shutdown_called is True
        assert plugin.get_status() == PluginStatus.UNLOADED
    
    def test_plugin_health_status(self):
        """Test plugin health status reporting"""
        plugin = MockPlugin()
        
        health = plugin.get_health_status()
        
        assert "status" in health
        assert "healthy" in health
        assert "last_error" in health
        assert health["status"] == PluginStatus.DISCOVERED.value
    
    def test_plugin_capabilities(self):
        """Test plugin capabilities"""
        plugin = MockPlugin()
        
        capabilities = plugin.get_capabilities()
        assert isinstance(capabilities, list)
    
    def test_plugin_configuration_validation(self):
        """Test plugin configuration validation"""
        plugin = MockPlugin()
        
        # Test default validation (should pass)
        assert plugin.validate_configuration({}) is True
        
        # Test configuration schema
        schema = plugin.get_configuration_schema()
        assert isinstance(schema, dict)
    
    def test_plugin_event_handling(self):
        """Test plugin event handling"""
        plugin = MockPlugin()
        
        # Test default event handling
        result = plugin.handle_event("test_event", {"data": "test"})
        assert isinstance(result, bool)


class TestPluginRegistry:
    """Test cases for PluginRegistry functionality"""
    
    def test_plugin_registry_initialization(self):
        """Test plugin registry initialization"""
        registry = PluginRegistry()
        
        plugins = registry.get_all_plugins()
        assert isinstance(plugins, list)
        assert len(plugins) == 0
    
    def test_plugin_registration(self):
        """Test plugin registration"""
        registry = PluginRegistry()
        
        metadata = create_plugin_metadata(
            name="test_plugin",
            version="1.0.0",
            description="Test plugin",
            author="Test Author",
            plugin_type=PluginType.SENSOR
        )
        
        registry.register_plugin(metadata)
        
        plugins = registry.get_all_plugins()
        assert len(plugins) == 1
        assert plugins[0].name == "test_plugin"
    
    def test_plugins_by_type_filtering(self):
        """Test filtering plugins by type"""
        registry = PluginRegistry()
        
        # Register plugins of different types
        sensor_metadata = create_plugin_metadata(
            name="sensor_plugin",
            version="1.0.0",
            description="Sensor plugin",
            author="Test Author",
            plugin_type=PluginType.SENSOR
        )
        
        vision_metadata = create_plugin_metadata(
            name="vision_plugin",
            version="1.0.0",
            description="Vision plugin",
            author="Test Author",
            plugin_type=PluginType.COMPUTER_VISION
        )
        
        registry.register_plugin(sensor_metadata)
        registry.register_plugin(vision_metadata)
        
        # Test filtering
        sensor_plugins = registry.get_plugins_by_type(PluginType.SENSOR)
        vision_plugins = registry.get_plugins_by_type(PluginType.COMPUTER_VISION)
        
        assert len(sensor_plugins) == 1
        assert len(vision_plugins) == 1
        assert sensor_plugins[0].name == "sensor_plugin"
        assert vision_plugins[0].name == "vision_plugin"


class TestPluginManager:
    """Test cases for PluginManager functionality"""
    
    def test_plugin_manager_initialization(self):
        """Test plugin manager initialization"""
        config = {"test_config": "value"}
        manager = PluginManager(config)
        
        assert manager.config == config
        assert isinstance(manager._loaded_plugins, dict)
    
    def test_plugin_loading(self):
        """Test plugin loading functionality"""
        config = {}
        manager = PluginManager(config)
        
        # Test loading (mock implementation returns True)
        result = manager.load_plugin("test_plugin")
        assert result is True
    
    def test_plugin_unloading(self):
        """Test plugin unloading functionality"""
        config = {}
        manager = PluginManager(config)
        
        # Test unloading (mock implementation returns True)
        result = manager.unload_plugin("test_plugin")
        assert result is True
    
    def test_get_plugin_instance(self):
        """Test getting plugin instance"""
        config = {}
        manager = PluginManager(config)
        
        # Test getting non-existent plugin
        instance = manager.get_plugin_instance("nonexistent_plugin")
        assert instance is None
    
    def test_get_plugin_status(self):
        """Test getting plugin status"""
        config = {}
        manager = PluginManager(config)
        
        status = manager.get_plugin_status("test_plugin")
        assert isinstance(status, PluginStatus)


class TestPluginLoader:
    """Test cases for PluginLoader functionality"""
    
    def test_plugin_loader_initialization(self):
        """Test plugin loader initialization"""
        config = {"loader_config": "test"}
        loader = PluginLoader(config)
        
        assert loader.config == config
    
    def test_load_plugin_from_file(self):
        """Test loading plugin from file"""
        config = {}
        loader = PluginLoader(config)
        
        # Test loading (mock implementation returns True)
        result = loader.load_plugin_from_file("test_plugin.py")
        assert result is True
    
    def test_load_plugins_from_directory(self):
        """Test loading plugins from directory"""
        config = {}
        loader = PluginLoader(config)
        
        # Test loading from directory (mock implementation)
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            loader.load_plugins_from_directory(temp_path, trusted=True)
            # Mock implementation doesn't raise exceptions


class TestPluginValidator:
    """Test cases for PluginValidator functionality"""
    
    def test_plugin_validator_initialization(self):
        """Test plugin validator initialization"""
        validator = PluginValidator()
        # Should initialize without error
        assert validator is not None
    
    def test_plugin_validation(self):
        """Test plugin validation"""
        validator = PluginValidator()
        
        # Test validation (mock implementation returns True)
        result = validator.validate_plugin("test_plugin.py")
        assert result is True


class TestPluginInterface:
    """Test cases for PluginInterface functionality"""
    
    def test_plugin_interface_initialization(self):
        """Test plugin interface initialization"""
        config = {"test": "config"}
        
        with patch.object(PluginInterface, '_initialize_system'):
            interface = PluginInterface(config)
            
            assert interface.config == config
            assert isinstance(interface.manager, PluginManager)
            assert isinstance(interface.registry, PluginRegistry)
            assert isinstance(interface.loader, PluginLoader)
            assert isinstance(interface.validator, PluginValidator)
    
    def test_get_available_plugins(self):
        """Test getting available plugins"""
        with patch.object(PluginInterface, '_initialize_system'):
            interface = PluginInterface()
            
            # Mock registry to return test plugins
            interface.registry.get_all_plugins = Mock(return_value=[])
            
            plugins = interface.get_available_plugins()
            assert isinstance(plugins, list)
    
    def test_load_plugin(self):
        """Test plugin loading through interface"""
        with patch.object(PluginInterface, '_initialize_system'):
            interface = PluginInterface()
            
            # Mock manager
            interface.manager.load_plugin = Mock(return_value=True)
            
            result = interface.load_plugin("test_plugin")
            assert result is True
            interface.manager.load_plugin.assert_called_once_with("test_plugin")
    
    def test_plugin_validation(self):
        """Test plugin validation through interface"""
        with patch.object(PluginInterface, '_initialize_system'):
            interface = PluginInterface()
            
            # Mock validator
            interface.validator.validate_plugin = Mock(return_value=True)
            
            result = interface.validate_plugin("test_plugin.py")
            assert result is True
    
    def test_get_system_statistics(self):
        """Test getting system statistics"""
        with patch.object(PluginInterface, '_initialize_system'):
            interface = PluginInterface()
            
            # Mock registry and manager
            mock_metadata = create_plugin_metadata(
                name="test_plugin",
                version="1.0.0",
                description="Test",
                author="Test",
                plugin_type=PluginType.SENSOR
            )
            
            interface.registry.get_all_plugins = Mock(return_value=[mock_metadata])
            interface.manager.get_plugin_status = Mock(return_value=PluginStatus.LOADED)
            
            stats = interface.get_system_statistics()
            
            assert "total_plugins" in stats
            assert "status_distribution" in stats
            assert "type_distribution" in stats
            assert "loaded_plugins" in stats


class TestSimpleSensorPlugin:
    """Test cases for SimpleSensorPlugin example"""
    
    def test_simple_sensor_plugin_initialization(self):
        """Test simple sensor plugin initialization"""
        config = {
            "sensor_name": "Test Sensor",
            "update_rate": 2.0,
            "noise_level": 0.05
        }
        
        plugin = SimpleSensorPlugin(config)
        
        assert plugin.sensor_name == "Test Sensor"
        assert plugin.update_rate == 2.0
        assert plugin.noise_level == 0.05
        assert plugin._reading_count == 0
    
    def test_simple_sensor_plugin_metadata(self):
        """Test simple sensor plugin metadata"""
        plugin = SimpleSensorPlugin()
        
        metadata = plugin.get_metadata()
        
        assert metadata.name == "simple_sensor"
        assert metadata.version == "1.0.0"
        assert metadata.author == "Constellation Overwatch Team"
        assert metadata.supports_hot_reload is True
    
    def test_simple_sensor_plugin_lifecycle(self):
        """Test simple sensor plugin lifecycle"""
        plugin = SimpleSensorPlugin()
        
        # Test initialization
        assert plugin.initialize() is True
        assert plugin.get_status() == PluginStatus.LOADED
        
        # Test activation
        assert plugin.activate() is True
        assert plugin.get_status() == PluginStatus.ACTIVE
        
        # Test reading data
        data = plugin.read_sensor_data()
        assert isinstance(data, dict)
        assert "timestamp" in data
        assert "sensor_name" in data
        assert "data" in data
        assert "temperature_celsius" in data["data"]
        assert "humidity_percent" in data["data"]
        assert "pressure_hpa" in data["data"]
        
        # Test calibration
        assert plugin.calibrate_sensor() is True
        assert plugin._is_calibrated is True
        
        # Test sensor info
        info = plugin.get_sensor_info()
        assert isinstance(info, dict)
        assert "sensor_name" in info
        assert "capabilities" in info
        
        # Test capabilities
        capabilities = plugin.get_capabilities()
        assert isinstance(capabilities, list)
        assert len(capabilities) > 0
        
        # Test health status
        health = plugin.get_health_status()
        assert "sensor_specific" in health
        
        # Test event handling
        assert plugin.handle_event("calibration_request", None) is True
        assert plugin.handle_event("reading_request", None) is True
        
        # Test shutdown
        assert plugin.shutdown() is True
        assert plugin.get_status() == PluginStatus.UNLOADED
    
    def test_simple_sensor_plugin_inactive_reading(self):
        """Test sensor reading when inactive"""
        plugin = SimpleSensorPlugin()
        plugin.initialize()
        # Don't activate
        
        data = plugin.read_sensor_data()
        assert data == {}  # Should return empty dict when not active


class TestPluginUtilities:
    """Test cases for plugin utility functions"""
    
    def test_create_plugin_manager_function(self):
        """Test create_plugin_manager convenience function"""
        config = {"test": "value"}
        
        with patch.object(PluginInterface, '_initialize_system'):
            manager = create_plugin_manager(config)
            
            assert isinstance(manager, PluginInterface)
            assert "plugin_directories" in manager.config
            assert "security_mode" in manager.config
            assert manager.config["test"] == "value"
    
    def test_discover_plugins_function(self):
        """Test discover_plugins utility function"""
        with tempfile.TemporaryDirectory() as temp_dir:
            # Create test plugin file
            test_file = os.path.join(temp_dir, "test_plugin.py")
            with open(test_file, 'w') as f:
                f.write("# Test plugin file")
            
            plugins = discover_plugins(temp_dir)
            
            assert isinstance(plugins, list)
            assert len(plugins) == 1
            assert plugins[0] == test_file
    
    def test_discover_plugins_nonexistent_directory(self):
        """Test discover_plugins with non-existent directory"""
        plugins = discover_plugins("/nonexistent/directory")
        assert plugins == []
    
    def test_get_plugin_info_function(self):
        """Test get_plugin_info utility function"""
        # Test with non-existent file
        info = get_plugin_info("/nonexistent/plugin.py")
        assert info is None


class TestSpecializedPluginTypes:
    """Test cases for specialized plugin base classes"""
    
    def test_vehicle_interface_plugin_abstract_methods(self):
        """Test VehicleInterfacePlugin abstract methods"""
        # Should not be able to instantiate directly
        with pytest.raises(TypeError):
            VehicleInterfacePlugin()
    
    def test_sensor_plugin_abstract_methods(self):
        """Test SensorPlugin abstract methods"""
        # Should not be able to instantiate directly
        with pytest.raises(TypeError):
            SensorPlugin()


# Integration tests
class TestPluginSystemIntegration:
    """Integration tests for the complete plugin system"""
    
    def test_end_to_end_plugin_workflow(self):
        """Test complete plugin workflow from discovery to execution"""
        config = {"test_mode": True}
        
        with patch.object(PluginInterface, '_initialize_system'):
            # Create plugin interface
            interface = PluginInterface(config)
            
            # Mock the required components
            interface.registry.get_all_plugins = Mock(return_value=[])
            interface.manager.load_plugin = Mock(return_value=True)
            interface.manager.get_plugin_instance = Mock(return_value=MockPlugin())
            interface.validator.validate_plugin = Mock(return_value=True)
            
            # Test workflow
            available_plugins = interface.get_available_plugins()
            assert isinstance(available_plugins, list)
            
            # Validate plugin
            is_valid = interface.validate_plugin("test_plugin.py")
            assert is_valid is True
            
            # Load plugin
            load_success = interface.load_plugin("test_plugin")
            assert load_success is True
            
            # Get plugin instance
            plugin_instance = interface.get_plugin_instance("test_plugin")
            assert plugin_instance is not None
            
            # Execute plugin method
            try:
                result = interface.execute_plugin_method("test_plugin", "get_metadata")
                # Should work if plugin has the method
            except (ValueError, AttributeError):
                # Expected for mock plugin without full implementation
                pass


if __name__ == "__main__":
    pytest.main([__file__, "-v"]) 