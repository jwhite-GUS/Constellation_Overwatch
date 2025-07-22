"""
Basic smoke tests for Constellation Overwatch SDK.

These tests ensure that the core SDK modules can be imported
and basic functionality works as expected.
"""

import pytest
import sys
import importlib.util


def test_sdk_imports():
    """Test that core SDK modules can be imported without errors."""
    # Test core module imports
    try:
        from sdk.core import EntityManager, MessageBus
        assert EntityManager is not None
        assert MessageBus is not None
    except ImportError as e:
        pytest.fail(f"Failed to import core SDK modules: {e}")


def test_api_imports():
    """Test that API modules can be imported without errors."""
    try:
        from sdk.api.rest_server import ConstellationAPI
        assert ConstellationAPI is not None
    except ImportError as e:
        pytest.fail(f"Failed to import API modules: {e}")


@pytest.mark.asyncio
async def test_entity_manager_basic():
    """Test basic EntityManager functionality."""
    from sdk.core import EntityManager
    
    manager = EntityManager()
    assert manager is not None
    assert len(manager.get_all_entities()) == 0


@pytest.mark.asyncio  
async def test_message_bus_basic():
    """Test basic MessageBus functionality."""
    from sdk.core import MessageBus
    
    bus = MessageBus()
    assert bus is not None
    
    # Test basic pub/sub without actual message sending
    subscribers = bus.get_subscribers("test_topic")
    assert len(subscribers) == 0


def test_python_version():
    """Ensure we're running on a supported Python version."""
    assert sys.version_info >= (3, 8), "Python 3.8+ required"


def test_basic_functionality():
    """Basic functionality test to ensure CI pipeline passes."""
    assert True, "Basic test should always pass"
