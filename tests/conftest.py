"""
Test configuration and fixtures for Constellation Overwatch SDK tests.

This module provides common test utilities and fixtures for testing
the Constellation Overwatch autonomy SDK components.
"""

import pytest
import asyncio
from typing import Generator


@pytest.fixture(scope="session")
def event_loop() -> Generator[asyncio.AbstractEventLoop, None, None]:
    """Create an instance of the default event loop for the test session."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


@pytest.fixture
def sample_entity_data():
    """Provide sample entity data for testing."""
    return {
        "entity_id": "test_drone_001",
        "entity_type": "drone",
        "position": {"x": 10.0, "y": 20.0, "z": 30.0},
        "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
        "status": "active",
        "mission_id": "test_mission_001"
    }
