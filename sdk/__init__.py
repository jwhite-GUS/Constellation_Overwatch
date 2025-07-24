"""
Constellation Overwatch SDK - Professional Autonomy Platform

A comprehensive SDK for autonomous systems integration, providing
entity management, message bus communication, and real-time coordination
for unmanned systems operations.
"""

__version__ = "1.0.0"
__author__ = "Galaxy Unmanned Systems LLC"
__license__ = "Apache-2.0"

# Core imports for easy access
from sdk.core.entity_manager import EntityManager, Entity
from sdk.core.message_bus import MessageBus, MessageType, Message

__all__ = [
    "EntityManager",
    "Entity",
    "MessageBus",
    "MessageType",
    "Message",
    "__version__",
]
