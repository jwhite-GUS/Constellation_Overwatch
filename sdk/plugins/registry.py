"""
Constellation Overwatch SDK - Plugin Registry

<!-- DEVTEAM: Plugin discovery and metadata management -->
"""

from typing import List, Dict, Any
import logging
from .base import PluginMetadata

logger = logging.getLogger(__name__)


class PluginRegistry:
    """Registry for discovered plugins and their metadata"""
    
    def __init__(self):
        self._plugins: List[PluginMetadata] = []
        logger.info("Plugin registry initialized")
    
    def get_all_plugins(self) -> List[PluginMetadata]:
        """Get all registered plugins"""
        return self._plugins.copy()
    
    def get_plugins_by_type(self, plugin_type) -> List[PluginMetadata]:
        """Get plugins filtered by type"""
        return [p for p in self._plugins if p.plugin_type == plugin_type]
    
    def register_plugin(self, metadata: PluginMetadata):
        """Register a plugin in the registry"""
        self._plugins.append(metadata)
        logger.info(f"Registered plugin: {metadata.name}") 