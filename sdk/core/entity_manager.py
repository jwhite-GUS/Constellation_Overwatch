"""
Constellation Overwatch SDK - Core Entity Manager
Basic implementation following industry standard patterns from API analysis.
"""

from typing import Dict, List, Optional, Any, Set, Callable
from dataclasses import dataclass, field
from enum import Enum
import asyncio
import uuid
import time
import json
from abc import ABC, abstractmethod


class EntityType(Enum):
    """Standard entity types following comprehensive API analysis"""

    UNKNOWN = "unknown"
    # Air Domain
    AIRCRAFT_FIXED_WING = "aircraft_fixed_wing"
    AIRCRAFT_MULTIROTOR = "aircraft_multirotor"
    AIRCRAFT_VTOL = "aircraft_vtol"
    AIRCRAFT_HELICOPTER = "aircraft_helicopter"
    # Ground Domain
    GROUND_VEHICLE_WHEELED = "ground_vehicle_wheeled"
    GROUND_VEHICLE_TRACKED = "ground_vehicle_tracked"
    # Surface Domain
    SURFACE_VESSEL_USV = "surface_vessel_usv"
    # Systems
    SENSOR_PLATFORM = "sensor_platform"
    PAYLOAD_SYSTEM = "payload_system"
    OPERATOR_STATION = "operator_station"


class EntityStatus(Enum):
    """Entity lifecycle status"""

    ACTIVE = "active"
    INACTIVE = "inactive"
    UNKNOWN = "unknown"


@dataclass
class Position:
    """Standard position component following analysis"""

    latitude: float
    longitude: float
    altitude: float
    heading: Optional[float] = None
    timestamp: float = field(default_factory=time.time)


@dataclass
class Component:
    """Modular component following ECS pattern from analysis"""

    component_type: str
    data: Dict[str, Any]
    timestamp: float = field(default_factory=time.time)


@dataclass
class Entity:
    """
    Core Entity structure following industry patterns from API analysis.
    Implements standard ECS (Entity Component System) architecture.
    """

    entity_id: str
    entity_type: EntityType
    timestamp: float
    is_live: bool = True
    expiry_time: Optional[float] = None
    position: Optional[Position] = None
    components: Dict[str, Component] = field(default_factory=dict)
    aliases: Dict[str, str] = field(default_factory=dict)
    provenance: Dict[str, Any] = field(default_factory=dict)

    def is_expired(self) -> bool:
        """Check if entity is expired"""
        if self.expiry_time is None:
            return False
        return time.time() > self.expiry_time

    def add_component(self, component: Component):
        """Add component to entity"""
        self.components[component.component_type] = component
        self.timestamp = time.time()

    def get_component(self, component_type: str) -> Optional[Component]:
        """Get component by type"""
        return self.components.get(component_type)

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization"""
        return {
            "entity_id": self.entity_id,
            "entity_type": self.entity_type.value,
            "timestamp": self.timestamp,
            "is_live": self.is_live,
            "expiry_time": self.expiry_time,
            "position": (
                {
                    "latitude": self.position.latitude,
                    "longitude": self.position.longitude,
                    "altitude": self.position.altitude,
                    "heading": self.position.heading,
                    "timestamp": self.position.timestamp,
                }
                if self.position
                else None
            ),
            "components": {
                k: {"type": v.component_type, "data": v.data, "timestamp": v.timestamp}
                for k, v in self.components.items()
            },
            "aliases": self.aliases,
            "provenance": self.provenance,
        }


class EntityManager:
    """
    Core Entity Manager implementing industry standard patterns.
    Follows analysis recommendations for situational awareness.
    """

    def __init__(self):
        self._entities: Dict[str, Entity] = {}
        self._subscribers: Set[callable] = set()
        self._running = False

    async def start(self):
        """Start the entity manager"""
        self._running = True
        # Start background cleanup task
        asyncio.create_task(self._cleanup_expired_entities())
        print("EntityManager started")

    async def stop(self):
        """Stop the entity manager"""
        self._running = False
        print("EntityManager stopped")

    async def publish_entity(self, entity: Entity) -> bool:
        """
        Publish entity following standard patterns from analysis.
        Returns success status.
        """
        try:
            self._entities[entity.entity_id] = entity
            # Notify subscribers
            await self._notify_subscribers("entity_created", entity)
            print(f"Published entity: {entity.entity_id} ({entity.entity_type.value})")
            return True
        except Exception as e:
            print(f"Error publishing entity: {e}")
            return False

    async def get_entity(self, entity_id: str) -> Optional[Entity]:
        """Get entity by ID"""
        entity = self._entities.get(entity_id)
        if entity and entity.is_expired():
            await self._remove_entity(entity_id)
            return None
        return entity

    async def query_entities(
        self, entity_type: Optional[EntityType] = None, active_only: bool = True
    ) -> List[Entity]:
        """
        Query entities with filtering following analysis patterns.
        """
        results = []
        for entity in self._entities.values():
            # Skip expired entities
            if entity.is_expired():
                continue

            # Filter by type if specified
            if entity_type and entity.entity_type != entity_type:
                continue

            # Filter by active status
            if active_only and not entity.is_live:
                continue

            results.append(entity)

        return results

    async def update_entity(self, entity_id: str, updates: Dict[str, Any]) -> bool:
        """Update entity with new data"""
        entity = await self.get_entity(entity_id)
        if not entity:
            return False

        # Update fields
        for key, value in updates.items():
            if hasattr(entity, key):
                setattr(entity, key, value)

        entity.timestamp = time.time()
        await self._notify_subscribers("entity_updated", entity)
        return True

    async def remove_entity(self, entity_id: str) -> bool:
        """Remove entity"""
        return await self._remove_entity(entity_id)

    async def subscribe(self, callback: Callable):
        """Subscribe to entity events"""
        self._subscribers.add(callback)

    async def unsubscribe(self, callback: Callable):
        """Unsubscribe from entity events"""
        self._subscribers.discard(callback)

    async def _remove_entity(self, entity_id: str) -> bool:
        """Internal entity removal"""
        if entity_id in self._entities:
            entity = self._entities.pop(entity_id)
            await self._notify_subscribers("entity_removed", entity)
            print(f"Removed entity: {entity_id}")
            return True
        return False

    async def _notify_subscribers(self, event_type: str, entity: Entity):
        """Notify all subscribers of entity events"""
        for callback in self._subscribers:
            try:
                await callback(event_type, entity)
            except Exception as e:
                print(f"Error notifying subscriber: {e}")

    async def _cleanup_expired_entities(self):
        """Background task to clean up expired entities"""
        while self._running:
            expired_ids = []
            for entity_id, entity in self._entities.items():
                if entity.is_expired():
                    expired_ids.append(entity_id)

            for entity_id in expired_ids:
                await self._remove_entity(entity_id)

            await asyncio.sleep(10)  # Check every 10 seconds

    def get_stats(self) -> Dict[str, Any]:
        """Get entity manager statistics"""
        return {
            "total_entities": len(self._entities),
            "entity_types": {
                entity_type.value: len(
                    [e for e in self._entities.values() if e.entity_type == entity_type]
                )
                for entity_type in EntityType
            },
            "active_entities": len([e for e in self._entities.values() if e.is_live]),
            "subscribers": len(self._subscribers),
        }


# Helper functions for creating standard entities
def create_drone_entity(lat: float, lon: float, alt: float, name: Optional[str] = None) -> Entity:
    """Create a standard drone entity"""
    entity_id = str(uuid.uuid4())
    position = Position(latitude=lat, longitude=lon, altitude=alt)

    entity = Entity(
        entity_id=entity_id,
        entity_type=EntityType.AIRCRAFT_MULTIROTOR,
        timestamp=time.time(),
        position=position,
        provenance={"source": "constellation_overwatch", "created_by": "system"},
    )

    if name:
        entity.aliases["name"] = name

    return entity


def create_ground_station_entity(lat: float, lon: float, name: Optional[str] = None) -> Entity:
    """Create a ground station entity"""
    entity_id = str(uuid.uuid4())
    position = Position(latitude=lat, longitude=lon, altitude=0)

    entity = Entity(
        entity_id=entity_id,
        entity_type=EntityType.OPERATOR_STATION,
        timestamp=time.time(),
        position=position,
        provenance={"source": "constellation_overwatch", "created_by": "system"},
    )

    if name:
        entity.aliases["name"] = name

    return entity


# Example usage and testing
async def main():
    """Example usage of the entity manager"""

    # Create entity manager
    em = EntityManager()
    await em.start()

    # Create sample entities
    drone1 = create_drone_entity(40.7128, -74.0060, 100, "Drone-1")
    drone2 = create_drone_entity(40.7589, -73.9851, 150, "Drone-2")
    gcs = create_ground_station_entity(40.7505, -73.9934, "GCS-Main")

    # Publish entities
    await em.publish_entity(drone1)
    await em.publish_entity(drone2)
    await em.publish_entity(gcs)

    # Query entities
    all_entities = await em.query_entities()
    print(f"Total entities: {len(all_entities)}")

    aircraft = await em.query_entities(EntityType.AIRCRAFT_MULTIROTOR)
    print(f"Aircraft entities: {len(aircraft)}")

    # Get statistics
    stats = em.get_stats()
    print(f"Entity Manager Stats: {json.dumps(stats, indent=2)}")

    # Test entity retrieval
    retrieved = await em.get_entity(drone1.entity_id)
    if retrieved:
        print(f"Retrieved entity: {retrieved.aliases.get('name', 'unnamed')}")

    await em.stop()


if __name__ == "__main__":
    asyncio.run(main())
