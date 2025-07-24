"""
Constellation Overwatch SDK - Core Entity Manager
Enhanced implementation with spatial querying, relationships, validation,
and persistence.
"""

from typing import Dict, List, Optional, Any, Set, Callable, Tuple
from dataclasses import dataclass, field
from enum import Enum
import asyncio
import uuid
import time
import json
import math
import logging
from pathlib import Path
import aiofiles

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


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
    # Subsurface Domain
    UNDERWATER_VEHICLE = "underwater_vehicle"
    # Systems
    SENSOR_PLATFORM = "sensor_platform"
    PAYLOAD_SYSTEM = "payload_system"
    OPERATOR_STATION = "operator_station"
    # Infrastructure
    WAYPOINT = "waypoint"
    NO_FLY_ZONE = "no_fly_zone"
    GEOFENCE = "geofence"


class EntityStatus(Enum):
    """Entity lifecycle status"""

    ACTIVE = "active"
    INACTIVE = "inactive"
    PENDING = "pending"
    ERROR = "error"
    MAINTENANCE = "maintenance"
    UNKNOWN = "unknown"


class EntityPriority(Enum):
    """Entity priority levels"""

    CRITICAL = "critical"
    HIGH = "high"
    NORMAL = "normal"
    LOW = "low"


class RelationshipType(Enum):
    """Entity relationship types"""

    PARENT_CHILD = "parent_child"
    ATTACHED_TO = "attached_to"
    FOLLOWS = "follows"
    ESCORTS = "escorts"
    COMMANDS = "commands"
    MONITORS = "monitors"


@dataclass
class Position:
    """Enhanced position component with velocity and accuracy"""

    latitude: float
    longitude: float
    altitude: float
    heading: Optional[float] = None
    velocity: Optional[float] = None
    accuracy: Optional[float] = None
    timestamp: float = field(default_factory=time.time)

    def distance_to(self, other: "Position") -> float:
        """Calculate distance to another position in meters"""
        R = 6371000  # Earth's radius in meters
        lat1, lon1 = math.radians(self.latitude), math.radians(self.longitude)
        lat2, lon2 = math.radians(other.latitude), math.radians(other.longitude)

        dlat = lat2 - lat1
        dlon = lon2 - lon1

        a = (
            math.sin(dlat / 2) ** 2
            + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c


@dataclass
class Component:
    """Enhanced modular component with schema validation"""

    component_type: str
    data: Dict[str, Any]
    timestamp: float = field(default_factory=time.time)
    schema: Optional[Dict[str, Any]] = None
    version: str = "1.0"

    def validate(self) -> bool:
        """Validate component data against schema"""
        if not self.schema:
            return True
        # Basic validation - can be extended with jsonschema
        required_fields = self.schema.get("required", [])
        return all(field in self.data for field in required_fields)


@dataclass
class EntityRelationship:
    """Relationship between entities"""

    source_id: str
    target_id: str
    relationship_type: RelationshipType
    metadata: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


@dataclass
class Entity:
    """Enhanced Entity structure with relationships and validation"""

    entity_id: str
    entity_type: EntityType
    timestamp: float
    status: EntityStatus = EntityStatus.ACTIVE
    priority: EntityPriority = EntityPriority.NORMAL
    is_live: bool = True
    expiry_time: Optional[float] = None
    position: Optional[Position] = None
    components: Dict[str, Component] = field(default_factory=dict)
    aliases: Dict[str, str] = field(default_factory=dict)
    provenance: Dict[str, Any] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(default_factory=dict)
    tags: Set[str] = field(default_factory=set)
    relationships: Dict[str, EntityRelationship] = field(default_factory=dict)

    def is_expired(self) -> bool:
        """Check if entity is expired"""
        if self.expiry_time is None:
            return False
        return time.time() > self.expiry_time

    def add_component(self, component: Component) -> bool:
        """Add component to entity with validation"""
        if component.validate():
            self.components[component.component_type] = component
            self.timestamp = time.time()
            return True
        return False

    def get_component(self, component_type: str) -> Optional[Component]:
        """Get component by type"""
        return self.components.get(component_type)

    def add_relationship(self, relationship: EntityRelationship):
        """Add relationship to another entity"""
        self.relationships[relationship.target_id] = relationship

    def get_relationships_by_type(
        self, rel_type: RelationshipType
    ) -> List[EntityRelationship]:
        """Get relationships by type"""
        return [
            rel
            for rel in self.relationships.values()
            if rel.relationship_type == rel_type
        ]

    def has_tag(self, tag: str) -> bool:
        """Check if entity has a specific tag"""
        return tag in self.tags

    def add_tag(self, tag: str):
        """Add tag to entity"""
        self.tags.add(tag)
        self.timestamp = time.time()

    def remove_tag(self, tag: str):
        """Remove tag from entity"""
        self.tags.discard(tag)
        self.timestamp = time.time()

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization"""
        return {
            "entity_id": self.entity_id,
            "entity_type": self.entity_type.value,
            "timestamp": self.timestamp,
            "status": self.status.value,
            "priority": self.priority.value,
            "is_live": self.is_live,
            "expiry_time": self.expiry_time,
            "position": (
                {
                    "latitude": self.position.latitude,
                    "longitude": self.position.longitude,
                    "altitude": self.position.altitude,
                    "heading": self.position.heading,
                    "velocity": self.position.velocity,
                    "accuracy": self.position.accuracy,
                    "timestamp": self.position.timestamp,
                }
                if self.position
                else None
            ),
            "components": {
                k: {
                    "type": v.component_type,
                    "data": v.data,
                    "timestamp": v.timestamp,
                    "version": v.version,
                }
                for k, v in self.components.items()
            },
            "aliases": self.aliases,
            "provenance": self.provenance,
            "metadata": self.metadata,
            "tags": list(self.tags),
            "relationships": {
                k: {
                    "source_id": v.source_id,
                    "target_id": v.target_id,
                    "type": v.relationship_type.value,
                    "metadata": v.metadata,
                    "timestamp": v.timestamp,
                }
                for k, v in self.relationships.items()
            },
        }


@dataclass
class SpatialQuery:
    """Spatial query parameters"""

    center_lat: float
    center_lon: float
    radius_meters: float
    entity_types: Optional[List[EntityType]] = None
    include_inactive: bool = False


@dataclass
class EntityManagerStats:
    """Entity manager performance statistics"""

    total_entities: int
    active_entities: int
    queries_per_second: float
    average_query_time: float
    memory_usage_mb: float
    uptime_seconds: float


class EntityValidator:
    """Entity validation service"""

    @staticmethod
    def validate_entity(entity: Entity) -> Tuple[bool, List[str]]:
        """Validate entity and return errors if any"""
        errors = []

        if not entity.entity_id:
            errors.append("Entity ID is required")

        if entity.position:
            if not (-90 <= entity.position.latitude <= 90):
                errors.append("Invalid latitude")
            if not (-180 <= entity.position.longitude <= 180):
                errors.append("Invalid longitude")

        # Validate components
        for comp in entity.components.values():
            if not comp.validate():
                errors.append(f"Component {comp.component_type} validation failed")

        return len(errors) == 0, errors


class EntityManager:
    """Enhanced Entity Manager with spatial queries, relationships, and persistence"""

    def __init__(self, persistence_path: Optional[str] = None):
        self._entities: Dict[str, Entity] = {}
        self._subscribers: Set[callable] = set()
        self._running = False
        self._start_time = time.time()
        self._query_count = 0
        self._total_query_time = 0.0
        self._persistence_path = persistence_path
        self._validator = EntityValidator()

        # Performance monitoring
        self._last_stats_update = time.time()
        self._queries_last_period = 0

    async def start(self):
        """Start the entity manager"""
        self._running = True
        self._start_time = time.time()

        # Load entities from persistence if configured
        if self._persistence_path:
            await self._load_entities()

        # Start background tasks
        asyncio.create_task(self._cleanup_expired_entities())
        asyncio.create_task(self._update_stats())
        asyncio.create_task(self._periodic_persistence())

        logger.info("EntityManager started")

    async def stop(self):
        """Stop the entity manager"""
        self._running = False

        # Save entities to persistence if configured
        if self._persistence_path:
            await self._save_entities()

        logger.info("EntityManager stopped")

    async def publish_entity(self, entity: Entity) -> Tuple[bool, List[str]]:
        """Publish entity with validation"""
        try:
            # Validate entity
            is_valid, errors = self._validator.validate_entity(entity)
            if not is_valid:
                logger.warning(f"Entity validation failed: {errors}")
                return False, errors

            self._entities[entity.entity_id] = entity
            await self._notify_subscribers("entity_created", entity)
            logger.info(
                f"Published entity: {entity.entity_id} ({entity.entity_type.value})"
            )
            return True, []

        except Exception as e:
            error_msg = f"Error publishing entity: {e}"
            logger.error(error_msg)
            return False, [error_msg]

    async def get_entity(self, entity_id: str) -> Optional[Entity]:
        """Get entity by ID with expiry check"""
        start_time = time.time()

        entity = self._entities.get(entity_id)
        if entity and entity.is_expired():
            await self._remove_entity(entity_id)
            entity = None

        self._update_query_stats(time.time() - start_time)
        return entity

    async def query_entities(
        self,
        entity_type: Optional[EntityType] = None,
        status: Optional[EntityStatus] = None,
        active_only: bool = True,
        tags: Optional[Set[str]] = None,
        limit: Optional[int] = None,
    ) -> List[Entity]:
        """Enhanced entity querying with multiple filters"""
        start_time = time.time()
        results = []

        for entity in self._entities.values():
            # Skip expired entities
            if entity.is_expired():
                continue

            # Filter by type if specified
            if entity_type and entity.entity_type != entity_type:
                continue

            # Filter by status
            if status and entity.status != status:
                continue

            # Filter by active status
            if active_only and not entity.is_live:
                continue

            # Filter by tags
            if tags and not tags.issubset(entity.tags):
                continue

            results.append(entity)

            # Apply limit
            if limit and len(results) >= limit:
                break

        self._update_query_stats(time.time() - start_time)
        return results

    async def spatial_query(self, query: SpatialQuery) -> List[Entity]:
        """Perform spatial query for entities within radius"""
        start_time = time.time()
        results = []

        center_pos = Position(query.center_lat, query.center_lon, 0)

        for entity in self._entities.values():
            # Skip entities without position
            if not entity.position:
                continue

            # Skip expired entities
            if entity.is_expired():
                continue

            # Skip inactive entities unless requested
            if not query.include_inactive and not entity.is_live:
                continue

            # Filter by entity types
            if query.entity_types and entity.entity_type not in query.entity_types:
                continue

            # Check distance
            distance = center_pos.distance_to(entity.position)
            if distance <= query.radius_meters:
                results.append(entity)

        self._update_query_stats(time.time() - start_time)
        return results

    async def get_entities_by_relationship(
        self, entity_id: str, relationship_type: RelationshipType
    ) -> List[Entity]:
        """Get entities related to a specific entity"""
        start_time = time.time()
        results = []

        source_entity = await self.get_entity(entity_id)
        if not source_entity:
            return results

        # Get entities this entity has relationships with
        for rel in source_entity.get_relationships_by_type(relationship_type):
            target_entity = await self.get_entity(rel.target_id)
            if target_entity:
                results.append(target_entity)

        # Get entities that have relationships with this entity
        for entity in self._entities.values():
            for rel in entity.get_relationships_by_type(relationship_type):
                if rel.target_id == entity_id:
                    results.append(entity)

        self._update_query_stats(time.time() - start_time)
        return results

    async def create_relationship(
        self,
        source_id: str,
        target_id: str,
        rel_type: RelationshipType,
        metadata: Dict[str, Any] = None,
    ) -> bool:
        """Create relationship between entities"""
        source_entity = await self.get_entity(source_id)
        target_entity = await self.get_entity(target_id)

        if not source_entity or not target_entity:
            return False

        relationship = EntityRelationship(
            source_id=source_id,
            target_id=target_id,
            relationship_type=rel_type,
            metadata=metadata or {},
        )

        source_entity.add_relationship(relationship)
        await self._notify_subscribers("relationship_created", source_entity)
        return True

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
        """Remove entity and cleanup relationships"""
        return await self._remove_entity(entity_id)

    async def subscribe(self, callback: Callable):
        """Subscribe to entity events"""
        self._subscribers.add(callback)

    async def unsubscribe(self, callback: Callable):
        """Unsubscribe from entity events"""
        self._subscribers.discard(callback)

    async def _remove_entity(self, entity_id: str) -> bool:
        """Internal entity removal with relationship cleanup"""
        if entity_id in self._entities:
            entity = self._entities.pop(entity_id)

            # Clean up relationships pointing to this entity
            for other_entity in self._entities.values():
                if entity_id in other_entity.relationships:
                    del other_entity.relationships[entity_id]

            await self._notify_subscribers("entity_removed", entity)
            logger.info(f"Removed entity: {entity_id}")
            return True
        return False

    async def _notify_subscribers(self, event_type: str, entity: Entity):
        """Notify all subscribers of entity events"""
        for callback in self._subscribers:
            try:
                if asyncio.iscoroutinefunction(callback):
                    await callback(event_type, entity)
                else:
                    callback(event_type, entity)
            except Exception as e:
                logger.error(f"Error notifying subscriber: {e}")

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

    async def _update_stats(self):
        """Update performance statistics"""
        while self._running:
            await asyncio.sleep(5)  # Update every 5 seconds
            self._last_stats_update = time.time()

    async def _periodic_persistence(self):
        """Periodically save entities to persistence"""
        if not self._persistence_path:
            return

        while self._running:
            await asyncio.sleep(60)  # Save every minute
            await self._save_entities()

    async def _save_entities(self):
        """Save entities to persistence"""
        if not self._persistence_path:
            return

        try:
            data = {
                "entities": {
                    eid: entity.to_dict() for eid, entity in self._entities.items()
                },
                "timestamp": time.time(),
            }

            async with aiofiles.open(self._persistence_path, "w") as f:
                await f.write(json.dumps(data, indent=2))

            logger.info(f"Saved {len(self._entities)} entities to persistence")
        except Exception as e:
            logger.error(f"Error saving entities: {e}")

    async def _load_entities(self):
        """Load entities from persistence"""
        if not self._persistence_path or not Path(self._persistence_path).exists():
            return

        try:
            async with aiofiles.open(self._persistence_path, "r") as f:
                content = await f.read()
                data = json.loads(content)

            # Reconstruct entities from saved data
            for entity_data in data.get("entities", {}).values():
                # This is a simplified reconstruction - in practice you'd want
                # proper deserialization with validation
                pass

            logger.info("Loaded entities from persistence")
        except Exception as e:
            logger.error(f"Error loading entities: {e}")

    def _update_query_stats(self, query_time: float):
        """Update query performance statistics"""
        self._query_count += 1
        self._total_query_time += query_time
        self._queries_last_period += 1

    def get_stats(self) -> EntityManagerStats:
        """Get comprehensive entity manager statistics"""
        current_time = time.time()
        uptime = current_time - self._start_time

        # Calculate queries per second
        time_since_last_update = current_time - self._last_stats_update
        qps = self._queries_last_period / max(time_since_last_update, 1)
        self._queries_last_period = 0

        # Calculate average query time
        avg_query_time = (
            self._total_query_time / max(self._query_count, 1)
            if self._query_count > 0
            else 0
        )

        return EntityManagerStats(
            total_entities=len(self._entities),
            active_entities=len([e for e in self._entities.values() if e.is_live]),
            queries_per_second=qps,
            average_query_time=avg_query_time,
            memory_usage_mb=0,  # Would need psutil for actual memory usage
            uptime_seconds=uptime,
        )


# Helper functions for creating standard entities
def create_drone_entity(
    lat: float,
    lon: float,
    alt: float,
    name: Optional[str] = None,
    heading: Optional[float] = None,
) -> Entity:
    """Create an enhanced drone entity"""
    entity_id = str(uuid.uuid4())
    position = Position(latitude=lat, longitude=lon, altitude=alt, heading=heading)

    entity = Entity(
        entity_id=entity_id,
        entity_type=EntityType.AIRCRAFT_MULTIROTOR,
        timestamp=time.time(),
        status=EntityStatus.ACTIVE,
        priority=EntityPriority.NORMAL,
        position=position,
        provenance={"source": "constellation_overwatch", "created_by": "system"},
        tags={"drone", "aerial_vehicle"},
    )

    if name:
        entity.aliases["name"] = name
        entity.aliases["callsign"] = name

    return entity


def create_ground_station_entity(
    lat: float, lon: float, name: Optional[str] = None
) -> Entity:
    """Create an enhanced ground station entity"""
    entity_id = str(uuid.uuid4())
    position = Position(latitude=lat, longitude=lon, altitude=0)

    entity = Entity(
        entity_id=entity_id,
        entity_type=EntityType.OPERATOR_STATION,
        timestamp=time.time(),
        status=EntityStatus.ACTIVE,
        priority=EntityPriority.HIGH,
        position=position,
        provenance={"source": "constellation_overwatch", "created_by": "system"},
        tags={"ground_station", "control"},
    )

    if name:
        entity.aliases["name"] = name
        entity.aliases["station_id"] = name

    return entity


def create_waypoint_entity(
    lat: float, lon: float, alt: float, name: Optional[str] = None
) -> Entity:
    """Create a waypoint entity"""
    entity_id = str(uuid.uuid4())
    position = Position(latitude=lat, longitude=lon, altitude=alt)

    entity = Entity(
        entity_id=entity_id,
        entity_type=EntityType.WAYPOINT,
        timestamp=time.time(),
        status=EntityStatus.ACTIVE,
        priority=EntityPriority.LOW,
        position=position,
        provenance={"source": "constellation_overwatch", "created_by": "system"},
        tags={"waypoint", "navigation"},
    )

    if name:
        entity.aliases["name"] = name

    return entity


# Enhanced example usage
async def main():
    """Enhanced example usage with new features"""

    # Create entity manager with persistence
    persistence_path = "entities.json"
    em = EntityManager(persistence_path=persistence_path)
    await em.start()

    # Create sample entities with enhanced features
    drone1 = create_drone_entity(40.7128, -74.0060, 100, "Drone-Alpha", heading=45.0)
    drone2 = create_drone_entity(40.7589, -73.9851, 150, "Drone-Beta", heading=180.0)
    gcs = create_ground_station_entity(40.7505, -73.9934, "GCS-Main")
    waypoint1 = create_waypoint_entity(40.7400, -74.0000, 50, "WP-001")

    # Add custom components
    telemetry_component = Component(
        component_type="telemetry",
        data={"battery": 85, "signal_strength": -65, "temperature": 22.5},
        schema={"required": ["battery", "signal_strength"]},
    )
    drone1.add_component(telemetry_component)

    # Publish entities
    success, errors = await em.publish_entity(drone1)
    if not success:
        logger.error(f"Failed to publish drone1: {errors}")

    await em.publish_entity(drone2)
    await em.publish_entity(gcs)
    await em.publish_entity(waypoint1)

    # Create relationships
    await em.create_relationship(
        gcs.entity_id,
        drone1.entity_id,
        RelationshipType.COMMANDS,
        {"control_link": "primary"},
    )
    await em.create_relationship(
        drone1.entity_id,
        waypoint1.entity_id,
        RelationshipType.FOLLOWS,
        {"mission_id": "patrol_001"},
    )

    # Perform spatial query
    spatial_query = SpatialQuery(
        center_lat=40.7400,
        center_lon=-74.0000,
        radius_meters=5000,
        entity_types=[EntityType.AIRCRAFT_MULTIROTOR],
    )
    nearby_drones = await em.spatial_query(spatial_query)
    logger.info(f"Found {len(nearby_drones)} drones within 5km")

    # Query by tags
    tagged_entities = await em.query_entities(tags={"drone"})
    logger.info(f"Found {len(tagged_entities)} entities with 'drone' tag")

    # Get related entities
    controlled_entities = await em.get_entities_by_relationship(
        gcs.entity_id, RelationshipType.COMMANDS
    )
    logger.info(f"GCS controls {len(controlled_entities)} entities")

    # Get comprehensive statistics
    stats = em.get_stats()
    logger.info(f"Entity Manager Stats: {stats}")

    await em.stop()


if __name__ == "__main__":
    asyncio.run(main())
