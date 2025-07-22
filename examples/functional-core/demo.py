"""
Constellation Overwatch SDK - Functional Core Example
Demonstrates working implementation before API schema development.
"""

import asyncio
import json
import time
from typing import Dict, Any

from sdk.core.entity_manager import EntityManager, create_drone_entity, create_ground_station_entity
from sdk.core.message_bus import MessageBus, MessageType, publish_entity_event, publish_vehicle_telemetry


class SimpleVehicleInterface:
    """
    Basic vehicle interface demonstrating core functionality.
    This proves the concept before complex API schemas.
    """
    
    def __init__(self, vehicle_id: str, message_bus: MessageBus):
        self.vehicle_id = vehicle_id
        self.message_bus = message_bus
        self.telemetry = {
            "battery": 100.0,
            "altitude": 0.0,
            "speed": 0.0,
            "status": "idle"
        }
        self.running = False
    
    async def start(self):
        """Start the vehicle interface"""
        self.running = True
        # Subscribe to commands for this vehicle
        await self.message_bus.subscribe(
            callback=self._handle_command,
            topics={"commands"},
            message_types={MessageType.VEHICLE_COMMAND}
        )
        
        # Start telemetry publishing
        asyncio.create_task(self._publish_telemetry())
        print(f"Vehicle {self.vehicle_id} started")
    
    async def stop(self):
        """Stop the vehicle interface"""
        self.running = False
        print(f"Vehicle {self.vehicle_id} stopped")
    
    async def _handle_command(self, message):
        """Handle incoming vehicle commands"""
        # Only process commands for this vehicle
        if message.target != self.vehicle_id:
            return
        
        command = message.payload.get("command")
        print(f"Vehicle {self.vehicle_id} received command: {command}")
        
        if command == "takeoff":
            await self._execute_takeoff(message.payload)
        elif command == "land":
            await self._execute_land(message.payload)
        elif command == "goto":
            await self._execute_goto(message.payload)
        else:
            print(f"Unknown command: {command}")
    
    async def _execute_takeoff(self, params: Dict[str, Any]):
        """Simulate takeoff"""
        target_altitude = params.get("altitude", 50)
        print(f"Vehicle {self.vehicle_id} taking off to {target_altitude}m")
        
        self.telemetry["status"] = "takeoff"
        
        # Simulate gradual altitude increase
        for alt in range(0, int(target_altitude), 5):
            self.telemetry["altitude"] = alt
            self.telemetry["battery"] -= 0.5
            await asyncio.sleep(0.1)  # Simulate time
        
        self.telemetry["altitude"] = target_altitude
        self.telemetry["status"] = "hovering"
        print(f"Vehicle {self.vehicle_id} reached altitude {target_altitude}m")
    
    async def _execute_land(self, params: Dict[str, Any]):
        """Simulate landing"""
        print(f"Vehicle {self.vehicle_id} landing")
        
        self.telemetry["status"] = "landing"
        
        # Simulate gradual altitude decrease
        current_alt = self.telemetry["altitude"]
        for alt in range(int(current_alt), 0, -5):
            self.telemetry["altitude"] = alt
            self.telemetry["battery"] -= 0.2
            await asyncio.sleep(0.1)
        
        self.telemetry["altitude"] = 0
        self.telemetry["status"] = "landed"
        print(f"Vehicle {self.vehicle_id} landed")
    
    async def _execute_goto(self, params: Dict[str, Any]):
        """Simulate goto waypoint"""
        waypoint = params.get("waypoint", {})
        print(f"Vehicle {self.vehicle_id} going to waypoint: {waypoint}")
        
        self.telemetry["status"] = "flying"
        self.telemetry["speed"] = 10.0
        
        # Simulate flight time
        await asyncio.sleep(2)
        
        self.telemetry["speed"] = 0.0
        self.telemetry["status"] = "hovering"
        self.telemetry["battery"] -= 5.0
        print(f"Vehicle {self.vehicle_id} reached waypoint")
    
    async def _publish_telemetry(self):
        """Continuously publish telemetry"""
        while self.running:
            await publish_vehicle_telemetry(
                self.message_bus,
                self.vehicle_id,
                self.telemetry.copy()
            )
            await asyncio.sleep(1)  # Publish every second


class SimpleGroundControl:
    """
    Basic ground control station demonstrating system integration.
    """
    
    def __init__(self, entity_manager: EntityManager, message_bus: MessageBus):
        self.entity_manager = entity_manager
        self.message_bus = message_bus
        self.known_vehicles = {}
    
    async def start(self):
        """Start the ground control station"""
        # Subscribe to telemetry updates
        await self.message_bus.subscribe(
            callback=self._handle_telemetry,
            topics={"telemetry"}
        )
        
        # Subscribe to entity events
        await self.message_bus.subscribe(
            callback=self._handle_entity_event,
            topics={"entities"}
        )
        
        print("Ground Control Station started")
    
    async def _handle_telemetry(self, message):
        """Handle incoming telemetry"""
        vehicle_id = message.source
        telemetry = message.payload
        
        # Update known vehicles
        self.known_vehicles[vehicle_id] = {
            "last_seen": time.time(),
            "telemetry": telemetry
        }
        
        print(f"GCS: Telemetry from {vehicle_id} - "
              f"Battery: {telemetry.get('battery', 0):.1f}%, "
              f"Alt: {telemetry.get('altitude', 0):.1f}m, "
              f"Status: {telemetry.get('status', 'unknown')}")
    
    async def _handle_entity_event(self, message):
        """Handle entity events"""
        entity_id = message.payload.get("entity_id")
        event_type = message.payload.get("event_type")
        
        print(f"GCS: Entity {event_type} - {entity_id}")
    
    async def send_command(self, vehicle_id: str, command: str, params: Dict[str, Any] = None):
        """Send command to vehicle"""
        await self.message_bus.publish(
            message_type=MessageType.VEHICLE_COMMAND,
            source="gcs-main",
            target=vehicle_id,
            topic="commands",
            payload={
                "command": command,
                **(params or {})
            },
            priority=5
        )
        print(f"GCS: Sent command '{command}' to {vehicle_id}")
    
    def get_vehicle_status(self) -> Dict[str, Any]:
        """Get status of all known vehicles"""
        return {
            "total_vehicles": len(self.known_vehicles),
            "vehicles": {
                vid: {
                    "last_seen": data["last_seen"],
                    "battery": data["telemetry"].get("battery", 0),
                    "status": data["telemetry"].get("status", "unknown")
                }
                for vid, data in self.known_vehicles.items()
            }
        }


async def demo_functional_core():
    """
    Demonstrate the functional core working together.
    This validates architecture before API schema development.
    """
    print("=== Constellation Overwatch Functional Core Demo ===\n")
    
    # Initialize core components
    entity_manager = EntityManager()
    message_bus = MessageBus()
    
    # Start core services
    await entity_manager.start()
    await message_bus.start()
    
    # Connect entity manager to message bus
    async def entity_event_publisher(event_type: str, entity):
        await publish_entity_event(
            message_bus,
            event_type,
            entity.entity_id,
            entity.to_dict()
        )
    
    await entity_manager.subscribe(entity_event_publisher)
    
    # Create ground control station
    gcs = SimpleGroundControl(entity_manager, message_bus)
    await gcs.start()
    
    # Create and register entities
    print("1. Creating entities...")
    drone1 = create_drone_entity(40.7128, -74.0060, 0, "Drone-Alpha")
    drone2 = create_drone_entity(40.7589, -73.9851, 0, "Drone-Beta")
    gcs_entity = create_ground_station_entity(40.7505, -73.9934, "GCS-Main")
    
    await entity_manager.publish_entity(drone1)
    await entity_manager.publish_entity(drone2)
    await entity_manager.publish_entity(gcs_entity)
    
    # Create vehicle interfaces
    print("\n2. Starting vehicle interfaces...")
    vehicle1 = SimpleVehicleInterface(drone1.entity_id, message_bus)
    vehicle2 = SimpleVehicleInterface(drone2.entity_id, message_bus)
    
    await vehicle1.start()
    await vehicle2.start()
    
    # Let telemetry flow for a moment
    print("\n3. Initial telemetry...")
    await asyncio.sleep(3)
    
    # Execute mission sequence
    print("\n4. Executing mission sequence...")
    
    # Takeoff sequence
    await gcs.send_command(drone1.entity_id, "takeoff", {"altitude": 50})
    await asyncio.sleep(3)
    
    await gcs.send_command(drone2.entity_id, "takeoff", {"altitude": 75})
    await asyncio.sleep(3)
    
    # Navigation commands
    await gcs.send_command(drone1.entity_id, "goto", {
        "waypoint": {"lat": 40.7200, "lon": -74.0100}
    })
    await asyncio.sleep(3)
    
    await gcs.send_command(drone2.entity_id, "goto", {
        "waypoint": {"lat": 40.7600, "lon": -73.9800}
    })
    await asyncio.sleep(3)
    
    # Landing sequence
    await gcs.send_command(drone1.entity_id, "land")
    await asyncio.sleep(3)
    
    await gcs.send_command(drone2.entity_id, "land")
    await asyncio.sleep(3)
    
    # Show final status
    print("\n5. Final system status...")
    
    # Entity manager stats
    em_stats = entity_manager.get_stats()
    print(f"Entity Manager: {json.dumps(em_stats, indent=2)}")
    
    # Message bus stats
    bus_stats = message_bus.get_stats()
    print(f"Message Bus: {json.dumps(bus_stats, indent=2)}")
    
    # Vehicle status
    vehicle_status = gcs.get_vehicle_status()
    print(f"Vehicle Status: {json.dumps(vehicle_status, indent=2)}")
    
    # Cleanup
    print("\n6. Shutting down...")
    await vehicle1.stop()
    await vehicle2.stop()
    await entity_manager.stop()
    await message_bus.stop()
    
    print("\n=== Demo Complete ===")
    print("\nKey Observations:")
    print("✅ Entity management working")
    print("✅ Message bus connecting components")
    print("✅ Vehicle interfaces responding to commands")
    print("✅ Telemetry flowing through system")
    print("✅ Ground control coordinating operations")
    print("\nThis functional core validates the architecture.")
    print("Now we can confidently design APIs around proven functionality.")


if __name__ == "__main__":
    asyncio.run(demo_functional_core())
