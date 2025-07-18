#!/usr/bin/env python3
"""
Basic Drone Integration Example

This example demonstrates how to integrate a basic drone system
using the Constellation Overwatch SDK.
"""

import asyncio
import logging
from typing import Dict, Any, Optional

# Import from the SDK core module
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'sdk', 'core'))

from __init__ import (
    ConstellationOverwatchSDK,
    AutonomousSystem,
    SensorInterface,
    PayloadInterface,
    SystemStatus,
    Position,
    Velocity,
    Attitude,
    SensorData
)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class BasicDroneSystem(AutonomousSystem):
    """Basic drone system implementation"""
    
    def __init__(self, system_id: str):
        super().__init__(system_id, "multirotor_drone")
        self.altitude_setpoint = 0.0
        self.position_setpoint = None
        
    async def initialize(self) -> bool:
        """Initialize the drone system"""
        logger.info(f"Initializing drone system: {self.system_id}")
        self.status = SystemStatus.INITIALIZING
        
        # Simulate initialization process
        await asyncio.sleep(2)
        
        # Set initial position (example coordinates)
        self.position = Position(
            latitude=37.7749,
            longitude=-122.4194,
            altitude=0.0,
            timestamp=asyncio.get_event_loop().time()
        )
        
        self.velocity = Velocity(0.0, 0.0, 0.0, asyncio.get_event_loop().time())
        self.attitude = Attitude(0.0, 0.0, 0.0, asyncio.get_event_loop().time())
        
        self.status = SystemStatus.READY
        logger.info(f"Drone system {self.system_id} initialized successfully")
        return True
    
    async def start(self) -> bool:
        """Start the drone system"""
        if self.status == SystemStatus.READY:
            logger.info(f"Starting drone system: {self.system_id}")
            self.status = SystemStatus.ACTIVE
            return True
        return False
    
    async def stop(self) -> bool:
        """Stop the drone system"""
        logger.info(f"Stopping drone system: {self.system_id}")
        self.status = SystemStatus.READY
        return True
    
    async def get_telemetry(self) -> Dict[str, Any]:
        """Get current telemetry data"""
        return {
            "system_id": self.system_id,
            "status": self.status.value,
            "position": {
                "latitude": self.position.latitude,
                "longitude": self.position.longitude,
                "altitude": self.position.altitude
            },
            "velocity": {
                "x": self.velocity.x,
                "y": self.velocity.y,
                "z": self.velocity.z
            },
            "attitude": {
                "roll": self.attitude.roll,
                "pitch": self.attitude.pitch,
                "yaw": self.attitude.yaw
            }
        }
    
    async def send_command(self, command: str, parameters: Dict[str, Any]) -> bool:
        """Send command to the drone"""
        logger.info(f"Sending command '{command}' to {self.system_id}")
        
        if command == "takeoff":
            altitude = parameters.get("altitude", 10.0)
            await self.takeoff(altitude)
        elif command == "land":
            await self.land()
        elif command == "goto":
            lat = parameters.get("latitude")
            lon = parameters.get("longitude")
            alt = parameters.get("altitude", self.position.altitude)
            await self.goto_position(lat, lon, alt)
        elif command == "return_home":
            await self.return_home()
        else:
            logger.warning(f"Unknown command: {command}")
            return False
        
        return True
    
    async def takeoff(self, altitude: float):
        """Takeoff to specified altitude"""
        logger.info(f"Taking off to {altitude}m")
        self.altitude_setpoint = altitude
        
        # Simulate takeoff
        start_alt = self.position.altitude
        steps = 20
        for i in range(steps):
            progress = (i + 1) / steps
            self.position.altitude = start_alt + (altitude - start_alt) * progress
            await asyncio.sleep(0.1)
        
        logger.info(f"Takeoff complete - altitude: {self.position.altitude}m")
    
    async def land(self):
        """Land the drone"""
        logger.info("Landing")
        
        # Simulate landing
        start_alt = self.position.altitude
        steps = 20
        for i in range(steps):
            progress = (i + 1) / steps
            self.position.altitude = start_alt * (1 - progress)
            await asyncio.sleep(0.1)
        
        self.position.altitude = 0.0
        logger.info("Landing complete")
    
    async def goto_position(self, latitude: float, longitude: float, altitude: float):
        """Go to specified position"""
        logger.info(f"Going to position: {latitude}, {longitude}, {altitude}m")
        
        # Simulate movement
        start_pos = self.position
        steps = 30
        for i in range(steps):
            progress = (i + 1) / steps
            self.position.latitude = start_pos.latitude + (latitude - start_pos.latitude) * progress
            self.position.longitude = start_pos.longitude + (longitude - start_pos.longitude) * progress
            self.position.altitude = start_pos.altitude + (altitude - start_pos.altitude) * progress
            await asyncio.sleep(0.1)
        
        logger.info(f"Arrived at position: {self.position.latitude}, {self.position.longitude}, {self.position.altitude}m")
    
    async def return_home(self):
        """Return to home position"""
        logger.info("Returning home")
        # Return to origin
        await self.goto_position(37.7749, -122.4194, 0.0)


class CameraSensor(SensorInterface):
    """Basic camera sensor implementation"""
    
    def __init__(self, sensor_id: str):
        super().__init__(sensor_id, "rgb_camera")
        self.resolution = "1920x1080"
        self.fps = 30
    
    async def initialize(self) -> bool:
        """Initialize the camera sensor"""
        logger.info(f"Initializing camera sensor: {self.sensor_id}")
        await asyncio.sleep(1)
        logger.info(f"Camera sensor {self.sensor_id} initialized - {self.resolution} @ {self.fps}fps")
        return True
    
    async def start_streaming(self) -> bool:
        """Start camera streaming"""
        logger.info(f"Starting camera stream: {self.sensor_id}")
        self.is_active = True
        return True
    
    async def stop_streaming(self) -> bool:
        """Stop camera streaming"""
        logger.info(f"Stopping camera stream: {self.sensor_id}")
        self.is_active = False
        return True
    
    async def get_data(self) -> SensorData:
        """Get current camera data"""
        # Simulate camera data
        return SensorData(
            sensor_id=self.sensor_id,
            sensor_type=self.sensor_type,
            data=f"Camera frame #{asyncio.get_event_loop().time():.0f}",
            timestamp=asyncio.get_event_loop().time(),
            metadata={
                "resolution": self.resolution,
                "fps": self.fps,
                "format": "RGB24"
            }
        )


class GimbalPayload(PayloadInterface):
    """Basic gimbal payload implementation"""
    
    def __init__(self, payload_id: str):
        super().__init__(payload_id, "camera_gimbal")
        self.pan_angle = 0.0
        self.tilt_angle = 0.0
    
    async def initialize(self) -> bool:
        """Initialize the gimbal payload"""
        logger.info(f"Initializing gimbal payload: {self.payload_id}")
        await asyncio.sleep(1)
        logger.info(f"Gimbal payload {self.payload_id} initialized")
        return True
    
    async def activate(self) -> bool:
        """Activate the gimbal"""
        logger.info(f"Activating gimbal: {self.payload_id}")
        self.is_active = True
        return True
    
    async def deactivate(self) -> bool:
        """Deactivate the gimbal"""
        logger.info(f"Deactivating gimbal: {self.payload_id}")
        self.is_active = False
        return True
    
    async def execute_task(self, task: Dict[str, Any]) -> bool:
        """Execute gimbal task"""
        task_type = task.get("type")
        
        if task_type == "point_at":
            pan = task.get("pan", 0.0)
            tilt = task.get("tilt", 0.0)
            await self.point_at(pan, tilt)
        elif task_type == "scan":
            await self.scan_area()
        else:
            logger.warning(f"Unknown gimbal task: {task_type}")
            return False
        
        return True
    
    async def point_at(self, pan: float, tilt: float):
        """Point gimbal at specified angles"""
        logger.info(f"Pointing gimbal to pan:{pan}°, tilt:{tilt}°")
        self.pan_angle = pan
        self.tilt_angle = tilt
        await asyncio.sleep(0.5)  # Simulate movement time
    
    async def scan_area(self):
        """Scan area with gimbal"""
        logger.info("Starting area scan")
        scan_positions = [
            (-45, 0), (0, 0), (45, 0),
            (-45, -30), (0, -30), (45, -30)
        ]
        
        for pan, tilt in scan_positions:
            await self.point_at(pan, tilt)
            await asyncio.sleep(1)  # Dwell time
        
        logger.info("Area scan complete")


async def main():
    """Main example function"""
    logger.info("Starting Constellation Overwatch SDK Basic Drone Example")
    
    # Initialize SDK
    sdk = ConstellationOverwatchSDK()
    config = {
        "simulation": True,
        "log_level": "INFO"
    }
    
    if not await sdk.initialize(config):
        logger.error("Failed to initialize SDK")
        return
    
    # Create and register drone system
    drone = BasicDroneSystem("drone_001")
    if not await sdk.register_system(drone):
        logger.error("Failed to register drone system")
        return
    
    # Create and register camera sensor
    camera = CameraSensor("camera_001")
    if not await sdk.register_sensor(camera):
        logger.error("Failed to register camera sensor")
        return
    
    # Create and register gimbal payload
    gimbal = GimbalPayload("gimbal_001")
    if not await sdk.register_payload(gimbal):
        logger.error("Failed to register gimbal payload")
        return
    
    # Start systems
    await drone.start()
    await camera.start_streaming()
    await gimbal.activate()
    
    # Execute a basic mission
    logger.info("Starting basic mission")
    
    # Takeoff
    await drone.send_command("takeoff", {"altitude": 50.0})
    
    # Point gimbal down
    await gimbal.execute_task({"type": "point_at", "pan": 0.0, "tilt": -45.0})
    
    # Get some sensor data
    camera_data = await camera.get_data()
    logger.info(f"Camera data: {camera_data.data}")
    
    # Move to a new position
    await drone.send_command("goto", {
        "latitude": 37.7849,
        "longitude": -122.4094,
        "altitude": 30.0
    })
    
    # Scan the area
    await gimbal.execute_task({"type": "scan"})
    
    # Return home and land
    await drone.send_command("return_home", {})
    await drone.send_command("land", {})
    
    # Get final system status
    status = await sdk.get_system_status()
    logger.info(f"Final system status: {status}")
    
    logger.info("Example completed successfully!")


if __name__ == "__main__":
    asyncio.run(main())
