#!/usr/bin/env python3
"""
VR Control Interface Example

This example demonstrates virtual reality control of drone operations,
providing immersive 3D visualization and intuitive gesture-based control
for single and swarm drone operations.
"""

import asyncio
import logging
import json
import numpy as np
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import time
import math

# Import from the SDK core and interface modules
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'sdk'))

from core import (
    ConstellationOverwatchSDK,
    AutonomousSystem,
    Position,
    Velocity,
    Attitude,
    SystemStatus
)

from interfaces import (
    VRInterface,
    UserInputEvent,
    VRControllerState,
    VRGesture,
    VREnvironment
)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class VRControlMode(Enum):
    """VR control modes"""
    OVERVIEW = "overview"          # High-level mission overview
    DIRECT_CONTROL = "direct"      # Direct drone piloting
    FORMATION = "formation"        # Formation control
    MISSION_PLANNING = "planning"  # Mission planning mode
    TELEMETRY = "telemetry"       # Detailed telemetry view
    SWARM_CONTROL = "swarm"       # Swarm orchestration


@dataclass
class VRControllerInput:
    """VR controller input data"""
    controller_id: str  # "left" or "right"
    position: Tuple[float, float, float]  # X, Y, Z position
    rotation: Tuple[float, float, float, float]  # Quaternion (w, x, y, z)
    buttons: Dict[str, bool]  # Button states
    trigger: float  # Trigger pressure (0.0 - 1.0)
    grip: float  # Grip pressure (0.0 - 1.0)
    touchpad: Tuple[float, float]  # Touchpad position (-1.0 to 1.0)
    gestures: List[VRGesture]  # Detected gestures
    timestamp: float


@dataclass
class VRDroneRepresentation:
    """Virtual representation of a drone in VR space"""
    drone_id: str
    position: Position
    virtual_position: Tuple[float, float, float]  # Position in VR space
    model_scale: float
    color: Tuple[float, float, float, float]  # RGBA
    selected: bool
    trail_points: List[Tuple[float, float, float]]  # Position trail
    status_indicators: Dict[str, Any]
    last_update: float


@dataclass
class VRGestureCommand:
    """VR gesture mapped to drone command"""
    gesture_type: str
    parameters: Dict[str, Any]
    target_drones: List[str]
    confidence: float
    execution_time: float


class VRGestureRecognizer:
    """Recognizes VR gestures for drone control"""
    
    def __init__(self):
        self.gesture_templates = {}
        self.gesture_history = []
        self.recognition_threshold = 0.8
        
    async def initialize(self) -> bool:
        """Initialize gesture recognition"""
        logger.info("Initializing VR gesture recognition")
        
        # Define gesture templates
        self.gesture_templates = {
            "point_and_send": {
                "description": "Point at location and press trigger to send drone",
                "pattern": "point_trigger",
                "required_controllers": ["right"],
                "parameters": ["target_position"]
            },
            "formation_spread": {
                "description": "Spread hands apart to expand formation",
                "pattern": "hands_spread",
                "required_controllers": ["left", "right"],
                "parameters": ["spread_distance"]
            },
            "formation_contract": {
                "description": "Bring hands together to contract formation",
                "pattern": "hands_contract",
                "required_controllers": ["left", "right"],
                "parameters": ["contract_distance"]
            },
            "swoop_gesture": {
                "description": "Swooping motion to command takeoff",
                "pattern": "upward_swoop",
                "required_controllers": ["right"],
                "parameters": ["altitude"]
            },
            "circle_patrol": {
                "description": "Circle motion to command patrol",
                "pattern": "circular_motion",
                "required_controllers": ["right"],
                "parameters": ["patrol_radius", "patrol_center"]
            },
            "emergency_stop": {
                "description": "Both hands up to emergency stop",
                "pattern": "hands_up",
                "required_controllers": ["left", "right"],
                "parameters": []
            },
            "land_gesture": {
                "description": "Downward motion to command landing",
                "pattern": "downward_motion",
                "required_controllers": ["right"],
                "parameters": ["landing_position"]
            },
            "selection_box": {
                "description": "Draw box to select multiple drones",
                "pattern": "box_selection",
                "required_controllers": ["right"],
                "parameters": ["selection_bounds"]
            }
        }
        
        logger.info(f"Loaded {len(self.gesture_templates)} gesture templates")
        return True
    
    async def recognize_gesture(self, left_controller: VRControllerInput, 
                              right_controller: VRControllerInput) -> Optional[VRGestureCommand]:
        """Recognize gesture from controller inputs"""
        
        # Update gesture history
        self.gesture_history.append({
            "left": left_controller,
            "right": right_controller,
            "timestamp": time.time()
        })
        
        # Keep only last 2 seconds of history
        cutoff_time = time.time() - 2.0
        self.gesture_history = [h for h in self.gesture_history if h["timestamp"] > cutoff_time]
        
        # Check each gesture template
        for gesture_name, template in self.gesture_templates.items():
            confidence = await self.match_gesture_pattern(template, self.gesture_history)
            
            if confidence > self.recognition_threshold:
                # Extract parameters based on gesture
                parameters = await self.extract_gesture_parameters(gesture_name, self.gesture_history)
                
                return VRGestureCommand(
                    gesture_type=gesture_name,
                    parameters=parameters,
                    target_drones=[],  # Will be filled by controller
                    confidence=confidence,
                    execution_time=time.time()
                )
        
        return None
    
    async def match_gesture_pattern(self, template: Dict[str, Any], history: List[Dict]) -> float:
        """Match gesture pattern against history"""
        if len(history) < 5:  # Need minimum history
            return 0.0
        
        pattern = template["pattern"]
        
        if pattern == "point_trigger":
            # Check if right controller is pointing and trigger is pressed
            recent = history[-1]
            right_ctrl = recent["right"]
            
            # Check if trigger is pressed
            if right_ctrl.trigger > 0.8:
                # Check if controller is in pointing position
                # (This is a simplified check - in reality, you'd analyze the rotation)
                return 0.9
        
        elif pattern == "hands_spread":
            # Check if hands are moving apart
            if len(history) >= 10:
                early = history[-10]
                recent = history[-1]
                
                early_distance = self.calculate_hand_distance(early["left"], early["right"])
                recent_distance = self.calculate_hand_distance(recent["left"], recent["right"])
                
                if recent_distance > early_distance * 1.5:  # Hands spread 50% more
                    return 0.85
        
        elif pattern == "hands_contract":
            # Check if hands are moving together
            if len(history) >= 10:
                early = history[-10]
                recent = history[-1]
                
                early_distance = self.calculate_hand_distance(early["left"], early["right"])
                recent_distance = self.calculate_hand_distance(recent["left"], recent["right"])
                
                if recent_distance < early_distance * 0.7:  # Hands contract 30%
                    return 0.85
        
        elif pattern == "upward_swoop":
            # Check for upward swooping motion
            if len(history) >= 8:
                positions = [h["right"].position[1] for h in history[-8:]]  # Y positions
                
                # Check if there's a clear upward trend
                if positions[-1] > positions[0] + 0.3:  # Moved up 30cm
                    return 0.8
        
        elif pattern == "circular_motion":
            # Check for circular hand motion
            if len(history) >= 15:
                positions = [(h["right"].position[0], h["right"].position[2]) for h in history[-15:]]
                
                # Simplified circle detection
                if self.is_circular_motion(positions):
                    return 0.75
        
        elif pattern == "hands_up":
            # Check if both hands are raised
            recent = history[-1]
            left_y = recent["left"].position[1]
            right_y = recent["right"].position[1]
            
            if left_y > 1.5 and right_y > 1.5:  # Both hands above head level
                return 0.9
        
        elif pattern == "downward_motion":
            # Check for downward motion
            if len(history) >= 8:
                positions = [h["right"].position[1] for h in history[-8:]]
                
                if positions[-1] < positions[0] - 0.3:  # Moved down 30cm
                    return 0.8
        
        return 0.0
    
    def calculate_hand_distance(self, left_ctrl: VRControllerInput, right_ctrl: VRControllerInput) -> float:
        """Calculate distance between hands"""
        left_pos = np.array(left_ctrl.position)
        right_pos = np.array(right_ctrl.position)
        return np.linalg.norm(right_pos - left_pos)
    
    def is_circular_motion(self, positions: List[Tuple[float, float]]) -> bool:
        """Check if positions form a circular pattern"""
        if len(positions) < 10:
            return False
        
        # Calculate center of positions
        center_x = sum(pos[0] for pos in positions) / len(positions)
        center_z = sum(pos[1] for pos in positions) / len(positions)
        
        # Check if positions are roughly equidistant from center
        distances = [math.sqrt((pos[0] - center_x)**2 + (pos[1] - center_z)**2) for pos in positions]
        avg_distance = sum(distances) / len(distances)
        
        # Check if all distances are within 20% of average
        return all(abs(d - avg_distance) / avg_distance < 0.2 for d in distances)
    
    async def extract_gesture_parameters(self, gesture_name: str, history: List[Dict]) -> Dict[str, Any]:
        """Extract parameters from recognized gesture"""
        parameters = {}
        
        if gesture_name == "point_and_send":
            # Extract pointing direction and calculate target position
            recent = history[-1]
            right_ctrl = recent["right"]
            
            # In a real implementation, you'd use the VR space mapping
            # to convert controller position/rotation to world coordinates
            parameters["target_position"] = {
                "x": right_ctrl.position[0] * 10,  # Scale to world coordinates
                "y": right_ctrl.position[1] * 10,
                "z": right_ctrl.position[2] * 10
            }
        
        elif gesture_name in ["formation_spread", "formation_contract"]:
            # Calculate spread/contract distance
            recent = history[-1]
            distance = self.calculate_hand_distance(recent["left"], recent["right"])
            parameters["distance"] = distance
        
        elif gesture_name == "swoop_gesture":
            # Extract altitude from swoop height
            positions = [h["right"].position[1] for h in history[-8:]]
            altitude = max(positions) - min(positions)
            parameters["altitude"] = max(20.0, altitude * 50)  # Scale to meters
        
        elif gesture_name == "circle_patrol":
            # Extract patrol radius and center
            positions = [(h["right"].position[0], h["right"].position[2]) for h in history[-15:]]
            center_x = sum(pos[0] for pos in positions) / len(positions)
            center_z = sum(pos[1] for pos in positions) / len(positions)
            
            # Calculate average radius
            distances = [math.sqrt((pos[0] - center_x)**2 + (pos[1] - center_z)**2) for pos in positions]
            avg_radius = sum(distances) / len(distances)
            
            parameters["patrol_center"] = {"x": center_x * 10, "z": center_z * 10}
            parameters["patrol_radius"] = max(10.0, avg_radius * 20)  # Scale to meters
        
        elif gesture_name == "land_gesture":
            # Extract landing position
            recent = history[-1]
            right_ctrl = recent["right"]
            
            parameters["landing_position"] = {
                "x": right_ctrl.position[0] * 10,
                "z": right_ctrl.position[2] * 10
            }
        
        return parameters


class VRDroneController:
    """VR-based drone controller with immersive interface"""
    
    def __init__(self, controller_id: str):
        self.controller_id = controller_id
        self.vr_interface = None
        self.gesture_recognizer = VRGestureRecognizer()
        self.drone_systems = {}
        self.vr_drones = {}
        self.current_mode = VRControlMode.OVERVIEW
        self.selected_drones = []
        self.vr_environment = None
        self.scale_factor = 0.1  # VR space scale (1 meter real = 0.1 meters VR)
        
    async def initialize(self, vr_config: Dict[str, Any]) -> bool:
        """Initialize VR controller"""
        logger.info(f"Initializing VR drone controller: {self.controller_id}")
        
        # Initialize VR interface
        self.vr_interface = VRInterface("openvr")  # or "oculus", "steamvr"
        
        if not await self.vr_interface.initialize(vr_config):
            logger.error("Failed to initialize VR interface")
            return False
        
        # Initialize gesture recognition
        await self.gesture_recognizer.initialize()
        
        # Set up VR environment
        self.vr_environment = VREnvironment(
            name="DroneCommandCenter",
            scale=self.scale_factor,
            ground_plane=True,
            sky_dome=True,
            lighting="daylight"
        )
        
        await self.vr_interface.load_environment(self.vr_environment)
        
        logger.info("VR drone controller initialized successfully")
        return True
    
    async def register_drone_system(self, drone_system: AutonomousSystem):
        """Register a drone system for VR control"""
        self.drone_systems[drone_system.system_id] = drone_system
        
        # Create VR representation
        vr_drone = VRDroneRepresentation(
            drone_id=drone_system.system_id,
            position=drone_system.position,
            virtual_position=(0.0, 0.0, 0.0),
            model_scale=1.0,
            color=(0.0, 1.0, 0.0, 1.0),  # Green
            selected=False,
            trail_points=[],
            status_indicators={},
            last_update=time.time()
        )
        
        self.vr_drones[drone_system.system_id] = vr_drone
        
        # Load drone model in VR
        await self.vr_interface.load_drone_model(drone_system.system_id, {
            "model_path": "models/drone_quadcopter.obj",
            "scale": vr_drone.model_scale,
            "color": vr_drone.color
        })
        
        logger.info(f"Registered drone {drone_system.system_id} in VR")
    
    async def update_vr_loop(self):
        """Main VR update loop"""
        logger.info("Starting VR update loop")
        
        while True:
            try:
                # Get VR input
                vr_state = await self.vr_interface.get_vr_state()
                
                if not vr_state.hmd_connected:
                    logger.warning("VR headset disconnected")
                    await asyncio.sleep(1.0)
                    continue
                
                # Update drone positions in VR
                await self.update_drone_representations()
                
                # Process controller input
                await self.process_vr_input(vr_state)
                
                # Update VR display
                await self.update_vr_display()
                
                # Small delay to maintain frame rate
                await asyncio.sleep(1.0 / 90.0)  # 90 FPS target
                
            except Exception as e:
                logger.error(f"Error in VR update loop: {e}")
                await asyncio.sleep(0.1)
    
    async def update_drone_representations(self):
        """Update drone representations in VR space"""
        for drone_id, drone_system in self.drone_systems.items():
            if drone_id not in self.vr_drones:
                continue
            
            vr_drone = self.vr_drones[drone_id]
            
            # Get latest telemetry
            telemetry = await drone_system.get_telemetry()
            
            # Update position
            real_pos = telemetry.get("position", drone_system.position)
            vr_pos = self.world_to_vr_coordinates(real_pos)
            vr_drone.virtual_position = vr_pos
            
            # Update trail
            vr_drone.trail_points.append(vr_pos)
            if len(vr_drone.trail_points) > 100:  # Keep last 100 points
                vr_drone.trail_points = vr_drone.trail_points[-100:]
            
            # Update status indicators
            vr_drone.status_indicators = {
                "battery": telemetry.get("battery_level", 0),
                "status": telemetry.get("status", "unknown"),
                "altitude": real_pos.altitude if hasattr(real_pos, 'altitude') else 0.0,
                "speed": telemetry.get("speed", 0.0)
            }
            
            # Update color based on status
            status = vr_drone.status_indicators["status"]
            if status == "active":
                vr_drone.color = (0.0, 1.0, 0.0, 1.0)  # Green
            elif status == "warning":
                vr_drone.color = (1.0, 1.0, 0.0, 1.0)  # Yellow
            elif status == "error":
                vr_drone.color = (1.0, 0.0, 0.0, 1.0)  # Red
            else:
                vr_drone.color = (0.5, 0.5, 0.5, 1.0)  # Gray
            
            # Update VR object
            await self.vr_interface.update_drone_position(drone_id, vr_pos)
            await self.vr_interface.update_drone_color(drone_id, vr_drone.color)
            
            vr_drone.last_update = time.time()
    
    async def process_vr_input(self, vr_state):
        """Process VR controller input"""
        left_controller = vr_state.controllers.get("left")
        right_controller = vr_state.controllers.get("right")
        
        if not left_controller or not right_controller:
            return
        
        # Check for gesture recognition
        gesture_command = await self.gesture_recognizer.recognize_gesture(
            left_controller, right_controller
        )
        
        if gesture_command:
            await self.execute_gesture_command(gesture_command)
        
        # Check for button presses
        await self.process_button_input(left_controller, right_controller)
        
        # Check for direct manipulation
        await self.process_direct_manipulation(left_controller, right_controller)
    
    async def execute_gesture_command(self, gesture_command: VRGestureCommand):
        """Execute recognized gesture command"""
        logger.info(f"Executing gesture command: {gesture_command.gesture_type} "
                   f"(confidence: {gesture_command.confidence:.2f})")
        
        # Determine target drones
        target_drones = self.selected_drones if self.selected_drones else list(self.drone_systems.keys())
        
        if gesture_command.gesture_type == "point_and_send":
            # Send selected drones to pointed location
            target_pos = gesture_command.parameters["target_position"]
            world_pos = self.vr_to_world_coordinates(target_pos)
            
            for drone_id in target_drones:
                if drone_id in self.drone_systems:
                    await self.drone_systems[drone_id].send_command("goto", {
                        "position": world_pos
                    })
        
        elif gesture_command.gesture_type == "formation_spread":
            # Spread formation
            spread_distance = gesture_command.parameters["distance"] * 10  # Scale to meters
            await self.execute_formation_spread(target_drones, spread_distance)
        
        elif gesture_command.gesture_type == "formation_contract":
            # Contract formation
            contract_distance = gesture_command.parameters["distance"] * 5  # Scale to meters
            await self.execute_formation_contract(target_drones, contract_distance)
        
        elif gesture_command.gesture_type == "swoop_gesture":
            # Takeoff command
            altitude = gesture_command.parameters["altitude"]
            for drone_id in target_drones:
                if drone_id in self.drone_systems:
                    await self.drone_systems[drone_id].send_command("takeoff", {
                        "altitude": altitude
                    })
        
        elif gesture_command.gesture_type == "circle_patrol":
            # Patrol command
            patrol_center = gesture_command.parameters["patrol_center"]
            patrol_radius = gesture_command.parameters["patrol_radius"]
            
            for drone_id in target_drones:
                if drone_id in self.drone_systems:
                    await self.drone_systems[drone_id].send_command("patrol", {
                        "center": patrol_center,
                        "radius": patrol_radius,
                        "pattern": "circle"
                    })
        
        elif gesture_command.gesture_type == "emergency_stop":
            # Emergency stop all drones
            for drone_id in self.drone_systems:
                await self.drone_systems[drone_id].send_command("emergency_stop", {})
        
        elif gesture_command.gesture_type == "land_gesture":
            # Land command
            landing_pos = gesture_command.parameters["landing_position"]
            world_pos = self.vr_to_world_coordinates(landing_pos)
            
            for drone_id in target_drones:
                if drone_id in self.drone_systems:
                    await self.drone_systems[drone_id].send_command("land", {
                        "position": world_pos
                    })
    
    async def process_button_input(self, left_controller: VRControllerInput, 
                                 right_controller: VRControllerInput):
        """Process VR controller button inputs"""
        
        # Menu button - toggle control mode
        if right_controller.buttons.get("menu", False):
            await self.cycle_control_mode()
        
        # Grip button - selection mode
        if right_controller.buttons.get("grip", False):
            await self.handle_selection_mode(right_controller)
        
        # Trigger - context-dependent action
        if right_controller.trigger > 0.8:
            await self.handle_trigger_action(right_controller)
        
        # Touchpad - navigation
        if right_controller.touchpad != (0.0, 0.0):
            await self.handle_navigation(right_controller.touchpad)
    
    async def cycle_control_mode(self):
        """Cycle through VR control modes"""
        modes = list(VRControlMode)
        current_index = modes.index(self.current_mode)
        next_index = (current_index + 1) % len(modes)
        self.current_mode = modes[next_index]
        
        logger.info(f"Switched to VR control mode: {self.current_mode.value}")
        
        # Update VR display based on new mode
        await self.update_mode_display()
    
    async def update_mode_display(self):
        """Update VR display based on current mode"""
        if self.current_mode == VRControlMode.OVERVIEW:
            await self.vr_interface.show_overview_mode()
        elif self.current_mode == VRControlMode.DIRECT_CONTROL:
            await self.vr_interface.show_direct_control_mode()
        elif self.current_mode == VRControlMode.FORMATION:
            await self.vr_interface.show_formation_mode()
        elif self.current_mode == VRControlMode.TELEMETRY:
            await self.vr_interface.show_telemetry_mode()
    
    def world_to_vr_coordinates(self, world_pos) -> Tuple[float, float, float]:
        """Convert world coordinates to VR space coordinates"""
        # In a real implementation, this would handle proper coordinate transformation
        # including GPS to local coordinates conversion
        
        if hasattr(world_pos, 'latitude'):
            # GPS coordinates - simplified conversion
            x = (world_pos.longitude - (-122.4194)) * 111320 * self.scale_factor
            z = (world_pos.latitude - 37.7749) * 111320 * self.scale_factor
            y = world_pos.altitude * self.scale_factor
        else:
            # Already in local coordinates
            x = world_pos.get("x", 0.0) * self.scale_factor
            y = world_pos.get("y", 0.0) * self.scale_factor
            z = world_pos.get("z", 0.0) * self.scale_factor
        
        return (x, y, z)
    
    def vr_to_world_coordinates(self, vr_pos) -> Dict[str, float]:
        """Convert VR space coordinates to world coordinates"""
        # Convert back to world scale
        world_x = vr_pos["x"] / self.scale_factor
        world_y = vr_pos["y"] / self.scale_factor
        world_z = vr_pos["z"] / self.scale_factor
        
        # Convert to GPS coordinates (simplified)
        longitude = -122.4194 + (world_x / 111320)
        latitude = 37.7749 + (world_z / 111320)
        altitude = world_y
        
        return {
            "latitude": latitude,
            "longitude": longitude,
            "altitude": altitude
        }
    
    async def update_vr_display(self):
        """Update VR display elements"""
        # Update HUD elements
        await self.vr_interface.update_hud({
            "mode": self.current_mode.value,
            "selected_drones": len(self.selected_drones),
            "total_drones": len(self.drone_systems),
            "timestamp": time.time()
        })
        
        # Update drone trails
        for drone_id, vr_drone in self.vr_drones.items():
            if vr_drone.trail_points:
                await self.vr_interface.update_drone_trail(drone_id, vr_drone.trail_points)


async def main():
    """Main VR control interface example"""
    logger.info("Starting Constellation Overwatch SDK VR Control Interface Example")
    
    # Initialize SDK
    sdk = ConstellationOverwatchSDK()
    config = {
        "simulation": True,
        "vr_enabled": True,
        "graphics_quality": "high"
    }
    
    if not await sdk.initialize(config):
        logger.error("Failed to initialize SDK")
        return
    
    # VR configuration
    vr_config = {
        "headset_type": "auto_detect",
        "tracking_space": "room_scale",
        "render_resolution": [2160, 1200],  # Per eye
        "refresh_rate": 90,
        "comfort_settings": {
            "snap_turning": True,
            "teleport_locomotion": True,
            "comfort_vignette": True
        }
    }
    
    # Create VR controller
    vr_controller = VRDroneController("vr_main_controller")
    
    if not await vr_controller.initialize(vr_config):
        logger.error("Failed to initialize VR controller")
        return
    
    # Create and register sample drone systems
    from examples.basic_drone import BasicDrone  # Hypothetical import
    
    drone_positions = [
        Position(37.7749, -122.4194, 0.0, 0.0),
        Position(37.7750, -122.4195, 0.0, 0.0),
        Position(37.7748, -122.4193, 0.0, 0.0),
        Position(37.7751, -122.4192, 0.0, 0.0),
    ]
    
    for i, pos in enumerate(drone_positions):
        drone = BasicDrone(f"drone_{i+1}")
        drone.position = pos
        await drone.initialize()
        await vr_controller.register_drone_system(drone)
    
    logger.info("VR environment ready. Put on your VR headset to begin drone control.")
    logger.info("Available gestures:")
    logger.info("- Point and trigger: Send drone to location")
    logger.info("- Spread hands: Expand formation")
    logger.info("- Contract hands: Contract formation")
    logger.info("- Upward swoop: Takeoff")
    logger.info("- Circular motion: Patrol")
    logger.info("- Both hands up: Emergency stop")
    logger.info("- Downward motion: Land")
    
    # Start VR update loop
    await vr_controller.update_vr_loop()


if __name__ == "__main__":
    asyncio.run(main())
