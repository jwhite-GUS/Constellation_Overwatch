#!/usr/bin/env python3
"""
Text-to-Operations (LLM Integration) Example

This example demonstrates natural language processing for drone operations,
allowing operators to control drone missions using conversational commands
and receiving intelligent responses from the AI system.
"""

import asyncio
import logging
import json
import re
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import time

# Import from the SDK core and AI modules
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "..", "sdk"))

from core import ConstellationOverwatchSDK, AutonomousSystem, Position, SystemStatus

from ai import (
    NaturalLanguageInterface,
    AIInferenceResult,
    DecisionMakingInterface,
    DecisionContext,
)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class CommandType(Enum):
    """Types of drone commands"""

    TAKEOFF = "takeoff"
    LAND = "land"
    GOTO = "goto"
    SEARCH = "search"
    PATROL = "patrol"
    FOLLOW = "follow"
    RETURN = "return"
    HOVER = "hover"
    EMERGENCY = "emergency"
    STATUS = "status"
    MISSION_PLAN = "mission_plan"
    FORMATION = "formation"
    UNKNOWN = "unknown"


@dataclass
class ParsedCommand:
    """Parsed natural language command"""

    command_type: CommandType
    parameters: Dict[str, Any]
    confidence: float
    original_text: str
    drone_targets: List[str]  # Which drones this command applies to
    priority: str  # "low", "medium", "high", "emergency"
    estimated_duration: float  # Estimated execution time in seconds


@dataclass
class ConversationContext:
    """Context for maintaining conversation state"""

    operator_id: str
    conversation_id: str
    mission_context: Dict[str, Any]
    recent_commands: List[ParsedCommand]
    drone_states: Dict[str, Dict[str, Any]]
    current_objectives: List[str]
    safety_constraints: Dict[str, Any]
    timestamp: float


class AdvancedNLPProcessor(NaturalLanguageInterface):
    """Advanced NLP processor with context awareness and intent recognition"""

    def __init__(self, model_id: str):
        super().__init__(model_id)
        self.command_patterns = {}
        self.context_memory = {}
        self.entity_patterns = {}
        self.conversation_contexts = {}

    async def load_model(self, model_path: str, config: Dict[str, Any]) -> bool:
        """Load advanced NLP model"""
        logger.info(f"Loading advanced NLP model from {model_path}")

        # Enhanced command patterns with context
        self.command_patterns = {
            "takeoff": {
                "patterns": [
                    r"take\s+off|launch|lift\s+off|ascend|go\s+up",
                    r"start\s+flying|begin\s+mission|get\s+airborne",
                ],
                "parameters": ["altitude", "speed", "formation"],
                "context_dependent": True,
            },
            "land": {
                "patterns": [
                    r"land|descend|go\s+down|touch\s+down|come\s+down",
                    r"return\s+to\s+ground|end\s+flight",
                ],
                "parameters": ["location", "speed", "approach_type"],
                "context_dependent": True,
            },
            "goto": {
                "patterns": [
                    r"go\s+to|fly\s+to|navigate\s+to|move\s+to|head\s+to",
                    r"proceed\s+to|travel\s+to|approach",
                ],
                "parameters": ["location", "coordinates", "altitude", "speed"],
                "context_dependent": True,
            },
            "search": {
                "patterns": [
                    r"search|look\s+for|find|scan|locate|hunt\s+for",
                    r"investigate|survey|examine",
                ],
                "parameters": ["target", "area", "pattern", "radius"],
                "context_dependent": True,
            },
            "patrol": {
                "patterns": [
                    r"patrol|circle|monitor|watch|guard|secure",
                    r"keep\s+watch|stand\s+guard",
                ],
                "parameters": ["area", "pattern", "duration", "altitude"],
                "context_dependent": True,
            },
            "follow": {
                "patterns": [
                    r"follow|track|pursue|tail|shadow|chase",
                    r"stay\s+with|accompany",
                ],
                "parameters": ["target", "distance", "speed", "formation"],
                "context_dependent": True,
            },
            "return": {
                "patterns": [
                    r"return|come\s+back|go\s+home|return\s+to\s+base",
                    r"abort\s+mission|head\s+back",
                ],
                "parameters": ["route", "speed", "formation"],
                "context_dependent": False,
            },
            "emergency": {
                "patterns": [
                    r"emergency|abort|stop|halt|freeze|immediate",
                    r"urgent|critical|help|mayday",
                ],
                "parameters": ["action", "reason"],
                "context_dependent": False,
            },
            "status": {
                "patterns": [
                    r"status|report|situation|update|condition",
                    r"how\s+are|what\s+is|tell\s+me",
                ],
                "parameters": ["detail_level", "specific_system"],
                "context_dependent": True,
            },
            "mission_plan": {
                "patterns": [
                    r"plan\s+mission|create\s+mission|design\s+operation",
                    r"set\s+up\s+mission|prepare\s+operation",
                ],
                "parameters": ["objective", "area", "resources", "timeline"],
                "context_dependent": True,
            },
        }

        # Entity extraction patterns
        self.entity_patterns = {
            "coordinates": r"(-?\d+\.?\d*),?\s*(-?\d+\.?\d*)",
            "altitude": r"(\d+)\s*(meters?|feet|ft|m)\s*(altitude|high|up)?",
            "speed": r"(\d+)\s*(mph|kph|m/s|knots?)",
            "time": r"(\d+)\s*(seconds?|minutes?|hours?|sec|min|hr)",
            "distance": r"(\d+)\s*(meters?|feet|km|miles?|ft|m)",
            "direction": r"(north|south|east|west|northeast|northwest|southeast|southwest)",
            "drone_id": r"drone\s*(\d+)|unit\s*(\d+)|aircraft\s*(\d+)",
            "formation": r"(line|column|diamond|square|circle|triangle|wedge)\s*formation",
        }

        self.is_loaded = True
        logger.info("Advanced NLP model loaded successfully")
        return True

    async def parse_command_with_context(
        self, text: str, context: ConversationContext
    ) -> ParsedCommand:
        """Parse command with conversation context"""
        logger.info(f"Parsing command with context: '{text}'")

        # Clean and normalize text
        text_clean = text.lower().strip()

        # Extract entities first
        entities = await self.extract_entities_advanced(text_clean)

        # Determine command type
        command_type = CommandType.UNKNOWN
        confidence = 0.0

        for cmd_type, cmd_config in self.command_patterns.items():
            for pattern in cmd_config["patterns"]:
                if re.search(pattern, text_clean):
                    command_type = CommandType(cmd_type)
                    confidence = 0.8 + (
                        0.1 if cmd_config.get("context_dependent") else 0.0
                    )
                    break
            if command_type != CommandType.UNKNOWN:
                break

        # Extract parameters based on command type
        parameters = await self.extract_command_parameters(
            text_clean, command_type, entities, context
        )

        # Determine target drones
        drone_targets = self.extract_drone_targets(text_clean, entities, context)

        # Determine priority
        priority = self.determine_priority(text_clean, command_type)

        # Estimate duration
        estimated_duration = self.estimate_command_duration(command_type, parameters)

        return ParsedCommand(
            command_type=command_type,
            parameters=parameters,
            confidence=confidence,
            original_text=text,
            drone_targets=drone_targets,
            priority=priority,
            estimated_duration=estimated_duration,
        )

    async def extract_entities_advanced(self, text: str) -> Dict[str, Any]:
        """Advanced entity extraction with context"""
        entities = {}

        for entity_type, pattern in self.entity_patterns.items():
            matches = re.findall(pattern, text)
            if matches:
                if entity_type == "coordinates":
                    # Handle coordinate pairs
                    coords = []
                    for match in matches:
                        if len(match) == 2:
                            coords.append(
                                {"lat": float(match[0]), "lon": float(match[1])}
                            )
                    entities[entity_type] = coords

                elif entity_type in ["altitude", "speed", "time", "distance"]:
                    # Handle numeric values with units
                    values = []
                    for match in matches:
                        if isinstance(match, tuple):
                            value = float(match[0]) if match[0] else 0.0
                            unit = match[1] if len(match) > 1 else ""
                            values.append({"value": value, "unit": unit})
                        else:
                            values.append({"value": float(match), "unit": ""})
                    entities[entity_type] = values

                else:
                    entities[entity_type] = matches

        # Extract location names and landmarks
        location_keywords = [
            "base",
            "home",
            "target",
            "waypoint",
            "checkpoint",
            "building",
            "tower",
            "bridge",
            "park",
            "field",
            "north",
            "south",
            "east",
            "west",
            "center",
        ]

        locations = []
        for keyword in location_keywords:
            if keyword in text:
                locations.append(keyword)

        if locations:
            entities["locations"] = locations

        return entities

    async def extract_command_parameters(
        self,
        text: str,
        command_type: CommandType,
        entities: Dict[str, Any],
        context: ConversationContext,
    ) -> Dict[str, Any]:
        """Extract command-specific parameters"""
        parameters = {}

        if command_type == CommandType.TAKEOFF:
            # Extract altitude
            if "altitude" in entities:
                alt_data = entities["altitude"][0]
                altitude = alt_data["value"]
                if alt_data["unit"] in ["feet", "ft"]:
                    altitude = altitude * 0.3048  # Convert to meters
                parameters["altitude"] = altitude
            else:
                parameters["altitude"] = 50.0  # Default altitude

            # Extract formation
            if "formation" in entities:
                parameters["formation"] = entities["formation"][0]

        elif command_type == CommandType.GOTO:
            # Extract destination
            if "coordinates" in entities:
                coord = entities["coordinates"][0]
                parameters["destination"] = {
                    "latitude": coord["lat"],
                    "longitude": coord["lon"],
                }
            elif "locations" in entities:
                location = entities["locations"][0]
                parameters["destination"] = await self.resolve_location_name(
                    location, context
                )

            # Extract altitude if specified
            if "altitude" in entities:
                alt_data = entities["altitude"][0]
                altitude = alt_data["value"]
                if alt_data["unit"] in ["feet", "ft"]:
                    altitude = altitude * 0.3048
                parameters["altitude"] = altitude

        elif command_type == CommandType.SEARCH:
            # Extract search target
            search_targets = ["person", "vehicle", "object", "survivor", "target"]
            for target in search_targets:
                if target in text:
                    parameters["target"] = target
                    break

            # Extract search area
            if "distance" in entities:
                dist_data = entities["distance"][0]
                radius = dist_data["value"]
                if dist_data["unit"] in ["feet", "ft"]:
                    radius = radius * 0.3048
                parameters["search_radius"] = radius

            # Extract search pattern
            patterns = ["grid", "spiral", "random", "systematic"]
            for pattern in patterns:
                if pattern in text:
                    parameters["search_pattern"] = pattern
                    break

        elif command_type == CommandType.PATROL:
            # Extract patrol area
            if "distance" in entities:
                dist_data = entities["distance"][0]
                radius = dist_data["value"]
                parameters["patrol_radius"] = radius

            # Extract patrol duration
            if "time" in entities:
                time_data = entities["time"][0]
                duration = time_data["value"]
                unit = time_data["unit"]
                if unit in ["minutes", "min"]:
                    duration = duration * 60
                elif unit in ["hours", "hr"]:
                    duration = duration * 3600
                parameters["duration"] = duration

        elif command_type == CommandType.FOLLOW:
            # Extract follow target
            if "drone_id" in entities:
                target_id = entities["drone_id"][0]
                parameters["target"] = f"drone_{target_id}"

            # Extract follow distance
            if "distance" in entities:
                dist_data = entities["distance"][0]
                distance = dist_data["value"]
                parameters["follow_distance"] = distance

        return parameters

    async def resolve_location_name(
        self, location_name: str, context: ConversationContext
    ) -> Dict[str, float]:
        """Resolve location name to coordinates"""
        # In a real implementation, this would use a gazetteer or map service
        known_locations = {
            "base": {"latitude": 37.7749, "longitude": -122.4194},
            "home": {"latitude": 37.7749, "longitude": -122.4194},
            "target": {"latitude": 37.7759, "longitude": -122.4184},
            "waypoint": {"latitude": 37.7739, "longitude": -122.4204},
        }

        return known_locations.get(
            location_name, {"latitude": 37.7749, "longitude": -122.4194}
        )

    def extract_drone_targets(
        self, text: str, entities: Dict[str, Any], context: ConversationContext
    ) -> List[str]:
        """Determine which drones the command applies to"""
        targets = []

        # Check for specific drone IDs
        if "drone_id" in entities:
            for drone_id in entities["drone_id"]:
                targets.append(f"drone_{drone_id}")

        # Check for group references
        if "all" in text or "everyone" in text or "entire" in text:
            # Apply to all available drones
            targets = list(context.drone_states.keys())
        elif "formation" in text or "swarm" in text:
            # Apply to formation drones
            targets = [
                drone_id
                for drone_id, state in context.drone_states.items()
                if state.get("in_formation", False)
            ]

        # Default to all drones if none specified
        if not targets:
            targets = list(context.drone_states.keys())

        return targets

    def determine_priority(self, text: str, command_type: CommandType) -> str:
        """Determine command priority"""
        if command_type == CommandType.EMERGENCY:
            return "emergency"

        emergency_keywords = ["urgent", "immediate", "critical", "emergency", "now"]
        high_keywords = ["quickly", "fast", "asap", "priority"]

        if any(keyword in text for keyword in emergency_keywords):
            return "emergency"
        elif any(keyword in text for keyword in high_keywords):
            return "high"
        elif command_type in [
            CommandType.TAKEOFF,
            CommandType.LAND,
            CommandType.RETURN,
        ]:
            return "medium"
        else:
            return "low"

    def estimate_command_duration(
        self, command_type: CommandType, parameters: Dict[str, Any]
    ) -> float:
        """Estimate command execution duration"""
        base_durations = {
            CommandType.TAKEOFF: 30.0,
            CommandType.LAND: 45.0,
            CommandType.GOTO: 60.0,
            CommandType.SEARCH: 300.0,
            CommandType.PATROL: 600.0,
            CommandType.FOLLOW: 0.0,  # Continuous
            CommandType.RETURN: 120.0,
            CommandType.HOVER: 0.0,  # Until next command
            CommandType.EMERGENCY: 5.0,
            CommandType.STATUS: 2.0,
            CommandType.MISSION_PLAN: 10.0,
        }

        base_time = base_durations.get(command_type, 60.0)

        # Adjust based on parameters
        if "duration" in parameters:
            return parameters["duration"]
        elif "search_radius" in parameters:
            # Larger search areas take longer
            radius = parameters["search_radius"]
            return base_time + (radius * 2.0)  # 2 seconds per meter radius

        return base_time


class ConversationalDroneController:
    """Conversational drone controller with LLM integration"""

    def __init__(self, controller_id: str):
        self.controller_id = controller_id
        self.nlp_processor = AdvancedNLPProcessor("conversational_nlp")
        self.conversation_contexts = {}
        self.active_missions = {}
        self.drone_systems = {}

    async def initialize(self) -> bool:
        """Initialize the conversational controller"""
        logger.info(
            f"Initializing conversational drone controller: {self.controller_id}"
        )

        # Load NLP model
        config = {
            "model_type": "conversational",
            "context_window": 1000,
            "memory_size": 10000,
        }

        await self.nlp_processor.load_model("models/conversational_nlp.bin", config)

        logger.info("Conversational drone controller initialized")
        return True

    async def register_drone_system(self, drone_system: AutonomousSystem):
        """Register a drone system for control"""
        self.drone_systems[drone_system.system_id] = drone_system
        logger.info(f"Registered drone system: {drone_system.system_id}")

    async def start_conversation(self, operator_id: str) -> str:
        """Start a new conversation with an operator"""
        conversation_id = f"conv_{operator_id}_{int(time.time())}"

        context = ConversationContext(
            operator_id=operator_id,
            conversation_id=conversation_id,
            mission_context={},
            recent_commands=[],
            drone_states={
                drone_id: {"status": "ready", "in_formation": False}
                for drone_id in self.drone_systems.keys()
            },
            current_objectives=[],
            safety_constraints={
                "max_altitude": 100.0,
                "max_speed": 15.0,
                "no_fly_zones": [],
                "weather_limits": {"max_wind": 8.0},
            },
            timestamp=time.time(),
        )

        self.conversation_contexts[conversation_id] = context

        welcome_message = (
            f"Hello! I'm your AI drone operations assistant. "
            f"I'm currently managing {len(self.drone_systems)} drone(s). "
            f"You can give me commands in natural language, and I'll coordinate the missions. "
            f"How can I help you today?"
        )

        logger.info(
            f"Started conversation {conversation_id} with operator {operator_id}"
        )
        return welcome_message

    async def process_natural_language_command(
        self, conversation_id: str, text: str
    ) -> Dict[str, Any]:
        """Process natural language command and execute drone operations"""
        if conversation_id not in self.conversation_contexts:
            return {
                "error": "Invalid conversation ID",
                "response": "Please start a new conversation session.",
            }

        context = self.conversation_contexts[conversation_id]

        try:
            # Parse command with context
            parsed_command = await self.nlp_processor.parse_command_with_context(
                text, context
            )

            # Log the parsed command
            logger.info(
                f"Parsed command: {parsed_command.command_type.value} "
                f"(confidence: {parsed_command.confidence:.2f})"
            )

            # Execute command
            execution_result = await self.execute_parsed_command(
                parsed_command, context
            )

            # Generate response
            response = await self.generate_response(
                parsed_command, execution_result, context
            )

            # Update conversation context
            context.recent_commands.append(parsed_command)
            context.timestamp = time.time()

            # Keep only last 10 commands in memory
            if len(context.recent_commands) > 10:
                context.recent_commands = context.recent_commands[-10:]

            return {
                "success": True,
                "command": {
                    "type": parsed_command.command_type.value,
                    "parameters": parsed_command.parameters,
                    "confidence": parsed_command.confidence,
                    "targets": parsed_command.drone_targets,
                    "priority": parsed_command.priority,
                    "estimated_duration": parsed_command.estimated_duration,
                },
                "execution_result": execution_result,
                "response": response,
            }

        except Exception as e:
            logger.error(f"Error processing command: {e}")
            return {
                "error": str(e),
                "response": f"I'm sorry, I encountered an error processing your command: {str(e)}",
            }

    async def execute_parsed_command(
        self, command: ParsedCommand, context: ConversationContext
    ) -> Dict[str, Any]:
        """Execute the parsed command on target drones"""
        results = {}

        for drone_id in command.drone_targets:
            if drone_id not in self.drone_systems:
                results[drone_id] = {"error": "Drone not found"}
                continue

            drone_system = self.drone_systems[drone_id]

            try:
                # Execute command based on type
                if command.command_type == CommandType.TAKEOFF:
                    altitude = command.parameters.get("altitude", 50.0)
                    success = await drone_system.send_command(
                        "takeoff", {"altitude": altitude}
                    )
                    results[drone_id] = {
                        "success": success,
                        "action": "takeoff",
                        "altitude": altitude,
                    }

                elif command.command_type == CommandType.LAND:
                    success = await drone_system.send_command(
                        "land", command.parameters
                    )
                    results[drone_id] = {"success": success, "action": "land"}

                elif command.command_type == CommandType.GOTO:
                    destination = command.parameters.get("destination", {})
                    altitude = command.parameters.get("altitude", 50.0)
                    success = await drone_system.send_command(
                        "goto", {"position": destination, "altitude": altitude}
                    )
                    results[drone_id] = {
                        "success": success,
                        "action": "goto",
                        "destination": destination,
                    }

                elif command.command_type == CommandType.SEARCH:
                    search_params = {
                        "target": command.parameters.get("target", "unknown"),
                        "radius": command.parameters.get("search_radius", 100.0),
                        "pattern": command.parameters.get("search_pattern", "spiral"),
                    }
                    success = await drone_system.send_command("search", search_params)
                    results[drone_id] = {
                        "success": success,
                        "action": "search",
                        "parameters": search_params,
                    }

                elif command.command_type == CommandType.PATROL:
                    patrol_params = {
                        "radius": command.parameters.get("patrol_radius", 50.0),
                        "duration": command.parameters.get("duration", 300.0),
                    }
                    success = await drone_system.send_command("patrol", patrol_params)
                    results[drone_id] = {
                        "success": success,
                        "action": "patrol",
                        "parameters": patrol_params,
                    }

                elif command.command_type == CommandType.RETURN:
                    success = await drone_system.send_command("return_to_base", {})
                    results[drone_id] = {"success": success, "action": "return_to_base"}

                elif command.command_type == CommandType.EMERGENCY:
                    success = await drone_system.send_command("emergency_stop", {})
                    results[drone_id] = {"success": success, "action": "emergency_stop"}

                elif command.command_type == CommandType.STATUS:
                    telemetry = await drone_system.get_telemetry()
                    results[drone_id] = {
                        "success": True,
                        "action": "status",
                        "data": telemetry,
                    }

                else:
                    results[drone_id] = {"error": "Unknown command type"}

            except Exception as e:
                results[drone_id] = {"error": str(e)}

        return results

    async def generate_response(
        self,
        command: ParsedCommand,
        execution_result: Dict[str, Any],
        context: ConversationContext,
    ) -> str:
        """Generate natural language response"""
        success_count = sum(
            1 for result in execution_result.values() if result.get("success", False)
        )
        total_count = len(execution_result)

        # Generate response based on command type and results
        if command.command_type == CommandType.TAKEOFF:
            if success_count == total_count:
                altitude = command.parameters.get("altitude", 50)
                return f"All {total_count} drone(s) are taking off to {altitude} meters altitude."
            else:
                return f"{success_count} of {total_count} drone(s) successfully initiated takeoff. Please check the others."

        elif command.command_type == CommandType.LAND:
            if success_count == total_count:
                return f"All {total_count} drone(s) are landing safely."
            else:
                return f"{success_count} of {total_count} drone(s) are landing. Some may need manual intervention."

        elif command.command_type == CommandType.GOTO:
            dest = command.parameters.get("destination", {})
            if success_count == total_count:
                if "latitude" in dest:
                    return f"All {total_count} drone(s) are navigating to coordinates {dest['latitude']:.4f}, {dest['longitude']:.4f}."
                else:
                    return f"All {total_count} drone(s) are heading to the specified location."
            else:
                return f"{success_count} of {total_count} drone(s) are en route. Others may have navigation issues."

        elif command.command_type == CommandType.SEARCH:
            target = command.parameters.get("target", "target")
            if success_count == total_count:
                return f"All {total_count} drone(s) are beginning search operation for {target}."
            else:
                return f"{success_count} of {total_count} drone(s) have started the search mission."

        elif command.command_type == CommandType.PATROL:
            duration = (
                command.parameters.get("duration", 300) / 60
            )  # Convert to minutes
            if success_count == total_count:
                return f"All {total_count} drone(s) are starting patrol mission for {duration:.1f} minutes."
            else:
                return (
                    f"{success_count} of {total_count} drone(s) have begun patrolling."
                )

        elif command.command_type == CommandType.RETURN:
            if success_count == total_count:
                return f"All {total_count} drone(s) are returning to base."
            else:
                return f"{success_count} of {total_count} drone(s) are heading home. Others may need assistance."

        elif command.command_type == CommandType.EMERGENCY:
            if success_count == total_count:
                return f"Emergency stop executed for all {total_count} drone(s). They are taking immediate safety actions."
            else:
                return f"Emergency stop sent to {success_count} of {total_count} drone(s). Manual intervention may be required for others."

        elif command.command_type == CommandType.STATUS:
            status_reports = []
            for drone_id, result in execution_result.items():
                if result.get("success") and "data" in result:
                    data = result["data"]
                    status = data.get("status", "unknown")
                    battery = data.get("battery_level", "unknown")
                    status_reports.append(f"{drone_id}: {status}, battery {battery}%")

            return f"Status report:\n" + "\n".join(status_reports)

        else:
            if success_count > 0:
                return (
                    f"Command executed for {success_count} of {total_count} drone(s)."
                )
            else:
                return "I'm sorry, I couldn't execute that command. Please try rephrasing or check drone status."


async def main():
    """Main text-to-operations example"""
    logger.info("Starting Constellation Overwatch SDK Text-to-Operations Example")

    # Initialize SDK
    sdk = ConstellationOverwatchSDK()
    config = {"simulation": True, "nlp_enabled": True, "conversation_mode": True}

    if not await sdk.initialize(config):
        logger.error("Failed to initialize SDK")
        return

    # Create conversational controller
    controller = ConversationalDroneController("text_to_ops_controller")
    await controller.initialize()

    # Create and register sample drone systems
    from examples.basic_drone import BasicDrone  # Hypothetical import

    for i in range(3):
        drone = BasicDrone(f"drone_{i+1}")
        await drone.initialize()
        controller.register_drone_system(drone)

    # Start conversation
    conversation_id = await controller.start_conversation("operator_001")
    print(f"Conversation started: {conversation_id}")

    # Sample natural language commands
    test_commands = [
        "Take off all drones to 60 meters",
        "Send drone 1 to search for people in the north area",
        "Have the remaining drones patrol in a circle formation",
        "What's the status of all aircraft?",
        "Bring everyone back to base when the mission is complete",
        "Emergency stop all drones immediately!",
    ]

    print("\n=== Interactive Text-to-Operations Demo ===")

    for i, command in enumerate(test_commands):
        print(f"\nOperator: {command}")

        result = await controller.process_natural_language_command(
            conversation_id, command
        )

        if result.get("success"):
            print(f"AI Assistant: {result['response']}")

            # Show parsed command details
            cmd_info = result["command"]
            print(
                f"  └─ Parsed as: {cmd_info['type']} (confidence: {cmd_info['confidence']:.2f})"
            )
            print(f"     Targets: {', '.join(cmd_info['targets'])}")
            print(f"     Priority: {cmd_info['priority']}")
            print(f"     Duration: {cmd_info['estimated_duration']:.0f}s")
        else:
            print(f"AI Assistant: {result['response']}")

        # Small delay between commands
        await asyncio.sleep(2)

    logger.info("Text-to-operations example completed successfully!")


if __name__ == "__main__":
    asyncio.run(main())
