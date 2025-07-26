"""
Constellation Overwatch SDK - Decision Making Module

<!-- DEVTEAM: AI-powered decision making for autonomous systems -->
<!-- DEVTEAM: Provides strategic planning and tactical decision capabilities -->

Decision making framework for autonomous drone systems including mission planning,
risk assessment, multi-objective optimization, and tactical decision processing.
"""

from typing import Dict, List, Optional, Tuple, Any, Union
import numpy as np
from dataclasses import dataclass
from enum import Enum
import logging
import time

logger = logging.getLogger(__name__)

# Core decision making components
from .mission_planner import MissionPlanner, MissionPlan, MissionObjective
from .risk_assessor import RiskAssessor, RiskAssessment, RiskLevel
from .tactical_decision import TacticalDecisionMaker, TacticalDecision
from .multi_objective import MultiObjectiveOptimizer, OptimizationResult, ObjectiveFunction

__all__ = [
    'DecisionMakingSystem',
    'DecisionType',
    'DecisionPriority',
    'DecisionResult',
    'MissionPlanner',
    'MissionPlan',
    'MissionObjective',
    'RiskAssessor',
    'RiskAssessment',
    'RiskLevel',
    'TacticalDecisionMaker',
    'TacticalDecision',
    'MultiObjectiveOptimizer',
    'create_decision_system'
]


class DecisionType(Enum):
    """Types of decisions the system can make"""
    
    # Strategic Decisions
    MISSION_PLANNING = "mission_planning"
    ROUTE_OPTIMIZATION = "route_optimization"
    RESOURCE_ALLOCATION = "resource_allocation"
    
    # Tactical Decisions
    OBSTACLE_AVOIDANCE = "obstacle_avoidance"
    TARGET_ENGAGEMENT = "target_engagement"
    EVASIVE_MANEUVERS = "evasive_maneuvers"
    
    # Operational Decisions
    LANDING_SITE_SELECTION = "landing_site_selection"
    COMMUNICATION_PROTOCOL = "communication_protocol"
    SENSOR_CONFIGURATION = "sensor_configuration"
    
    # Emergency Decisions
    EMERGENCY_LANDING = "emergency_landing"
    ABORT_MISSION = "abort_mission"
    RETURN_TO_BASE = "return_to_base"


class DecisionPriority(Enum):
    """Priority levels for decision processing"""
    
    CRITICAL = "critical"      # Safety-critical decisions
    HIGH = "high"             # Mission-critical decisions
    MEDIUM = "medium"         # Operational decisions
    LOW = "low"              # Optimization decisions


@dataclass
class DecisionContext:
    """Context information for decision making"""
    
    timestamp: float
    vehicle_state: Dict[str, Any]
    mission_state: Dict[str, Any]
    environmental_conditions: Dict[str, Any]
    available_resources: Dict[str, Any]
    constraints: List[str]
    objectives: List[str]


@dataclass
class DecisionResult:
    """Result of a decision making process"""
    
    decision_type: DecisionType
    priority: DecisionPriority
    action: str
    parameters: Dict[str, Any]
    confidence: float
    reasoning: str
    alternatives: List[Dict[str, Any]]
    execution_time: float
    timestamp: float
    context_id: str


class DecisionMakingSystem:
    """
    Main decision making system coordinator
    
    Integrates multiple decision making capabilities including mission planning,
    risk assessment, tactical decisions, and multi-objective optimization.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize decision making system
        
        Args:
            config: System configuration
        """
        self.config = config
        self.decision_history: List[DecisionResult] = []
        self.active_objectives: List[str] = []
        
        # Initialize subsystems
        self.mission_planner = MissionPlanner(config.get("mission_planner", {}))
        self.risk_assessor = RiskAssessor(config.get("risk_assessor", {}))
        self.tactical_decision_maker = TacticalDecisionMaker(config.get("tactical", {}))
        self.multi_objective_optimizer = MultiObjectiveOptimizer(config.get("optimizer", {}))
        
        logger.info("Decision making system initialized")
    
    def make_decision(
        self, 
        decision_type: DecisionType,
        context: DecisionContext,
        priority: DecisionPriority = DecisionPriority.MEDIUM
    ) -> DecisionResult:
        """
        Make a decision based on type and context
        
        Args:
            decision_type: Type of decision to make
            context: Current context information
            priority: Decision priority level
            
        Returns:
            Decision result with action and reasoning
        """
        start_time = time.time()
        
        try:
            # Route to appropriate decision maker
            if decision_type in [DecisionType.MISSION_PLANNING, DecisionType.ROUTE_OPTIMIZATION]:
                result = self._handle_strategic_decision(decision_type, context, priority)
            elif decision_type in [DecisionType.OBSTACLE_AVOIDANCE, DecisionType.TARGET_ENGAGEMENT]:
                result = self._handle_tactical_decision(decision_type, context, priority)
            elif decision_type in [DecisionType.EMERGENCY_LANDING, DecisionType.ABORT_MISSION]:
                result = self._handle_emergency_decision(decision_type, context, priority)
            else:
                result = self._handle_operational_decision(decision_type, context, priority)
            
            # Calculate execution time
            result.execution_time = time.time() - start_time
            result.timestamp = time.time()
            
            # Store in history
            self.decision_history.append(result)
            
            logger.info(f"Decision made: {decision_type.value} -> {result.action}")
            return result
            
        except Exception as e:
            logger.error(f"Error making decision {decision_type.value}: {e}")
            return self._create_fallback_decision(decision_type, context, priority)
    
    def _handle_strategic_decision(
        self, 
        decision_type: DecisionType, 
        context: DecisionContext,
        priority: DecisionPriority
    ) -> DecisionResult:
        """Handle strategic level decisions"""
        
        if decision_type == DecisionType.MISSION_PLANNING:
            # Use mission planner
            objectives = [MissionObjective(obj_type="waypoint", parameters={}) for obj in context.objectives]
            mission_plan = self.mission_planner.create_mission_plan(objectives, context)
            
            return DecisionResult(
                decision_type=decision_type,
                priority=priority,
                action="execute_mission_plan",
                parameters={"plan": mission_plan.to_dict() if mission_plan else {}},
                confidence=0.8,
                reasoning="Generated optimal mission plan based on objectives and constraints",
                alternatives=[],
                execution_time=0.0,
                timestamp=0.0,
                context_id=f"ctx_{time.time()}"
            )
        
        elif decision_type == DecisionType.ROUTE_OPTIMIZATION:
            # Use multi-objective optimizer for route planning
            objectives = [
                ObjectiveFunction("minimize_distance", weight=0.4),
                ObjectiveFunction("minimize_risk", weight=0.4),
                ObjectiveFunction("minimize_energy", weight=0.2)
            ]
            
            optimization_result = self.multi_objective_optimizer.optimize(objectives, context)
            
            return DecisionResult(
                decision_type=decision_type,
                priority=priority,
                action="follow_optimized_route",
                parameters={"route": optimization_result.solution if optimization_result else {}},
                confidence=optimization_result.confidence if optimization_result else 0.5,
                reasoning="Optimized route for multiple objectives",
                alternatives=optimization_result.alternatives if optimization_result else [],
                execution_time=0.0,
                timestamp=0.0,
                context_id=f"ctx_{time.time()}"
            )
        
        return self._create_fallback_decision(decision_type, context, priority)
    
    def _handle_tactical_decision(
        self, 
        decision_type: DecisionType, 
        context: DecisionContext,
        priority: DecisionPriority
    ) -> DecisionResult:
        """Handle tactical level decisions"""
        
        # Assess risks first
        risk_assessment = self.risk_assessor.assess_risks(context)
        
        # Make tactical decision
        tactical_decision = self.tactical_decision_maker.make_decision(
            decision_type, context, risk_assessment
        )
        
        if tactical_decision:
            return DecisionResult(
                decision_type=decision_type,
                priority=priority,
                action=tactical_decision.action,
                parameters=tactical_decision.parameters,
                confidence=tactical_decision.confidence,
                reasoning=tactical_decision.reasoning,
                alternatives=tactical_decision.alternatives,
                execution_time=0.0,
                timestamp=0.0,
                context_id=f"ctx_{time.time()}"
            )
        
        return self._create_fallback_decision(decision_type, context, priority)
    
    def _handle_emergency_decision(
        self, 
        decision_type: DecisionType, 
        context: DecisionContext,
        priority: DecisionPriority
    ) -> DecisionResult:
        """Handle emergency decisions with high priority"""
        
        if decision_type == DecisionType.EMERGENCY_LANDING:
            return DecisionResult(
                decision_type=decision_type,
                priority=DecisionPriority.CRITICAL,
                action="initiate_emergency_landing",
                parameters={
                    "landing_mode": "immediate",
                    "site_selection": "nearest_safe_area"
                },
                confidence=0.9,
                reasoning="Emergency landing required due to critical system state",
                alternatives=[],
                execution_time=0.0,
                timestamp=0.0,
                context_id=f"ctx_{time.time()}"
            )
        
        elif decision_type == DecisionType.ABORT_MISSION:
            return DecisionResult(
                decision_type=decision_type,
                priority=DecisionPriority.CRITICAL,
                action="abort_current_mission",
                parameters={"return_mode": "direct_path"},
                confidence=0.95,
                reasoning="Mission abort required due to unacceptable risk level",
                alternatives=[],
                execution_time=0.0,
                timestamp=0.0,
                context_id=f"ctx_{time.time()}"
            )
        
        return self._create_fallback_decision(decision_type, context, priority)
    
    def _handle_operational_decision(
        self, 
        decision_type: DecisionType, 
        context: DecisionContext,
        priority: DecisionPriority
    ) -> DecisionResult:
        """Handle operational level decisions"""
        
        return DecisionResult(
            decision_type=decision_type,
            priority=priority,
            action="continue_current_operation",
            parameters={},
            confidence=0.6,
            reasoning="Standard operational decision",
            alternatives=[],
            execution_time=0.0,
            timestamp=0.0,
            context_id=f"ctx_{time.time()}"
        )
    
    def _create_fallback_decision(
        self, 
        decision_type: DecisionType, 
        context: DecisionContext,
        priority: DecisionPriority
    ) -> DecisionResult:
        """Create fallback decision when primary decision making fails"""
        
        return DecisionResult(
            decision_type=decision_type,
            priority=priority,
            action="maintain_current_state",
            parameters={},
            confidence=0.3,
            reasoning="Fallback decision due to insufficient information or processing error",
            alternatives=[],
            execution_time=0.0,
            timestamp=0.0,
            context_id=f"ctx_{time.time()}"
        )
    
    def get_decision_history(self, decision_type: Optional[DecisionType] = None) -> List[DecisionResult]:
        """
        Get decision history, optionally filtered by type
        
        Args:
            decision_type: Optional filter by decision type
            
        Returns:
            List of decision results
        """
        if decision_type:
            return [d for d in self.decision_history if d.decision_type == decision_type]
        return self.decision_history.copy()
    
    def set_objectives(self, objectives: List[str]):
        """
        Set active mission objectives
        
        Args:
            objectives: List of objective descriptions
        """
        self.active_objectives = objectives
        logger.info(f"Active objectives updated: {objectives}")
    
    def evaluate_decision_quality(self, decision_result: DecisionResult) -> float:
        """
        Evaluate the quality of a past decision
        
        Args:
            decision_result: Decision to evaluate
            
        Returns:
            Quality score between 0 and 1
        """
        # Simple quality evaluation based on confidence and outcome
        # In practice, this would use feedback from mission outcomes
        base_quality = decision_result.confidence
        
        # Adjust based on priority handling
        if decision_result.priority == DecisionPriority.CRITICAL:
            base_quality *= 1.1  # Critical decisions get slight boost
        
        return min(base_quality, 1.0)
    
    def get_system_statistics(self) -> Dict[str, Any]:
        """Get decision making system statistics"""
        
        total_decisions = len(self.decision_history)
        if total_decisions == 0:
            return {"total_decisions": 0}
        
        # Calculate statistics
        avg_confidence = sum(d.confidence for d in self.decision_history) / total_decisions
        avg_execution_time = sum(d.execution_time for d in self.decision_history) / total_decisions
        
        decision_types = {}
        priorities = {}
        
        for decision in self.decision_history:
            decision_types[decision.decision_type.value] = decision_types.get(decision.decision_type.value, 0) + 1
            priorities[decision.priority.value] = priorities.get(decision.priority.value, 0) + 1
        
        return {
            "total_decisions": total_decisions,
            "average_confidence": avg_confidence,
            "average_execution_time": avg_execution_time,
            "decision_type_distribution": decision_types,
            "priority_distribution": priorities,
            "active_objectives": len(self.active_objectives)
        }


def create_decision_system(config: Optional[Dict[str, Any]] = None) -> DecisionMakingSystem:
    """
    Create decision making system with default configuration
    
    Args:
        config: Optional configuration dictionary
        
    Returns:
        Configured DecisionMakingSystem instance
    """
    default_config = {
        "mission_planner": {
            "planning_horizon": 3600,  # 1 hour
            "waypoint_tolerance": 5.0   # meters
        },
        "risk_assessor": {
            "risk_threshold": 0.7,
            "assessment_interval": 1.0  # seconds
        },
        "tactical": {
            "reaction_time": 0.1,      # seconds
            "safety_margin": 10.0      # meters
        },
        "optimizer": {
            "max_iterations": 100,
            "convergence_threshold": 0.01
        }
    }
    
    if config:
        default_config.update(config)
    
    return DecisionMakingSystem(default_config) 