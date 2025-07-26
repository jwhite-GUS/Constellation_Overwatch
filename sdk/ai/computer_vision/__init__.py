"""
Constellation Overwatch SDK - Computer Vision Module

<!-- DEVTEAM: Core computer vision capabilities for autonomous systems -->
<!-- DEVTEAM: Maintains professional tone and government-appropriate standards -->

This module provides computer vision capabilities for autonomous drone systems including
object detection, tracking, image classification, and semantic segmentation.
"""

from typing import Dict, List, Optional, Tuple, Any, Union
import numpy as np
from dataclasses import dataclass
from enum import Enum
import logging

# Set up logging
logger = logging.getLogger(__name__)

# Core computer vision components
from .object_detection import ObjectDetector, DetectionResult
from .image_classification import ImageClassifier, ClassificationResult  
from .tracking import ObjectTracker, TrackingResult
from .segmentation import SemanticSegmentator, SegmentationResult
from .preprocessing import ImagePreprocessor
from .pipeline import VisionPipeline

__all__ = [
    'ObjectDetector',
    'DetectionResult', 
    'ImageClassifier',
    'ClassificationResult',
    'ObjectTracker',
    'TrackingResult',
    'SemanticSegmentator',
    'SegmentationResult',
    'ImagePreprocessor',
    'VisionPipeline',
    'VisionCapability',
    'ImageFormat',
    'ProcessingMode'
]


class VisionCapability(Enum):
    """Computer vision capabilities available in the system"""
    
    OBJECT_DETECTION = "object_detection"
    IMAGE_CLASSIFICATION = "image_classification"
    OBJECT_TRACKING = "object_tracking"
    SEMANTIC_SEGMENTATION = "semantic_segmentation"
    OPTICAL_FLOW = "optical_flow"
    STEREO_VISION = "stereo_vision"
    SLAM = "simultaneous_localization_mapping"


class ImageFormat(Enum):
    """Supported image formats for processing"""
    
    RGB = "rgb"
    BGR = "bgr"
    GRAYSCALE = "grayscale"
    THERMAL = "thermal"
    DEPTH = "depth"
    INFRARED = "infrared"


class ProcessingMode(Enum):
    """Processing modes for computer vision operations"""
    
    REAL_TIME = "real_time"
    BATCH = "batch"
    HIGH_ACCURACY = "high_accuracy"
    LOW_LATENCY = "low_latency"


@dataclass
class VisionConfig:
    """Configuration for computer vision operations"""
    
    capabilities: List[VisionCapability]
    input_format: ImageFormat
    processing_mode: ProcessingMode
    max_fps: int = 30
    resolution: Tuple[int, int] = (640, 480)
    confidence_threshold: float = 0.5
    enable_gpu: bool = True
    model_path: Optional[str] = None
    

class VisionSystem:
    """
    Main computer vision system coordinator
    
    Manages multiple vision capabilities and provides unified interface
    for autonomous systems computer vision operations.
    """
    
    def __init__(self, config: VisionConfig):
        """
        Initialize vision system with configuration
        
        Args:
            config: Vision system configuration
        """
        self.config = config
        self.capabilities: Dict[VisionCapability, Any] = {}
        self._initialize_capabilities()
        
        logger.info(f"Vision system initialized with capabilities: {config.capabilities}")
    
    def _initialize_capabilities(self):
        """Initialize requested vision capabilities"""
        
        for capability in self.config.capabilities:
            if capability == VisionCapability.OBJECT_DETECTION:
                self.capabilities[capability] = ObjectDetector(self.config)
            elif capability == VisionCapability.IMAGE_CLASSIFICATION:
                self.capabilities[capability] = ImageClassifier(self.config)
            elif capability == VisionCapability.OBJECT_TRACKING:
                self.capabilities[capability] = ObjectTracker(self.config)
            elif capability == VisionCapability.SEMANTIC_SEGMENTATION:
                self.capabilities[capability] = SemanticSegmentator(self.config)
            else:
                logger.warning(f"Capability {capability} not yet implemented")
    
    def process_frame(self, image: np.ndarray) -> Dict[VisionCapability, Any]:
        """
        Process single image frame through all configured capabilities
        
        Args:
            image: Input image as numpy array
            
        Returns:
            Dictionary mapping capabilities to their results
        """
        results = {}
        
        for capability, processor in self.capabilities.items():
            try:
                result = processor.process(image)
                results[capability] = result
            except Exception as e:
                logger.error(f"Error processing {capability}: {e}")
                results[capability] = None
        
        return results
    
    def get_capability(self, capability: VisionCapability) -> Optional[Any]:
        """
        Get specific vision capability processor
        
        Args:
            capability: Requested capability
            
        Returns:
            Capability processor or None if not available
        """
        return self.capabilities.get(capability)
    
    def is_capability_available(self, capability: VisionCapability) -> bool:
        """
        Check if specific capability is available
        
        Args:
            capability: Capability to check
            
        Returns:
            True if capability is available
        """
        return capability in self.capabilities


# Convenience function for quick vision system setup
def create_vision_system(
    capabilities: List[VisionCapability],
    processing_mode: ProcessingMode = ProcessingMode.REAL_TIME,
    **kwargs
) -> VisionSystem:
    """
    Create vision system with common configuration
    
    Args:
        capabilities: List of vision capabilities to enable
        processing_mode: Processing mode for operations
        **kwargs: Additional configuration parameters
        
    Returns:
        Configured VisionSystem instance
    """
    config = VisionConfig(
        capabilities=capabilities,
        input_format=ImageFormat.RGB,
        processing_mode=processing_mode,
        **kwargs
    )
    
    return VisionSystem(config) 