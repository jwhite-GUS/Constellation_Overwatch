"""
Unit tests for Constellation Overwatch Computer Vision Module

<!-- DEVTEAM: Comprehensive test coverage for computer vision capabilities -->
<!-- DEVTEAM: Follows professional testing standards with proper assertions -->
"""

import pytest
import numpy as np
from unittest.mock import Mock, patch
import time

# Computer vision imports
from sdk.ai.computer_vision import (
    VisionSystem, VisionConfig, VisionCapability, ImageFormat, ProcessingMode,
    create_vision_system
)
from sdk.ai.computer_vision.object_detection import (
    ObjectDetector, DetectionResult, BoundingBox, ObjectClass,
    filter_detections_by_class, non_maximum_suppression
)
from sdk.ai.computer_vision.image_classification import (
    ImageClassifier, ClassificationResult, SceneType
)


class TestVisionSystem:
    """Test cases for VisionSystem class"""
    
    def test_vision_system_initialization(self):
        """Test VisionSystem initialization with default config"""
        config = VisionConfig(
            capabilities=[VisionCapability.OBJECT_DETECTION],
            input_format=ImageFormat.RGB,
            processing_mode=ProcessingMode.REAL_TIME
        )
        
        vision_system = VisionSystem(config)
        
        assert vision_system.config == config
        assert VisionCapability.OBJECT_DETECTION in vision_system.capabilities
        assert vision_system.is_capability_available(VisionCapability.OBJECT_DETECTION)
    
    def test_vision_system_multiple_capabilities(self):
        """Test VisionSystem with multiple capabilities"""
        config = VisionConfig(
            capabilities=[
                VisionCapability.OBJECT_DETECTION,
                VisionCapability.IMAGE_CLASSIFICATION
            ],
            input_format=ImageFormat.RGB,
            processing_mode=ProcessingMode.HIGH_ACCURACY
        )
        
        vision_system = VisionSystem(config)
        
        assert len(vision_system.capabilities) == 2
        assert vision_system.is_capability_available(VisionCapability.OBJECT_DETECTION)
        assert vision_system.is_capability_available(VisionCapability.IMAGE_CLASSIFICATION)
    
    def test_process_frame(self):
        """Test frame processing through vision system"""
        config = VisionConfig(
            capabilities=[VisionCapability.OBJECT_DETECTION],
            input_format=ImageFormat.RGB,
            processing_mode=ProcessingMode.REAL_TIME
        )
        
        vision_system = VisionSystem(config)
        
        # Create test image
        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        results = vision_system.process_frame(test_image)
        
        assert isinstance(results, dict)
        assert VisionCapability.OBJECT_DETECTION in results
    
    def test_get_capability(self):
        """Test getting specific capability processor"""
        config = VisionConfig(
            capabilities=[VisionCapability.OBJECT_DETECTION],
            input_format=ImageFormat.RGB,
            processing_mode=ProcessingMode.REAL_TIME
        )
        
        vision_system = VisionSystem(config)
        detector = vision_system.get_capability(VisionCapability.OBJECT_DETECTION)
        
        assert detector is not None
        assert hasattr(detector, 'process')
    
    def test_create_vision_system_convenience_function(self):
        """Test convenience function for creating vision system"""
        capabilities = [VisionCapability.OBJECT_DETECTION, VisionCapability.IMAGE_CLASSIFICATION]
        
        vision_system = create_vision_system(
            capabilities=capabilities,
            processing_mode=ProcessingMode.HIGH_ACCURACY,
            confidence_threshold=0.7
        )
        
        assert isinstance(vision_system, VisionSystem)
        assert vision_system.config.processing_mode == ProcessingMode.HIGH_ACCURACY
        assert vision_system.config.confidence_threshold == 0.7


class TestObjectDetection:
    """Test cases for object detection functionality"""
    
    def test_bounding_box_creation(self):
        """Test BoundingBox creation and methods"""
        bbox = BoundingBox(0.1, 0.2, 0.8, 0.9)
        
        assert bbox.x1 == 0.1
        assert bbox.y1 == 0.2
        assert bbox.x2 == 0.8
        assert bbox.y2 == 0.9
        
        # Test area calculation
        expected_area = (0.8 - 0.1) * (0.9 - 0.2)
        assert abs(bbox.area() - expected_area) < 1e-6
        
        # Test center calculation
        center = bbox.center()
        assert abs(center[0] - 0.45) < 1e-6  # (0.1 + 0.8) / 2
        assert abs(center[1] - 0.55) < 1e-6  # (0.2 + 0.9) / 2
    
    def test_bounding_box_pixel_coordinates(self):
        """Test bounding box pixel coordinate conversion"""
        bbox = BoundingBox(0.1, 0.2, 0.8, 0.9)
        
        pixel_coords = bbox.to_pixel_coordinates(640, 480)
        
        assert pixel_coords == (64, 96, 512, 432)
    
    def test_detection_result_creation(self):
        """Test DetectionResult creation"""
        bbox = BoundingBox(0.1, 0.2, 0.8, 0.9)
        
        detection = DetectionResult(
            object_class=ObjectClass.PERSON,
            confidence=0.85,
            bounding_box=bbox,
            detection_id="test_detection"
        )
        
        assert detection.object_class == ObjectClass.PERSON
        assert detection.confidence == 0.85
        assert detection.bounding_box == bbox
        assert detection.detection_id == "test_detection"
        assert detection.timestamp is not None
    
    def test_object_detector_initialization(self):
        """Test ObjectDetector initialization"""
        config = VisionConfig(
            capabilities=[VisionCapability.OBJECT_DETECTION],
            input_format=ImageFormat.RGB,
            processing_mode=ProcessingMode.REAL_TIME,
            confidence_threshold=0.6
        )
        
        detector = ObjectDetector(config)
        
        assert detector.config == config
        assert detector.model is not None
        assert detector.detection_count == 0
    
    def test_object_detector_process_image(self):
        """Test object detection on image"""
        config = VisionConfig(
            capabilities=[VisionCapability.OBJECT_DETECTION],
            input_format=ImageFormat.RGB,
            processing_mode=ProcessingMode.REAL_TIME,
            confidence_threshold=0.5
        )
        
        detector = ObjectDetector(config)
        
        # Create test image
        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        detections = detector.process(test_image)
        
        assert isinstance(detections, list)
        # Mock detector might return 0-3 detections
        assert len(detections) <= 3
        
        # If detections exist, verify their structure
        for detection in detections:
            assert isinstance(detection, DetectionResult)
            assert detection.confidence >= config.confidence_threshold
            assert isinstance(detection.object_class, ObjectClass)
    
    def test_object_detector_invalid_image(self):
        """Test object detector with invalid image"""
        config = VisionConfig(
            capabilities=[VisionCapability.OBJECT_DETECTION],
            input_format=ImageFormat.RGB,
            processing_mode=ProcessingMode.REAL_TIME
        )
        
        detector = ObjectDetector(config)
        
        # Test with invalid image (2D instead of 3D)
        invalid_image = np.random.randint(0, 255, (480, 640), dtype=np.uint8)
        
        detections = detector.process(invalid_image)
        
        # Should return empty list for invalid input
        assert detections == []
    
    def test_confidence_threshold_update(self):
        """Test updating confidence threshold"""
        config = VisionConfig(
            capabilities=[VisionCapability.OBJECT_DETECTION],
            input_format=ImageFormat.RGB,
            processing_mode=ProcessingMode.REAL_TIME,
            confidence_threshold=0.5
        )
        
        detector = ObjectDetector(config)
        
        # Update threshold
        detector.set_confidence_threshold(0.8)
        assert detector.config.confidence_threshold == 0.8
        
        # Test invalid threshold
        with pytest.raises(ValueError):
            detector.set_confidence_threshold(1.5)
    
    def test_detection_filtering_by_class(self):
        """Test filtering detections by object class"""
        bbox1 = BoundingBox(0.1, 0.1, 0.5, 0.5)
        bbox2 = BoundingBox(0.6, 0.6, 0.9, 0.9)
        
        detections = [
            DetectionResult(ObjectClass.PERSON, 0.9, bbox1),
            DetectionResult(ObjectClass.CAR, 0.8, bbox2),
            DetectionResult(ObjectClass.PERSON, 0.7, bbox1)
        ]
        
        person_detections = filter_detections_by_class(detections, [ObjectClass.PERSON])
        
        assert len(person_detections) == 2
        assert all(det.object_class == ObjectClass.PERSON for det in person_detections)
    
    def test_non_maximum_suppression(self):
        """Test non-maximum suppression"""
        # Create overlapping bounding boxes
        bbox1 = BoundingBox(0.1, 0.1, 0.5, 0.5)
        bbox2 = BoundingBox(0.15, 0.15, 0.55, 0.55)  # Overlapping with bbox1
        bbox3 = BoundingBox(0.6, 0.6, 0.9, 0.9)      # Non-overlapping
        
        detections = [
            DetectionResult(ObjectClass.PERSON, 0.9, bbox1),
            DetectionResult(ObjectClass.PERSON, 0.7, bbox2),  # Lower confidence, overlapping
            DetectionResult(ObjectClass.PERSON, 0.8, bbox3)
        ]
        
        filtered_detections = non_maximum_suppression(detections, iou_threshold=0.3)
        
        # Should keep highest confidence detection and non-overlapping detection
        assert len(filtered_detections) == 2
        assert filtered_detections[0].confidence == 0.9  # Highest confidence kept
        assert filtered_detections[1].confidence == 0.8  # Non-overlapping kept


class TestImageClassification:
    """Test cases for image classification functionality"""
    
    def test_classification_result_creation(self):
        """Test ClassificationResult creation"""
        probabilities = {
            SceneType.URBAN: 0.8,
            SceneType.RURAL: 0.15,
            SceneType.FOREST: 0.05
        }
        
        result = ClassificationResult(
            scene_type=SceneType.URBAN,
            confidence=0.8,
            class_probabilities=probabilities
        )
        
        assert result.scene_type == SceneType.URBAN
        assert result.confidence == 0.8
        assert result.class_probabilities == probabilities
        assert result.timestamp is not None
    
    def test_image_classifier_initialization(self):
        """Test ImageClassifier initialization"""
        config = VisionConfig(
            capabilities=[VisionCapability.IMAGE_CLASSIFICATION],
            input_format=ImageFormat.RGB,
            processing_mode=ProcessingMode.HIGH_ACCURACY
        )
        
        classifier = ImageClassifier(config)
        
        assert classifier.config == config
        assert classifier.model is not None
        assert classifier.classification_count == 0
    
    def test_image_classifier_process(self):
        """Test image classification processing"""
        config = VisionConfig(
            capabilities=[VisionCapability.IMAGE_CLASSIFICATION],
            input_format=ImageFormat.RGB,
            processing_mode=ProcessingMode.REAL_TIME
        )
        
        classifier = ImageClassifier(config)
        
        # Create test image
        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        result = classifier.process(test_image)
        
        assert isinstance(result, ClassificationResult)
        assert isinstance(result.scene_type, SceneType)
        assert 0.0 <= result.confidence <= 1.0
        assert result.processing_time is not None
        assert result.processing_time > 0
    
    def test_image_classifier_top_predictions(self):
        """Test getting top-k predictions"""
        config = VisionConfig(
            capabilities=[VisionCapability.IMAGE_CLASSIFICATION],
            input_format=ImageFormat.RGB,
            processing_mode=ProcessingMode.HIGH_ACCURACY
        )
        
        classifier = ImageClassifier(config)
        
        # Create test image
        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        top_predictions = classifier.get_top_predictions(test_image, top_k=3)
        
        assert isinstance(top_predictions, list)
        assert len(top_predictions) <= 3
        
        # Verify predictions are sorted by confidence (descending)
        for i in range(len(top_predictions) - 1):
            assert top_predictions[i][1] >= top_predictions[i + 1][1]
    
    def test_image_classifier_statistics(self):
        """Test classifier statistics"""
        config = VisionConfig(
            capabilities=[VisionCapability.IMAGE_CLASSIFICATION],
            input_format=ImageFormat.RGB,
            processing_mode=ProcessingMode.REAL_TIME
        )
        
        classifier = ImageClassifier(config)
        
        # Process some images
        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        classifier.process(test_image)
        classifier.process(test_image)
        
        stats = classifier.get_statistics()
        
        assert stats["total_classifications"] == 2
        assert "model_type" in stats
        assert "supported_classes" in stats


class TestVisionEnums:
    """Test cases for vision enumeration types"""
    
    def test_vision_capability_enum(self):
        """Test VisionCapability enum values"""
        assert VisionCapability.OBJECT_DETECTION.value == "object_detection"
        assert VisionCapability.IMAGE_CLASSIFICATION.value == "image_classification"
        assert VisionCapability.OBJECT_TRACKING.value == "object_tracking"
    
    def test_image_format_enum(self):
        """Test ImageFormat enum values"""
        assert ImageFormat.RGB.value == "rgb"
        assert ImageFormat.BGR.value == "bgr"
        assert ImageFormat.GRAYSCALE.value == "grayscale"
    
    def test_processing_mode_enum(self):
        """Test ProcessingMode enum values"""
        assert ProcessingMode.REAL_TIME.value == "real_time"
        assert ProcessingMode.HIGH_ACCURACY.value == "high_accuracy"
        assert ProcessingMode.LOW_LATENCY.value == "low_latency"
    
    def test_object_class_enum(self):
        """Test ObjectClass enum values"""
        assert ObjectClass.PERSON.value == "person"
        assert ObjectClass.VEHICLE.value == "vehicle"
        assert ObjectClass.AIRCRAFT.value == "aircraft"
    
    def test_scene_type_enum(self):
        """Test SceneType enum values"""
        assert SceneType.URBAN.value == "urban"
        assert SceneType.RURAL.value == "rural"
        assert SceneType.FOREST.value == "forest"


@pytest.fixture
def sample_image():
    """Fixture providing a sample test image"""
    return np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)


@pytest.fixture
def vision_config():
    """Fixture providing a standard vision configuration"""
    return VisionConfig(
        capabilities=[VisionCapability.OBJECT_DETECTION, VisionCapability.IMAGE_CLASSIFICATION],
        input_format=ImageFormat.RGB,
        processing_mode=ProcessingMode.REAL_TIME,
        confidence_threshold=0.6
    )


class TestVisionConfigFunctionality:
    """Test cases for VisionConfig functionality"""
    
    def test_vision_config_creation(self, vision_config):
        """Test VisionConfig creation and default values"""
        assert vision_config.input_format == ImageFormat.RGB
        assert vision_config.processing_mode == ProcessingMode.REAL_TIME
        assert vision_config.max_fps == 30
        assert vision_config.resolution == (640, 480)
        assert vision_config.confidence_threshold == 0.6
        assert vision_config.enable_gpu is True
    
    def test_vision_config_custom_values(self):
        """Test VisionConfig with custom values"""
        config = VisionConfig(
            capabilities=[VisionCapability.OBJECT_DETECTION],
            input_format=ImageFormat.BGR,
            processing_mode=ProcessingMode.HIGH_ACCURACY,
            max_fps=60,
            resolution=(1920, 1080),
            confidence_threshold=0.8,
            enable_gpu=False
        )
        
        assert config.input_format == ImageFormat.BGR
        assert config.processing_mode == ProcessingMode.HIGH_ACCURACY
        assert config.max_fps == 60
        assert config.resolution == (1920, 1080)
        assert config.confidence_threshold == 0.8
        assert config.enable_gpu is False


# Integration tests
class TestVisionIntegration:
    """Integration tests for vision system components"""
    
    def test_full_vision_pipeline(self, sample_image, vision_config):
        """Test complete vision processing pipeline"""
        vision_system = VisionSystem(vision_config)
        
        # Process image through complete pipeline
        results = vision_system.process_frame(sample_image)
        
        # Verify both detection and classification results
        assert VisionCapability.OBJECT_DETECTION in results
        assert VisionCapability.IMAGE_CLASSIFICATION in results
        
        # Verify detection results
        detections = results[VisionCapability.OBJECT_DETECTION]
        assert isinstance(detections, list)
        
        # Verify classification results
        classification = results[VisionCapability.IMAGE_CLASSIFICATION]
        assert isinstance(classification, ClassificationResult)
    
    def test_vision_system_error_handling(self):
        """Test vision system error handling"""
        config = VisionConfig(
            capabilities=[VisionCapability.OBJECT_DETECTION],
            input_format=ImageFormat.RGB,
            processing_mode=ProcessingMode.REAL_TIME
        )
        
        vision_system = VisionSystem(config)
        
        # Test with None image
        results = vision_system.process_frame(None)
        
        # Should handle gracefully and return empty/error results
        assert isinstance(results, dict)


if __name__ == "__main__":
    pytest.main([__file__, "-v"]) 