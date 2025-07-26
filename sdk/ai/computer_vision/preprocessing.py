"""
Constellation Overwatch SDK - Image Preprocessing Module

<!-- DEVTEAM: Image preprocessing utilities for computer vision pipeline -->
"""

import numpy as np
import logging

logger = logging.getLogger(__name__)


class ImagePreprocessor:
    """Image preprocessing utilities for vision pipeline"""
    
    def __init__(self, config=None):
        self.config = config
        logger.info("Image preprocessor initialized")
    
    def normalize(self, image: np.ndarray) -> np.ndarray:
        """Normalize image to 0-1 range"""
        return image.astype(np.float32) / 255.0
    
    def resize(self, image: np.ndarray, target_size: tuple) -> np.ndarray:
        """Resize image to target dimensions"""
        # Mock implementation - would use actual image resizing
        return image 