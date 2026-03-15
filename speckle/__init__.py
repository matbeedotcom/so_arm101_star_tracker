"""
Speckle interferometry library for multi-camera burst capture and processing.

Usage:
    from speckle import SpecklePipeline, CaptureConfig, StabilityConfig
    pipeline = SpecklePipeline(cam, CaptureConfig(burst_count=100))
    results = pipeline.run(imu_bus, (ph, pkt), context={'target_name': 'polaris'})
"""

import sys

# Make telescope project importable for reconstruction wrappers
_TELESCOPE_SRC = '/home/acidhax/dev/telescope/src'
if _TELESCOPE_SRC not in sys.path:
    sys.path.insert(0, _TELESCOPE_SRC)

from .config import CaptureConfig, StabilityConfig, ProcessingConfig
from .capture import SpeckleCapture, BurstResult
from .storage import save_burst, load_burst, list_sessions
from .processing import SpeckleProcessor
from .reconstruction import (
    reconstruct,
    get_clean_reconstructor,
    get_deconvolution,
    get_super_resolution,
    get_interferometry_processor,
)
from .pipeline import SpecklePipeline

__all__ = [
    'CaptureConfig', 'StabilityConfig', 'ProcessingConfig',
    'SpeckleCapture', 'BurstResult',
    'save_burst', 'load_burst', 'list_sessions',
    'SpeckleProcessor',
    'reconstruct', 'get_clean_reconstructor', 'get_deconvolution',
    'get_super_resolution', 'get_interferometry_processor',
    'SpecklePipeline',
]
