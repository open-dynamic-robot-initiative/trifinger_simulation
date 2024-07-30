__version__ = "2.0.0"

import pathlib

# import some important classes to the main module
from .action import Action  # noqa
from .observation import Observation  # noqa
from .sim_finger import SimFinger  # noqa
from .trifinger_platform import (  # noqa
    TriFingerPlatform,
    ObjectPose,
    CameraObservation,
    TriCameraObservation,
    TriCameraObjectObservation,
)


def get_data_dir() -> pathlib.Path:
    """Get path to the data directory of this package."""
    p = pathlib.Path(__file__)
    return p.parent / "data"
