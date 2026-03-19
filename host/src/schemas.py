from dataclasses import dataclass, field
from typing import Optional
import enum
import time
import numpy as np

class RobotState(enum.Enum):
    SEARCH = "search"
    SCAN = "scan"
    APPROACH = "approach"
    AVOID = "avoid"
    LOST_TARGET = "lost_target"
    FINISHED = "finished"


@dataclass
class MarkerDetection:
    visible: bool
    x_offset: float = 0.0
    area: float = 0.0
    corners: Optional[np.ndarray] = None  # shape (4, 2) if detected
    marker_id: Optional[int] = None


@dataclass
class SensorSnapshot:
    obstacle_distance_cm: Optional[float]
    marker: MarkerDetection
    timestamp: float = field(default_factory=time.time)


@dataclass
class Memory:
    last_seen_marker_time: Optional[float] = None
    last_seen_marker_x: Optional[float] = None
    last_seen_marker_area: Optional[float] = None

    last_scan_direction: int = 0
    repeated_turns: int = 0
    repeated_forward_steps: int = 0

    state_enter_time: float = field(default_factory=time.time)

    def remember_marker(self, marker: MarkerDetection) -> None:
        self.last_seen_marker_time = time.time()
        self.last_seen_marker_x = marker.x_offset
        self.last_seen_marker_area = marker.area

    def reset_repetition_counters(self) -> None:
        self.repeated_turns = 0
        self.repeated_forward_steps = 0