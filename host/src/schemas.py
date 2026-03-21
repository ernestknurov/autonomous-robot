from dataclasses import dataclass, field, asdict
from typing import Optional
import enum
import time
import numpy as np
import json

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
    # frame: Optional[np.ndarray] = None  # BGR image from camera
    timestamp: float = field(default_factory=time.time)


@dataclass
class SubActionItem:
    name: str
    parameters: dict = field(default_factory=dict)
    start_timestamp: float = field(default_factory=time.time)
    end_timestamp: float = field(default_factory=time.time)
    duration: float = 0.0

    def log_time(self) -> None:
        self.end_timestamp = time.time()
        self.duration = self.end_timestamp - self.start_timestamp


@dataclass
class ActionHistoryItem:
    iteration: int
    state: RobotState
    sensor_snapshot: Optional[SensorSnapshot] = None
    sub_actions: list[SubActionItem] = field(default_factory=list)
    start_timestamp: float = field(default_factory=time.time)
    end_timestamp: float = field(default_factory=time.time)
    duration: float = 0.0

    def log_time(self) -> None:
        self.end_timestamp = time.time()
        self.duration = self.end_timestamp - self.start_timestamp
    


@dataclass
class Memory:
    last_seen_marker_time: Optional[float] = None
    last_seen_marker_x: Optional[float] = None
    last_seen_marker_area: Optional[float] = None

    last_scan_direction: int = 0
    repeated_turns: int = 0
    repeated_forward_steps: int = 0

    action_history: list[ActionHistoryItem] = field(default_factory=list)

    state_enter_time: float = field(default_factory=time.time)

    def remember_marker(self, marker: MarkerDetection) -> None:
        self.last_seen_marker_time = time.time()
        self.last_seen_marker_x = marker.x_offset
        self.last_seen_marker_area = marker.area

    def reset_repetition_counters(self) -> None:
        self.repeated_turns = 0
        self.repeated_forward_steps = 0

    def save_action_history(self, save_path: str) -> None:
        
        def custom_encoder(obj):
            if isinstance(obj, enum.Enum):
                return obj.value  # Convert enum to its string value
            if isinstance(obj, np.ndarray):
                return obj.tolist()  # Convert numpy array to list
            if isinstance(obj, np.generic):
                return obj.item()  # Convert numpy scalar types (float32, int32, etc.) to Python native
            if hasattr(obj, '__dataclass_fields__'):
                return asdict(obj)  # Convert dataclass to dict
            return obj.__dict__
        
        with open(save_path, "w") as f:
            json.dump(self.action_history, f, default=custom_encoder, indent=2)