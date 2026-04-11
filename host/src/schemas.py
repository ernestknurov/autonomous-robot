from dataclasses import dataclass, field, asdict
from typing import Any, Optional
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
class DepthHazard:
    blocked: bool = False
    depth_score: float = 0.0


@dataclass
class SensorSnapshot:
    obstacle_distance_cm: Optional[float]
    marker: MarkerDetection
    depth_hazard: DepthHazard = field(default_factory=DepthHazard)
    timestamp: float = field(default_factory=time.time)


@dataclass
class Observation:
    obstacle_distance_cm: Optional[float]
    marker_visible: bool
    marker_x_offset: Optional[float]
    marker_area: Optional[float]
    marker_id: Optional[int]
    depth_blocked: bool
    depth_score: Optional[float]

    @classmethod
    def from_sensor_snapshot(cls, snapshot: SensorSnapshot) -> "Observation":
        return cls(
            obstacle_distance_cm=snapshot.obstacle_distance_cm,
            marker_visible=snapshot.marker.visible,
            marker_x_offset=snapshot.marker.x_offset,
            marker_area=snapshot.marker.area,
            marker_id=snapshot.marker.marker_id,
            depth_blocked=snapshot.depth_hazard.blocked,
            depth_score=snapshot.depth_hazard.depth_score
        )

@dataclass
class ActionCommand:
    name: str
    parameters: dict[str, Any]


@dataclass
class StepTimestamps:
    decision_start_ts: float
    action_start_ts: Optional[float]
    action_end_ts: Optional[float]
    decision_end_ts: Optional[float]


@dataclass
class StepOutcome:
    terminal: bool
    next_state: Optional[str] = None


@dataclass
class TransitionRecord:
    episode_id: str
    iteration: int
    state: str
    observation: Observation
    action: Optional[ActionCommand]
    next_observation: Optional[Observation]
    timestamps: StepTimestamps
    outcome: Optional[StepOutcome]

@dataclass
class EpisodeMetadata:
    episode_id: str
    policy_type: str
    started_at: Optional[float] = field(default_factory=time.time)
    finished_at: Optional[float] = None

@dataclass
class EpisodeLog:
    metadata: EpisodeMetadata
    steps: list[TransitionRecord] = field(default_factory=list)

    def save(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=2)


@dataclass
class Memory:
    last_seen_marker_time: Optional[float] = None
    last_seen_marker_x: Optional[float] = None
    last_seen_marker_area: Optional[float] = None

    last_scan_direction: int = 0
    repeated_turns: int = 0
    repeated_forward_steps: int = 0
    avoid_completed: bool = True

    state_enter_time: float = field(default_factory=time.time)

    def remember_marker(self, marker: MarkerDetection) -> None:
        self.last_seen_marker_time = time.time()
        self.last_seen_marker_x = marker.x_offset
        self.last_seen_marker_area = marker.area

    def reset_repetition_counters(self) -> None:
        self.repeated_turns = 0
        self.repeated_forward_steps = 0
