import time
import pandas as pd
import numpy as np

from src.hardware import RobotHardware
from src.logger_factory import get_logger
from src.schemas import (
    RobotState, Memory, SensorSnapshot, 
    EpisodeLog, EpisodeMetadata, TransitionRecord, Observation, StepTimestamps, ActionCommand, StepOutcome
)

DEBUG_STOP_ITERATION = 300
logger = get_logger(__name__, log_file=f"logs/{__name__}.log", level="INFO")

class ObjectHunterRobot:
    def __init__(self, hardware: RobotHardware) -> None:
        self.hw = hardware
        self.state = RobotState.SEARCH
        self.memory = Memory()
        self.episode_id = str(int(time.time()))
        episode_metadata = EpisodeMetadata(episode_id=self.episode_id, policy_type="handcrafted_fsm")
        self.episode_log = EpisodeLog(metadata=episode_metadata)

        # To adjust experimentally:
        self.obstacle_threshold_cm = 25.0 # If an obstacle is closer than this, we consider it "ahead" and need to avoid
        self.marker_center_tolerance = 0.1 # How far from the center the marker can be (as a fraction of frame width) before we consider it "off-center" and need to turn
        self.marker_close_area_threshold = 0.025 # How big the marker appears in the frame when we consider it "close enough" to stop at

        self.default_step_distance = 1.0 # meters
        self.small_step_distance = 0.5 # meters, used for more careful approach when the target is near
        self.default_turn_degree = 60
        self.small_turn_degree = 20

        self.search_step_duration = 7.5 # How long to keep moving forward in SEARCH before doing a SCAN
        self.lost_target_timeout = 7.0 # How long to stay in LOST_TARGET state before giving up and going back to SEARCH
        self.between_actions_delay = 1.5 # Small delay between actions to avoid overwhelming the hardware with commands

        self._running = False
        self._finished = False
        self._stopped = False

        # Stats
        self.iteration = 0
        self._execution_time = []

    def save_execution_time_stats(self, save_path: str) -> None:
        time_stats = pd.DataFrame(self._execution_time)
        time_stats.to_csv(save_path, index=False)

    def run(self) -> None:
        self._running = True
        logger.info("[ROBOT] Robot started")

        while self._running and not self._finished:
            if not self.run_iteration():
                break
            time.sleep(self.between_actions_delay)

        logger.info(f"[ROBOT] Robot run loop exited. Finished: {self._finished}, Iterations: {self.iteration}")

    def run_iteration(self) -> bool:
        if self._finished or self._stopped:
            return False

        self._running = True
        iteration_start_time = time.time()

        if DEBUG_STOP_ITERATION and self.iteration >= DEBUG_STOP_ITERATION:
            logger.info(f"[ROBOT] Reached debug stop iteration limit ({DEBUG_STOP_ITERATION}), stopping robot")
            self.stop()
            return False

        read_sensors_start_time = time.time()
        snapshot = self.hw.read_sensors()
        read_sensors_end_time = time.time()

        update_state_start_time = time.time()
        self._update_state(snapshot)
        update_state_end_time = time.time()

        self._log_next_state(snapshot)

        execute_state_start_time = time.time()
        self._execute_state(snapshot)
        execute_state_end_time = time.time()

        iteration_end_time = time.time()

        self._execution_time.append({
            "iteration": self.iteration,
            "read_sensors": read_sensors_end_time - read_sensors_start_time,
            "update_state": update_state_end_time - update_state_start_time,
            "execute_state": execute_state_end_time - execute_state_start_time,
            "total_iteration": iteration_end_time - iteration_start_time
        })
        logger.debug(f"[ROBOT] Iteration {self.iteration} completed in {iteration_end_time - iteration_start_time:.2f} seconds (read_sensors: {read_sensors_end_time - read_sensors_start_time:.2f}, update_state: {update_state_end_time - update_state_start_time:.2f}, execute_state: {execute_state_end_time - execute_state_start_time:.2f})")
        self.iteration += 1
        return not self._finished

    def stop(self) -> None:
        if self._stopped:
            return

        self._stopped = True
        self._running = False
        self.hw.stop()
        current_time = time.strftime("%Y-%m-%d_%H:%M:%S")   
        self.save_execution_time_stats(f"logs/time_stats/stats_{current_time}.csv")
        self.episode_log.save(f"logs/episode_logs/ep{self.episode_id}_{current_time}.json")
        logger.info("[ROBOT] Robot stopped")

    def _set_state(self, new_state: RobotState) -> None:
        if new_state != self.state:
            logger.debug("[ROBOT] State change: %s -> %s", self.state.value, new_state.value)
            self.state = new_state
            self.memory.state_enter_time = time.time()
            if self.state == RobotState.FINISHED:
                self._finished = True

    def _update_state(self, snapshot: SensorSnapshot) -> None:
        """
        Decides which state the robot should be in right now.
        This is the FSM transition logic.
        """
        marker = snapshot.marker
        obstacle_ahead = self._is_obstacle_ahead(snapshot)
        robot_stuck = self.is_robot_stuck()
        if robot_stuck:
            logger.warning("[ROBOT] Robot appears to be stuck based on recent sensor readings")

        if marker.visible:
            logger.debug("[ROBOT] Marker detected!")
            self.memory.remember_marker(marker)
            if marker.area >= self.marker_close_area_threshold:
                self._set_state(RobotState.FINISHED)
                logger.info(f"[ROBOT] Target reached with area {marker.area:.3f}, marking as FINISHED")
            else:
                self._set_state(RobotState.APPROACH)
            return

        if obstacle_ahead or not self.memory.avoid_completed or robot_stuck:
            self._set_state(RobotState.AVOID)
            return

        if self._recently_saw_marker():
            self._set_state(RobotState.LOST_TARGET)
            return

        # Simple logic: after moving forward, periodically do a scan
        state_time = time.time() - self.memory.state_enter_time

        if self.state == RobotState.SEARCH and state_time >= self.search_step_duration:
            self._set_state(RobotState.SCAN)
            return

        if self.state not in {RobotState.SEARCH, RobotState.SCAN}:
            self._set_state(RobotState.SEARCH)

    def _execute_state(self, snapshot: SensorSnapshot) -> None:
        if self.state == RobotState.SEARCH:
            self._handle_search(snapshot)
        elif self.state == RobotState.SCAN:
            self._handle_scan(snapshot)
        elif self.state == RobotState.APPROACH:
            self._handle_approach(snapshot)
        elif self.state == RobotState.AVOID:
            self._handle_avoid(snapshot)
        elif self.state == RobotState.LOST_TARGET:
            self._handle_lost_target(snapshot)
        elif self.state == RobotState.FINISHED:
            logger.info("[ROBOT] In FINISHED state, stopping robot")
            self.stop()
        else:
            logger.warning("[ROBOT] Unknown state: %s", self.state)

    def is_robot_stuck(self, n: int = 3, tolerance: float = 0.03) -> bool:
        """
        Robot is stuck when:
        - deviations from last n observation from dist and depth both are less than tolerance
        - Dist is not active and Depth deviation is less than tolerance and vice versa
        """
        last_n_dist = [rec.observation.obstacle_distance_cm for rec in self.episode_log.steps if rec.observation.obstacle_distance_cm is not None][-n:]
        last_n_depth = [rec.observation.depth_score for rec in self.episode_log.steps if rec.observation.depth_score is not None][-n:]
        dist_active = last_n_dist and len(last_n_dist) >= n and np.mean(last_n_dist) != 999.0
        depth_active = last_n_depth and len(last_n_depth) >= n
        relative_deviation = lambda arr: np.std(arr) / np.mean(arr)
        dist_deviation = relative_deviation(last_n_dist) if dist_active else np.inf
        depth_deviation = relative_deviation(last_n_depth) if depth_active else np.inf
        logger.debug(f"[ROBOT] Checking if stuck: dist_deviation={dist_deviation:.4f} (active: {dist_active}), depth_deviation={depth_deviation:.4f} (active: {depth_active}), tolerance={tolerance}")

        robot_stuck = ((dist_deviation < tolerance) and (depth_deviation < tolerance)) or\
                      ((depth_deviation < tolerance) and not dist_active) or\
                      ((dist_deviation < tolerance) and not depth_active)
        return robot_stuck

    def _create_log_record(self, snapshot: SensorSnapshot) -> None:
        prev_action = self.episode_log.steps[-1].action.name if len(self.episode_log.steps) > 0 else None
        prev_iter_time = round(self._execution_time[-1]['total_iteration'], 2) if len(self._execution_time) > 0 else None

        # Safe conversion for None values
        time_str = f"{prev_iter_time:.1f}" if prev_iter_time is not None else "N/A"
        action_str = prev_action if prev_action is not None else "N/A"
        obs_str = f"{snapshot.obstacle_distance_cm:.0f}" if snapshot.obstacle_distance_cm is not None else "N/A"
        depth_str = f"{snapshot.depth_hazard.depth_score:.3f}" if snapshot.depth_hazard.depth_score is not None else "N/A"

        # logger.info(f"[ROBOT] ep = {self.episode_id} | iter = {self.iteration} | prev_iter_time = {prev_iter_time} | state = {self.state.value} | prev_action = {prev_action} | marker_area = {snapshot.marker.area} | depth_scores = (left: {snapshot.depth_hazard.left_score:.3f}, center: {snapshot.depth_hazard.center_score:.3f}, right: {snapshot.depth_hazard.right_score:.3f}) | obstacle_distance_cm = {snapshot.obstacle_distance_cm}")
        logger.info("[ROBOT] ep=%s | iter=%d | time=%s | state=%s | action=%s | marker=%.3f | depth=%s | obs=%scm",
            self.episode_id, self.iteration, time_str, 
            self.state.value, action_str, snapshot.marker.area,
            depth_str, obs_str)
        
        self.episode_log.steps.append(TransitionRecord(
            episode_id=self.episode_id,
            iteration=self.iteration,
            state=self.state.value,
            observation=Observation.from_sensor_snapshot(snapshot),
            next_observation=None,
            action=None,
            outcome=None,
            timestamps=StepTimestamps(
                decision_start_ts=time.time(),
                action_start_ts=None,
                action_end_ts=None,
                decision_end_ts=None
            ),
        ))

    def _log_action(self, action_command: ActionCommand) -> None:
        self.episode_log.steps[-1].action = action_command
        self.episode_log.steps[-1].timestamps.action_start_ts = self.hw.last_action_start_ts
        self.episode_log.steps[-1].timestamps.action_end_ts = self.hw.last_action_end_ts
        self.episode_log.steps[-1].timestamps.decision_end_ts = time.time()
    
    def _log_next_state(self, snapshot: SensorSnapshot) -> None:
        if len(self.episode_log.steps) > 0:
            self.episode_log.steps[-1].next_observation = Observation.from_sensor_snapshot(snapshot)
            self.episode_log.steps[-1].outcome = StepOutcome(terminal=self._finished, next_state=self.state.value)
            if self._finished:
                self.episode_log.metadata.finished_at = time.time()

    def _handle_search(self, snapshot: SensorSnapshot) -> None:
        """
        Basic exploration mode:
        - short safe step forward (avoid obstacles)
        - then transition to scan
        """
        self._create_log_record(snapshot)

        # 1. Check for obstacles and adjust step distance if needed
        distance_to_obstacle = snapshot.obstacle_distance_cm
        distance_to_pass = self.default_step_distance
        if snapshot.depth_hazard.blocked:
            logger.debug(
                "[ROBOT] Depth obstacle ahead (score: %.3f), not moving forward",
                snapshot.depth_hazard.depth_score
            )
            return

        if distance_to_obstacle is not None:
            safe_distance_to_pass = distance_to_obstacle - self.obstacle_threshold_cm
            if safe_distance_to_pass <= 0:
                logger.debug("[ROBOT] Obstacle too close at %.1f cm, not moving forward", distance_to_obstacle)
                return
            distance_to_pass = min(self.default_step_distance, safe_distance_to_pass)

        # 2. Move forward
        self.hw.move_forward(distance=distance_to_pass)

        self.memory.repeated_forward_steps += 1
        self._log_action(action_command=ActionCommand(name="move_forward", parameters={"distance": distance_to_pass}))

    def _handle_scan(self, snapshot: SensorSnapshot) -> None:
        """
        Exploration of the surroundings.
        Idea: stop and look around in different directions.
        For now, just a rough strategy:
        - stop
        - turn left/right/center in sequence
        """
        self._create_log_record(snapshot)

        # Scan logic
        # 0 -> left, 1 -> right, 2 -> center/finish
        if self.memory.last_scan_direction == 0:
            self.hw.turn_left(degrees=self.default_turn_degree)
            self.memory.last_scan_direction = 1
            self._log_action(action_command=ActionCommand(name="turn_left", parameters={"degrees": self.default_turn_degree}))
            return

        if self.memory.last_scan_direction == 1:
            self.hw.turn_right(degrees=self.default_turn_degree * 1.5) # 1.5 instead of 2 to adjust for overshoot
            self.memory.last_scan_direction = 2
            self._log_action(action_command=ActionCommand(name="turn_right", parameters={"degrees": self.default_turn_degree * 1.5}))
            return

        # Finish scan, return to original orientation if needed, and go back to SEARCH
        self.hw.turn_left(degrees=self.default_turn_degree)
        self.memory.last_scan_direction = 0
        self._set_state(RobotState.SEARCH)
        self._log_action(action_command=ActionCommand(name="turn_left", parameters={"degrees": self.default_turn_degree}))

    def _handle_approach(self, snapshot: SensorSnapshot) -> None:
        """
        Marker is visible, we want to approach it:
        - center it in the frame by turning
        - move forward
        - if it's very close, we can stop
        """
        self._create_log_record(snapshot)
        marker = snapshot.marker

        if not marker.visible:
            self._set_state(RobotState.LOST_TARGET)
            return

        x_offset = marker.x_offset

        # Faster approach when the marker is far, slower when it's closer for better precision
        if marker.area <= self.marker_close_area_threshold / 4:
            self.hw.move_forward(distance=self.default_step_distance)
            self._log_action(action_command=ActionCommand(name="move_forward", parameters={"distance": self.default_step_distance}))
            return


        if abs(x_offset) > self.marker_center_tolerance:
            if x_offset < 0:
                self.hw.turn_left(degrees=self.small_turn_degree)
                self._log_action(action_command=ActionCommand(name="turn_left", parameters={"degrees": self.small_turn_degree}))
            else:
                self.hw.turn_right(degrees=self.small_turn_degree)
                self._log_action(action_command=ActionCommand(name="turn_right", parameters={"degrees": self.small_turn_degree}))
            return

        if marker.area >= self.marker_close_area_threshold:
            logger.info(f"[ROBOT] Target reached (distance = {snapshot.obstacle_distance_cm}, area = {marker.area})")
            # after reaching the target, we could:
            # - make a sound or light signal
            # - consider the task done and stop the robot
            # - start looking for the next marker
            return

        self.hw.move_forward(distance=self.small_step_distance)
        self._log_action(action_command=ActionCommand(name="move_forward", parameters={"distance": self.small_step_distance}))

    def _handle_avoid(self, snapshot: SensorSnapshot) -> None:
        """
        Avoiding obstacle:
        For now, a simple strategy:
        - stop
        - move back a bit
        - turn
        - back to SEARCH
        """
        self._create_log_record(snapshot)

        if self._is_obstacle_ahead(snapshot):
            self.hw.move_backward(distance=self.default_step_distance)
            self.memory.avoid_completed = False
            self._log_action(action_command=ActionCommand(name="move_backward", parameters={"distance": self.default_step_distance}))
            return

        turn_direction = "left" if self.memory.repeated_turns % 2 == 0 else "right"

        if turn_direction == "left":
            self.hw.turn_left(degrees=self.default_turn_degree)
            self._log_action(action_command=ActionCommand(name="turn_left", parameters={"degrees": self.default_turn_degree}))
        else:
            self.hw.turn_right(degrees=self.default_turn_degree)
            self._log_action(action_command=ActionCommand(name="turn_right", parameters={"degrees": self.default_turn_degree}))

        self.memory.repeated_turns += 1
        self.memory.avoid_completed = True
        self._set_state(RobotState.SEARCH)

    def _handle_lost_target(self, snapshot: SensorSnapshot) -> None:
        """
        Marker was just seen but now it's lost.
        So it makes sense to look for it around the last known direction,
        rather than immediately going into full exploration.
        """
        self._create_log_record(snapshot)

        if snapshot.marker.visible:
            self.memory.remember_marker(snapshot.marker)
            self._set_state(RobotState.APPROACH)
            return

        last_x = self.memory.last_seen_marker_x
        if last_x is None:
            self._set_state(RobotState.SEARCH)
            return
        
        # Small local search towards the last known direction of the target
        if last_x < 0:
            self.hw.turn_left(degrees=self.small_turn_degree)
            self._log_action(action_command=ActionCommand(name="turn_left", parameters={"degrees": self.small_turn_degree}))
        else:
            self.hw.turn_right(degrees=self.small_turn_degree)
            self._log_action(action_command=ActionCommand(name="turn_right", parameters={"degrees": self.small_turn_degree}))

        # If we can't re-detect the target for too long, we go back to SEARCH
        if (time.time() - self.memory.state_enter_time) > self.lost_target_timeout:
            self._set_state(RobotState.SEARCH)

    def _is_obstacle_ahead(self, snapshot: SensorSnapshot) -> bool:
        distance = snapshot.obstacle_distance_cm
        distance_blocked = distance is not None and distance < self.obstacle_threshold_cm
        return distance_blocked or snapshot.depth_hazard.blocked

    def _recently_saw_marker(self) -> bool:
        if self.memory.last_seen_marker_time is None:
            return False
        return (time.time() - self.memory.last_seen_marker_time) < self.lost_target_timeout
