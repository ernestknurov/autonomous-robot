from __future__ import annotations
import time

from src.schemas import (RobotState, Memory, SensorSnapshot)
from src.hardware import RobotHardware
from src.logger_factory import get_logger

logger = get_logger(__name__, log_file=f"logs/{__name__}.log")
DEBUG_STOP_ITERATION = 200

class ObjectHunterRobot:
    def __init__(self, hardware: RobotHardware) -> None:
        self.hw = hardware
        self.state = RobotState.SEARCH
        self.memory = Memory()

        # To adjust experimentally:
        self.obstacle_threshold_cm = 25.0
        self.marker_center_tolerance = 0.1
        self.marker_close_area_threshold = 0.025

        self.default_step_distance = 1.0
        self.small_step_distance = 0.5
        self.default_turn_degree = 40
        self.small_turn_degree = 20

        self.search_step_duration = 5.0
        self.scan_turn_duration = 0.5
        self.avoid_back_duration = 0.4
        self.avoid_turn_duration = 0.7
        self.lost_target_timeout = 7.0
        self.between_actions_delay = 0.5

        self._running = False
        self._finished = False
        self._iteration_count = 0

    def run(self) -> None:
        self._running = True
        logger.info("[ROBOT] Robot started")

        while self._running and not self._finished:
            if DEBUG_STOP_ITERATION and self._iteration_count >= DEBUG_STOP_ITERATION:
                logger.info("[ROBOT] Reached debug stop iteration limit (%d), stopping robot", DEBUG_STOP_ITERATION)
                self.stop()
                break
            snapshot = self.hw.read_sensors()
            self._update_state(snapshot)
            self._execute_state(snapshot)
            self._iteration_count += 1
            time.sleep(self.between_actions_delay)
        logger.info("[ROBOT] Robot run loop exited. Finished: %s, Iterations: %d", self._finished, self._iteration_count)

    def stop(self) -> None:
        self._running = False
        self.hw.stop()
        logger.info("[ROBOT] Robot stopped")

    def _set_state(self, new_state: RobotState) -> None:
        if new_state != self.state:
            logger.info("[ROBOT] State change: %s -> %s", self.state.value, new_state.value)
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

        if marker.visible:
            logger.debug("[ROBOT] Marker detected!")
            self.memory.remember_marker(marker)
            if marker.area >= self.marker_close_area_threshold:
                self._set_state(RobotState.FINISHED)
                logger.info("[ROBOT] Target reached with area %.3f, marking as FINISHED", marker.area)
            else:
                self._set_state(RobotState.APPROACH)
            return

        if obstacle_ahead:
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

    def _handle_search(self, snapshot: SensorSnapshot) -> None:
        """
        Basic exploration mode:
        - short safe step forward (avoid obstacles)
        - then transition to scan
        """
        logger.debug("[ROBOT] Current state: SEARCH")
        distance_to_obstacle = snapshot.obstacle_distance_cm
        distance_to_pass = self.default_step_distance
        if distance_to_obstacle is not None:
            safe_distance_to_pass = distance_to_obstacle - self.obstacle_threshold_cm
            if safe_distance_to_pass <= 0:
                logger.debug("[SEARCH] Obstacle too close at %.1f cm, not moving forward", distance_to_obstacle)
                return
            distance_to_pass = min(self.default_step_distance, safe_distance_to_pass)
        self.hw.move_forward(distance=distance_to_pass)
        self.memory.repeated_forward_steps += 1

    def _handle_scan(self, snapshot: SensorSnapshot) -> None:
        """
        Exploration of the surroundings.
        Idea: stop and look around in different directions.
        For now, just a rough strategy:
        - stop
        - turn left/right/center in sequence
        """
        logger.debug("SCAN")
        self.hw.stop()

        # Scan logic
        # 0 -> left, 1 -> right, 2 -> center/finish
        if self.memory.last_scan_direction == 0:
            self.hw.turn_left(degrees=self.default_turn_degree)
            time.sleep(self.scan_turn_duration)
            self.hw.stop()
            self.memory.last_scan_direction = 1
            return

        if self.memory.last_scan_direction == 1:
            self.hw.turn_right(degrees=self.default_turn_degree * 1.5) # 1.5 instead of 2 to adjust for overshoot
            time.sleep(self.scan_turn_duration)
            self.hw.stop()
            self.memory.last_scan_direction = 2
            return

        # Finish scan, return to original orientation if needed, and go back to SEARCH
        self.hw.turn_left(degrees=self.default_turn_degree)
        time.sleep(self.scan_turn_duration)
        self.hw.stop()
        self.memory.last_scan_direction = 0
        self._set_state(RobotState.SEARCH)

    def _handle_approach(self, snapshot: SensorSnapshot) -> None:
        """
        Marker is visible, we want to approach it:
        - center it in the frame by turning
        - move forward
        - if it's very close, we can stop
        """
        logger.debug("[ROBOT] APPROACH")
        marker = snapshot.marker

        if not marker.visible:
            self._set_state(RobotState.LOST_TARGET)
            return

        x_offset = marker.x_offset

        if marker.area <= self.marker_close_area_threshold / 4:
            self.hw.move_forward(distance=self.default_step_distance)
            return


        if abs(x_offset) > self.marker_center_tolerance:
            if x_offset < 0:
                # image is upside down, so negative x means target is on the right
                self.hw.turn_right(degrees=self.small_turn_degree)
            else:
                self.hw.turn_left(degrees=self.small_turn_degree)
            return

        if marker.area >= self.marker_close_area_threshold:
            logger.info("[ROBOT] Target reached")
            # self.hw.stop()
            # after reaching the target, we could:
            # - make a sound or light signal
            # - consider the task done and stop the robot
            # - start looking for the next marker
            return

        self.hw.move_forward(distance=self.small_step_distance)

    def _handle_avoid(self, snapshot: SensorSnapshot) -> None:
        """
        Avoiding obstacle:
        For now, a simple strategy:
        - stop
        - move back a bit
        - turn
        - back to SEARCH
        """
        logger.debug("[ROBOT] State: AVOID")
        self.hw.stop()

        self.hw.move_backward(distance=self.default_step_distance)
        time.sleep(self.avoid_back_duration)
        self.hw.stop()

        # For now, just alternate left and right turns to try to find a clear path
        if self.memory.repeated_turns % 2 == 0:
            self.hw.turn_left(degrees=self.default_turn_degree)
        else:
            self.hw.turn_right(degrees=self.default_turn_degree)

        time.sleep(self.avoid_turn_duration)
        self.hw.stop()

        self.memory.repeated_turns += 1
        self._set_state(RobotState.SEARCH)

    def _handle_lost_target(self, snapshot: SensorSnapshot) -> None:
        """
        Marker was just seen but now it's lost.
        So it makes sense to look for it around the last known direction,
        rather than immediately going into full exploration.
        """
        logger.debug("LOST_TARGET")

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
            # image is upside down, so negative x means target was on the right
            self.hw.turn_right(degrees=self.small_turn_degree)
        else:
            self.hw.turn_left(degrees=self.small_turn_degree)

        # If we can't re-detect the target for too long, we go back to SEARCH
        if (time.time() - self.memory.state_enter_time) > self.lost_target_timeout:
            self._set_state(RobotState.SEARCH)

    def _is_obstacle_ahead(self, snapshot: SensorSnapshot) -> bool:
        distance = snapshot.obstacle_distance_cm
        if distance is None:
            return False
        return distance < self.obstacle_threshold_cm

    def _recently_saw_marker(self) -> bool:
        if self.memory.last_seen_marker_time is None:
            return False
        return (time.time() - self.memory.last_seen_marker_time) < self.lost_target_timeout