import argparse
import json
from pathlib import Path

import pandas as pd

filename = "ep1776500276_2026-04-18_10:20:22"
LOG_DIR = Path(__file__).parent.parent / "logs" / "episode_logs"


def add_observation(action, observation, prefix):
    action[f"{prefix}_obstacle_distance"] = observation.get("obstacle_distance_cm")
    action[f"{prefix}_marker_visible"] = observation.get("marker_visible")
    action[f"{prefix}_marker_x_offset"] = observation.get("marker_x_offset")
    action[f"{prefix}_marker_area"] = observation.get("marker_area")
    action[f"{prefix}_marker_id"] = observation.get("marker_id")
    action[f"{prefix}_depth_blocked"] = observation.get("depth_blocked")
    action[f"{prefix}_depth_score"] = observation.get("depth_score")


def add_empty_observation(action, prefix):
    for field in (
        "obstacle_distance",
        "marker_visible",
        "marker_x_offset",
        "marker_area",
        "marker_id",
        "depth_blocked",
        "depth_score",
    ):
        action[f"{prefix}_{field}"] = None


def process_step(record):
    action = {}
    action["episode_id"] = record["episode_id"]
    action["iteration"] = record["iteration"]
    action["state"] = record["state"]
    action_record = record.get("action") or {}
    action_parameters = action_record.get("parameters") or {}
    action["action"] = action_record.get("name")
    action["action_param_distance"] = action_parameters.get("distance")
    action["action_param_degrees"] = action_parameters.get("degrees")

    add_observation(action, record.get("observation") or {}, "obs")

    next_obs = record.get("next_observation")
    if next_obs:
        add_observation(action, next_obs, "next_obs")
    else:
        add_empty_observation(action, "next_obs")

    try:
        action["duration"] = (
            record["timestamps"]["action_end_ts"] - record["timestamps"]["action_start_ts"]
        )
    except (KeyError, TypeError):
        action["duration"] = None

    outcome = record.get("outcome") or {}
    action["nest_state"] = outcome.get("next_state")
    action["terminal"] = outcome.get("terminal")

    return action


def resolve_load_path(log_name_or_path):
    log_path = Path(log_name_or_path)
    if log_path.suffix != ".json":
        log_path = log_path.with_suffix(".json")
    if not log_path.is_absolute() and not log_path.exists():
        log_path = LOG_DIR / log_path
    return log_path


def preprocess_log(log_name_or_path):
    load_path = resolve_load_path(log_name_or_path)
    save_path = LOG_DIR / "processed" / f"{load_path.stem}.csv"

    with open(load_path, "r") as f:
        logs = json.load(f)

    steps = logs.get("steps", [])
    all_actions = [process_step(step) for step in steps]

    data = pd.DataFrame(all_actions)
    save_path.parent.mkdir(parents=True, exist_ok=True)
    data.to_csv(save_path, index=False)
    return save_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert episode JSON logs into CSV.")
    parser.add_argument(
        "log",
        nargs="?",
        default=filename,
        help="Episode log filename, stem, or path. Defaults to the latest local debug log.",
    )
    args = parser.parse_args()

    output_path = preprocess_log(args.log)
    print(f"Saved processed log to {output_path}")
