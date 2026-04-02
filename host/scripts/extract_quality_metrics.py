"""Extract quality metrics from robot episode logs."""

import argparse
from pathlib import Path
from typing import Dict

import pandas as pd


OBSTACLE_DETECTION_THRESHOLD_CM = 100.0


def _normalize_dataframe(df: pd.DataFrame) -> pd.DataFrame:
    """Normalize legacy and preprocessed CSV schemas into one shape."""
    df = df.copy()

    rename_map = {
        "action": "action_name",
        "obs_obstacle_distance": "obstacle_distance",
        "obs_marker_visible": "marker_visible",
        "obs_marker_x_offset": "marker_x_offset",
        "obs_marker_area": "marker_area",
        "action_param_distance": "parameters_distance",
        "action_param_degrees": "parameters_degrees",
        "nest_state": "next_state",
    }
    df = df.rename(columns=rename_map)

    if "duration" not in df.columns and {
        "action_start_ts",
        "action_end_ts",
    }.issubset(df.columns):
        df["duration"] = df["action_end_ts"] - df["action_start_ts"]

    if "action_name" not in df.columns:
        df["action_name"] = "unknown"

    if "duration" not in df.columns:
        df["duration"] = 0.0

    if "marker_visible" not in df.columns:
        if "next_obs_marker_visible" in df.columns:
            df["marker_visible"] = df["next_obs_marker_visible"]
        else:
            df["marker_visible"] = False

    for column, fallback in (
        ("obstacle_distance", pd.NA),
        ("marker_x_offset", 0.0),
        ("marker_area", 0.0),
    ):
        if column not in df.columns:
            next_column = f"next_obs_{column}"
            if next_column in df.columns:
                df[column] = df[next_column]
            else:
                df[column] = fallback

    if "state" not in df.columns:
        df["state"] = "unknown"

    df["duration"] = pd.to_numeric(df["duration"], errors="coerce").fillna(0.0)
    df["marker_visible"] = (
        df["marker_visible"].astype(str).str.lower().map({"true": True, "false": False})
    ).fillna(df["marker_visible"]).astype(bool)
    df["obstacle_distance"] = pd.to_numeric(df["obstacle_distance"], errors="coerce")
    df["marker_x_offset"] = pd.to_numeric(df["marker_x_offset"], errors="coerce")
    df["marker_area"] = pd.to_numeric(df["marker_area"], errors="coerce")

    return df


def extract_quality_metrics(csv_path: str) -> Dict:
    """Extract meaningful quality indicators from episode history."""
    df = pd.read_csv(csv_path)
    df = _normalize_dataframe(df)
    metrics = {}

    detected_frames = df[df["marker_visible"]]
    first_detection_idx = detected_frames.index[0] if not detected_frames.empty else len(df)

    # === SEARCH / TARGET ACQUISITION ===
    acquisition_states = {"search", "scan", "approach"}
    acquisition_actions = df[df["state"].isin(acquisition_states)]
    acquisition_before_detection = acquisition_actions[
        acquisition_actions.index < first_detection_idx
    ]

    metrics["search_efficiency"] = {
        "actions_before_detection": first_detection_idx,
        "movement_actions_before_detection": len(acquisition_before_detection),
        "total_commanded_distance": acquisition_before_detection[
            "parameters_distance"
        ].fillna(0.0).sum()
        if "parameters_distance" in acquisition_before_detection.columns
        else 0.0,
        "total_search_time": acquisition_before_detection["duration"].sum(),
    }

    # === OBSTACLE AVOIDANCE ====
    obstacle_events = df[
        df["obstacle_distance"].notna()
        & (df["obstacle_distance"] < OBSTACLE_DETECTION_THRESHOLD_CM)
    ]
    avoid_actions = df[df["state"] == "avoid"]
    obstacle_event_count = len(obstacle_events)

    metrics["obstacle_handling"] = {
        "obstacles_encountered": obstacle_event_count,
        "avoidance_actions": len(avoid_actions),
        "closest_approach": obstacle_events["obstacle_distance"].min()
        if obstacle_event_count
        else None,
        "obstacle_detection_rate": (obstacle_event_count / len(df)) * 100 if len(df) else 0.0,
    }

    # === VISION QUALITY ===
    if not detected_frames.empty:
        first_detection = detected_frames.iloc[0]
        metrics["vision_quality"] = {
            "detections": len(detected_frames),
            "first_detection_marker_area": first_detection["marker_area"],
            "best_detection_marker_area": detected_frames["marker_area"].max(),
            "marker_area_improvement": (
                detected_frames["marker_area"].max() - first_detection["marker_area"]
            )
            if len(detected_frames) > 1
            else 0.0,
            "avg_x_offset_error": detected_frames["marker_x_offset"].abs().mean(),
            "x_offset_stability": detected_frames["marker_x_offset"].std(),
        }

    # === ACTION PERFORMANCE ===
    action_perf = {}
    for action in df["action_name"].dropna().unique():
        action_df = df[df["action_name"] == action]
        action_perf[action] = {
            "count": len(action_df),
            "avg_duration": action_df["duration"].mean(),
            "duration_consistency": action_df["duration"].std(),
        }
    metrics["action_performance"] = action_perf

    # === STATE MACHINE EFFICIENCY ===
    state_sequence = df["state"].fillna("unknown").tolist()
    state_changes = sum(
        1 for i in range(len(state_sequence) - 1) if state_sequence[i] != state_sequence[i + 1]
    )

    metrics["state_machine"] = {
        "total_state_changes": state_changes,
        "states_visited": df["state"].nunique(),
        "state_distribution": df["state"].value_counts().to_dict(),
    }

    # === OVERALL EPISODE QUALITY ===
    total_time = df["duration"].sum()
    metrics["episode_summary"] = {
        "total_duration": total_time,
        "total_actions": len(df),
        "success": "marker_detected" if not detected_frames.empty else "search_incomplete",
        "efficiency_score": (first_detection_idx / len(df)) * 100
        if first_detection_idx < len(df) and len(df)
        else 100,
    }

    return metrics


def print_metrics(metrics: Dict):
    """Pretty print quality metrics."""
    print("\n" + "=" * 60)
    print("ROBOT EPISODE QUALITY METRICS")
    print("=" * 60)

    ep = metrics["episode_summary"]
    print(f"\nEPISODE SUMMARY")
    print(f"  Duration: {ep['total_duration']:.2f}s")
    print(f"  Total Actions: {ep['total_actions']}")
    print(f"  Result: {ep['success']}")
    print(f"  Efficiency: {ep['efficiency_score']:.1f}% (lower = found sooner)")

    se = metrics["search_efficiency"]
    print(f"\nSEARCH EFFICIENCY")
    print(f"  Actions before detection: {se['actions_before_detection']}")
    print(f"  Movement/search actions: {se['movement_actions_before_detection']}")
    print(f"  Commanded distance: {se['total_commanded_distance']:.2f}")
    print(f"  Search time: {se['total_search_time']:.2f}s")

    oh = metrics["obstacle_handling"]
    print(f"\nOBSTACLE HANDLING")
    print(f"  Obstacles encountered: {oh['obstacles_encountered']}")
    print(f"  Avoidance actions triggered: {oh['avoidance_actions']}")
    closest_approach = oh["closest_approach"]
    closest_approach_display = (
        f"{closest_approach:.2f}cm" if closest_approach is not None else "N/A"
    )
    print(f"  Closest approach: {closest_approach_display}")
    print(f"  Detection rate: {oh['obstacle_detection_rate']:.1f}%")

    if "vision_quality" in metrics:
        vq = metrics["vision_quality"]
        print(f"\nVISION QUALITY")
        print(f"  Detections: {vq['detections']}")
        print(f"  Initial marker area: {vq['first_detection_marker_area']:.6f}")
        print(f"  Best marker area: {vq['best_detection_marker_area']:.6f}")
        print(f"  Area improvement: {vq['marker_area_improvement']:.6f}")
        print(f"  Avg centering error: {vq['avg_x_offset_error']:.4f}")
        x_offset_stability = vq["x_offset_stability"]
        stability_display = (
            f"{x_offset_stability:.4f}" if pd.notna(x_offset_stability) else "N/A"
        )
        print(f"  Position stability (sigma): {stability_display}")

    print(f"\nACTION PERFORMANCE")
    for action, stats in metrics["action_performance"].items():
        print(f"  {action}:")
        print(f"    Count: {stats['count']}")
        print(f"    Avg duration: {stats['avg_duration']:.3f}s")
        consistency = stats["duration_consistency"]
        consistency_display = f"{consistency:.4f}s" if pd.notna(consistency) else "N/A"
        print(f"    Consistency (sigma): {consistency_display}")

    sm = metrics["state_machine"]
    print(f"\nSTATE MACHINE")
    print(f"  State transitions: {sm['total_state_changes']}")
    print(f"  Unique states: {sm['states_visited']}")
    print(f"  Distribution: {sm['state_distribution']}")

    print("\n" + "=" * 60)


if __name__ == "__main__":
    default_log_path = (
        Path(__file__).parent.parent
        / "logs"
        / "episode_logs"
        / "processed"
        / "ep1774692642_2026-03-28_11:10:53.csv"
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "csv_path",
        nargs="?",
        default=str(default_log_path),
        help="Path to a processed CSV generated by preprocess_logs.py",
    )
    args = parser.parse_args()

    metrics = extract_quality_metrics(args.csv_path)
    print_metrics(metrics)
