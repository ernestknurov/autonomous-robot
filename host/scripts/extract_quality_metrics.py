"""Extract quality metrics from robot episode logs."""

import argparse
from pathlib import Path
from typing import Dict

import pandas as pd


OBSTACLE_DETECTION_THRESHOLD_CM = 100.0


def _normalize_bool_series(series: pd.Series, fallback: bool = False) -> pd.Series:
    """Normalize CSV bools that may arrive as bools, strings, numbers, or blanks."""
    normalized = series.astype(str).str.lower().map(
        {
            "true": True,
            "1": True,
            "1.0": True,
            "yes": True,
            "false": False,
            "0": False,
            "0.0": False,
            "no": False,
            "nan": pd.NA,
            "none": pd.NA,
            "": pd.NA,
        }
    )
    return normalized.fillna(fallback).astype(bool)


def _normalize_dataframe(df: pd.DataFrame) -> pd.DataFrame:
    """Normalize legacy and preprocessed CSV schemas into one shape."""
    df = df.copy()

    rename_map = {
        "action": "action_name",
        "obs_obstacle_distance": "obstacle_distance",
        "obs_marker_visible": "marker_visible",
        "obs_marker_x_offset": "marker_x_offset",
        "obs_marker_area": "marker_area",
        "obs_depth_blocked": "depth_blocked",
        "obs_depth_score": "depth_score",
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
        ("depth_blocked", False),
        ("depth_score", pd.NA),
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
    df["marker_visible"] = _normalize_bool_series(df["marker_visible"])
    df["depth_blocked"] = _normalize_bool_series(df["depth_blocked"])
    df["obstacle_distance"] = pd.to_numeric(df["obstacle_distance"], errors="coerce")
    df["marker_x_offset"] = pd.to_numeric(df["marker_x_offset"], errors="coerce")
    df["marker_area"] = pd.to_numeric(df["marker_area"], errors="coerce")
    df["depth_score"] = pd.to_numeric(df["depth_score"], errors="coerce")

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
    depth_events = df[df["depth_blocked"]]
    any_obstacle_events = df[
        (
            df["obstacle_distance"].notna()
            & (df["obstacle_distance"] < OBSTACLE_DETECTION_THRESHOLD_CM)
        )
        | df["depth_blocked"]
    ]
    avoid_actions = df[df["state"] == "avoid"]
    obstacle_event_count = len(obstacle_events)
    depth_event_count = len(depth_events)
    any_obstacle_event_count = len(any_obstacle_events)

    metrics["obstacle_handling"] = {
        "distance_obstacles_encountered": obstacle_event_count,
        "depth_obstacles_encountered": depth_event_count,
        "obstacles_encountered": any_obstacle_event_count,
        "avoidance_actions": len(avoid_actions),
        "closest_approach": obstacle_events["obstacle_distance"].min()
        if obstacle_event_count
        else None,
        "max_depth_score": df["depth_score"].max()
        if df["depth_score"].notna().any()
        else None,
        "avg_depth_score": df["depth_score"].mean()
        if df["depth_score"].notna().any()
        else None,
        "depth_block_rate": (depth_event_count / len(df)) * 100 if len(df) else 0.0,
        "obstacle_detection_rate": (any_obstacle_event_count / len(df)) * 100
        if len(df)
        else 0.0,
    }

    # === DEPTH SENSOR QUALITY ===
    depth_frames = df[df["depth_score"].notna()]
    if not depth_frames.empty:
        distance_blocked = (
            depth_frames["obstacle_distance"].notna()
            & (depth_frames["obstacle_distance"] < OBSTACLE_DETECTION_THRESHOLD_CM)
        )
        depth_blocked = depth_frames["depth_blocked"]
        metrics["depth_quality"] = {
            "samples": len(depth_frames),
            "blocked_samples": int(depth_blocked.sum()),
            "max_depth_score": depth_frames["depth_score"].max(),
            "avg_depth_score": depth_frames["depth_score"].mean(),
            "depth_score_stability": depth_frames["depth_score"].std(),
            "distance_depth_agreement_rate": (
                (distance_blocked == depth_blocked).sum() / len(depth_frames)
            )
            * 100,
            "depth_only_blocks": int((depth_blocked & ~distance_blocked).sum()),
            "distance_only_blocks": int((distance_blocked & ~depth_blocked).sum()),
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
    print(f"  Distance obstacles: {oh['distance_obstacles_encountered']}")
    print(f"  Depth obstacles: {oh['depth_obstacles_encountered']}")
    print(f"  Avoidance actions triggered: {oh['avoidance_actions']}")
    closest_approach = oh["closest_approach"]
    closest_approach_display = (
        f"{closest_approach:.2f}cm" if closest_approach is not None else "N/A"
    )
    print(f"  Closest approach: {closest_approach_display}")
    max_depth_score = oh["max_depth_score"]
    max_depth_score_display = (
        f"{max_depth_score:.4f}" if max_depth_score is not None else "N/A"
    )
    avg_depth_score = oh["avg_depth_score"]
    avg_depth_score_display = (
        f"{avg_depth_score:.4f}" if avg_depth_score is not None else "N/A"
    )
    print(f"  Max depth score: {max_depth_score_display}")
    print(f"  Avg depth score: {avg_depth_score_display}")
    print(f"  Depth block rate: {oh['depth_block_rate']:.1f}%")
    print(f"  Detection rate: {oh['obstacle_detection_rate']:.1f}%")

    if "depth_quality" in metrics:
        dq = metrics["depth_quality"]
        print(f"\nDEPTH QUALITY")
        print(f"  Samples: {dq['samples']}")
        print(f"  Blocked samples: {dq['blocked_samples']}")
        print(f"  Max score: {dq['max_depth_score']:.4f}")
        print(f"  Avg score: {dq['avg_depth_score']:.4f}")
        depth_stability = dq["depth_score_stability"]
        depth_stability_display = (
            f"{depth_stability:.4f}" if pd.notna(depth_stability) else "N/A"
        )
        print(f"  Score stability (sigma): {depth_stability_display}")
        print(f"  Distance/depth agreement: {dq['distance_depth_agreement_rate']:.1f}%")
        print(f"  Depth-only blocks: {dq['depth_only_blocks']}")
        print(f"  Distance-only blocks: {dq['distance_only_blocks']}")

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
        / "ep1776500276_2026-04-18_10:20:22.csv"
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
