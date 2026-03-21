"""Extract quality metrics from robot episode logs."""

import pandas as pd
from pathlib import Path
from typing import Dict

def extract_quality_metrics(csv_path: str) -> Dict:
    """Extract meaningful quality indicators from episode history."""
    
    df = pd.read_csv(csv_path)
    metrics = {}
    
    # === SEARCH EFFICIENCY ===
    search_actions = df[df['state'] == 'search']
    first_detection_idx = df[df['marker_visible'] == True].index[0] if any(df['marker_visible']) else len(df)
    
    metrics['search_efficiency'] = {
        'actions_before_detection': first_detection_idx,
        'forward_moves_before_detection': len(search_actions[search_actions.index < first_detection_idx]),
        'total_forward_distance_estimate': sum(search_actions[search_actions.index < first_detection_idx]['duration']) * 0.2,  # rough estimate
        'total_search_time': search_actions[search_actions.index < first_detection_idx]['duration'].sum(),
    }
    
    # === OBSTACLE AVOIDANCE ====
    avoid_actions = df[df['state'] == 'avoid']
    total_obstacles = len(df[df['obstacle_distance'] < 100]['obstacle_distance'].unique())
    
    metrics['obstacle_handling'] = {
        'obstacles_encountered': total_obstacles,
        'avoidance_actions': len(avoid_actions),
        'closest_approach': df['obstacle_distance'][df['obstacle_distance'] < 100].min(),
        'obstacle_detection_rate': (total_obstacles / len(df)) * 100,
    }
    
    # === VISION QUALITY ===
    detected_frames = df[df['marker_visible'] == True]
    if len(detected_frames) > 0:
        metrics['vision_quality'] = {
            'detections': len(detected_frames),
            'first_detection_marker_area': detected_frames.iloc[0]['marker_area'],
            'best_detection_marker_area': detected_frames['marker_area'].max(),
            'marker_area_improvement': (detected_frames['marker_area'].max() - detected_frames.iloc[0]['marker_area']) if len(detected_frames) > 1 else 0,
            'avg_x_offset_error': detected_frames['marker_x_offset'].abs().mean(),
            'x_offset_stability': detected_frames['marker_x_offset'].std(),
        }
    
    # === ACTION PERFORMANCE ===
    action_perf = {}
    for action in df['action_name'].unique():
        action_df = df[df['action_name'] == action]
        action_perf[action] = {
            'count': len(action_df),
            'avg_duration': action_df['duration'].mean(),
            'duration_consistency': action_df['duration'].std(),
        }
    metrics['action_performance'] = action_perf
    
    # === STATE MACHINE EFFICIENCY ===
    state_sequence = df['state'].values
    state_changes = sum(1 for i in range(len(state_sequence)-1) if state_sequence[i] != state_sequence[i+1])
    
    metrics['state_machine'] = {
        'total_state_changes': state_changes,
        'states_visited': df['state'].nunique(),
        'state_distribution': df['state'].value_counts().to_dict(),
    }
    
    # === OVERALL EPISODE QUALITY ===
    total_time = df['duration'].sum()
    metrics['episode_summary'] = {
        'total_duration': total_time,
        'total_actions': len(df),
        'success': 'marker_detected' if any(df['marker_visible']) else 'search_incomplete',
        'efficiency_score': (first_detection_idx / len(df)) * 100 if first_detection_idx < len(df) else 100,
    }
    
    return metrics

def print_metrics(metrics: Dict):
    """Pretty print quality metrics."""
    
    print("\n" + "="*60)
    print("ROBOT EPISODE QUALITY METRICS")
    print("="*60)
    
    # Episode Summary
    ep = metrics['episode_summary']
    print(f"\n📊 EPISODE SUMMARY")
    print(f"  Duration: {ep['total_duration']:.2f}s")
    print(f"  Total Actions: {ep['total_actions']}")
    print(f"  Result: {ep['success']}")
    print(f"  Efficiency: {ep['efficiency_score']:.1f}% (lower = found sooner)")
    
    # Search Efficiency
    se = metrics['search_efficiency']
    print(f"\n🔍 SEARCH EFFICIENCY")
    print(f"  Actions before detection: {se['actions_before_detection']}")
    print(f"  Forward moves: {se['forward_moves_before_detection']}")
    print(f"  Estimated distance: {se['total_forward_distance_estimate']:.1f}cm")
    print(f"  Search time: {se['total_search_time']:.2f}s")
    
    # Obstacle Handling
    oh = metrics['obstacle_handling']
    print(f"\n🚧 OBSTACLE HANDLING")
    print(f"  Obstacles encountered: {oh['obstacles_encountered']}")
    print(f"  Avoidance actions triggered: {oh['avoidance_actions']}")
    print(f"  Closest approach: {oh['closest_approach']:.2f}cm")
    print(f"  Detection rate: {oh['obstacle_detection_rate']:.1f}%")
    
    # Vision Quality
    if 'vision_quality' in metrics:
        vq = metrics['vision_quality']
        print(f"\n👁️  VISION QUALITY")
        print(f"  Detections: {vq['detections']}")
        print(f"  Initial marker area: {vq['first_detection_marker_area']:.6f}")
        print(f"  Best marker area: {vq['best_detection_marker_area']:.6f}")
        print(f"  Area improvement: {vq['marker_area_improvement']:.6f}")
        print(f"  Avg centering error: {vq['avg_x_offset_error']:.4f}")
        print(f"  Position stability (σ): {vq['x_offset_stability']:.4f}")
    
    # Action Performance
    print(f"\n⚡ ACTION PERFORMANCE")
    for action, stats in metrics['action_performance'].items():
        print(f"  {action}:")
        print(f"    Count: {stats['count']}")
        print(f"    Avg duration: {stats['avg_duration']:.3f}s")
        print(f"    Consistency (σ): {stats['duration_consistency']:.4f}s")
    
    # State Machine
    sm = metrics['state_machine']
    print(f"\n🔄 STATE MACHINE")
    print(f"  State transitions: {sm['total_state_changes']}")
    print(f"  Unique states: {sm['states_visited']}")
    print(f"  Distribution: {sm['state_distribution']}")
    
    print("\n" + "="*60)

if __name__ == '__main__':
    log_path = Path(__file__).parent.parent / 'logs' / 'analyzed_action_history.csv'
    metrics = extract_quality_metrics(str(log_path))
    print_metrics(metrics)
