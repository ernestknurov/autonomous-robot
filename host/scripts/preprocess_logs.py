import pandas as pd
import json

filename = "ep1774692050_2026-03-28_11:01:34"
load_path = f'logs/episode_logs/{filename}.json'
save_path = f"logs/episode_logs/processed/{filename}.csv"

with open(load_path, "r") as f:
    logs = json.load(f)

def process_step(record):

    action = {}
    action['episode_id'] = record['episode_id']
    action["iteration"] = record["iteration"]
    action["state"] = record["state"]
    action['action'] = record['action']['name']
    action['action_param_distance'] = record['action']['parameters']['distance'] if 'distance' in record['action']['parameters'] else None
    action['action_param_degrees'] = record['action']['parameters']['degrees'] if 'degrees' in record['action']['parameters'] else None

    action['obs_obstacle_distance'] = record['observation']['obstacle_distance_cm']
    action['obs_marker_visible'] = record['observation']['marker_visible']
    action['obs_marker_x_offset'] = record['observation']['marker_x_offset']
    action['obs_marker_area'] = record['observation']['marker_area']
    action['obs_marker_id'] = record['observation']['marker_id']

    next_obs = record['next_observation'] if 'next_observation' in record else None
    action['next_obs_obstacle_distance'] = next_obs['obstacle_distance_cm'] if next_obs else None
    action['next_obs_marker_visible'] = next_obs['marker_visible'] if next_obs else None
    action['next_obs_marker_x_offset'] = next_obs['marker_x_offset'] if next_obs else None
    action['next_obs_marker_area'] = next_obs['marker_area'] if next_obs else None
    action['next_obs_marker_id'] = next_obs['marker_id'] if next_obs else None

    try:
        action['duration'] = record['timestamps']['action_end_ts'] - record['timestamps']['action_start_ts']
    except:
        action['duration'] = None

    outcome = record['outcome'] if 'outcome' in record else None
    action['nest_state'] = outcome['next_state'] if outcome else None
    action['terminal'] = outcome['terminal'] if outcome else None

    return action

all_actions = []
steps = logs.get('steps', [])
all_actions = [process_step(step) for step in steps]

data = pd.DataFrame(all_actions)
data.to_csv(save_path, index=False)
