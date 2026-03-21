import pandas as pd
import json

load_path = 'logs/action_history/history_2026-03-21_20:36:18.json'
save_path = "logs/analyzed_action_history.csv"

with open(load_path, "r") as f:
    records = json.load(f)

def process_record(record):
    iteration = record["iteration"]
    state = record["state"]
    obstacle_distance = record["sensor_snapshot"]["obstacle_distance_cm"]
    marker_visible = record["sensor_snapshot"]["marker"]["visible"]
    marker_x_offset = record["sensor_snapshot"]["marker"]["x_offset"]
    marker_area = record["sensor_snapshot"]["marker"]["area"]
    sub_actions = record["sub_actions"]

    actions = []
    for sub in sub_actions:
        action = {}
        action["iteration"] = iteration
        action["state"] = state
        action["obstacle_distance"] = obstacle_distance
        action["marker_visible"] = marker_visible
        action["marker_x_offset"] = marker_x_offset
        action["marker_area"] = marker_area

        action["action_name"] = sub["name"]
        if sub['parameters']:
            action['parameters'] = sub["parameters"]['distance'] if 'distance' in sub["parameters"] else sub['parameters']['degrees']
        action['duration'] = sub['duration']
        actions.append(action)

    return actions

all_actions = []
for record in records:
    actions = process_record(record)
    all_actions.extend(actions)

data = pd.DataFrame(all_actions)
data.to_csv(save_path, index=False)
