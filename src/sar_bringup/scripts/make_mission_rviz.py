#!/usr/bin/env python3
"""Generate rviz/mission.rviz: Nav2 default view + victim detections, victim registry, ArUco camera image."""
import os

import yaml

src = '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'rviz', 'mission.rviz')

cfg = yaml.safe_load(open(src))
displays = cfg['Visualization Manager']['Displays']


def marker_array(name, topic):
    return {'Class': 'rviz_default_plugins/MarkerArray', 'Enabled': True, 'Name': name,
            'Namespaces': {}, 'Topic': {'Depth': 5, 'Durability Policy': 'Volatile',
                                        'History Policy': 'Keep Last', 'Reliability Policy': 'Reliable',
                                        'Value': topic}, 'Value': True}


displays.append(marker_array('Victim detections (live)', '/victim_markers'))
displays.append(marker_array('Victim registry', '/victim_registry_markers'))
displays.append({'Class': 'rviz_default_plugins/Image', 'Enabled': True, 'Name': 'ArUco camera',
                 'Max Value': 1, 'Median window': 5, 'Min Value': 0, 'Normalize Range': True,
                 'Topic': {'Depth': 5, 'Durability Policy': 'Volatile', 'History Policy': 'Keep Last',
                           'Reliability Policy': 'Best Effort', 'Value': '/aruco/debug_image'},
                 'Value': True})

yaml.safe_dump(cfg, open(out, 'w'), default_flow_style=False, sort_keys=False)
print(f'Wrote {os.path.abspath(out)}')
