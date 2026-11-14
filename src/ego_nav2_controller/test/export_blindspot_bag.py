#!/usr/bin/env python3
"""Read-only export of raw costmaps + odometry, with no ROS node or publishers.

source ROS/workspace; python3 export_blindspot_bag.py BAG_DIR OUTPUT.bin
Used by test_control's RecordedBlindSpotSequence (EGO_BLINDSPOT_CASES).
"""
import argparse
import bisect
from pathlib import Path
import math
import sqlite3
import struct
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('bag', type=Path)
parser.add_argument('output', type=Path)
args = parser.parse_args()
folder = args.bag / 'bag' if (args.bag / 'bag').is_dir() else args.bag
rows = {name: [] for name in ('/local_costmap/costmap_raw', '/odom_nav')}
for file in sorted(folder.glob('*.db3')):
    with sqlite3.connect(file.resolve().as_uri() + '?mode=ro', uri=True) as db:
        for topic_id, name, typ in db.execute('select id,name,type from topics'):
            if name not in rows:
                continue
            cls = get_message(typ)
            rows[name].extend((t, deserialize_message(data, cls)) for t, data in db.execute(
                'select timestamp,data from messages where topic_id=? order by timestamp', (topic_id,)))
for values in rows.values():
    values.sort(key=lambda pair: pair[0])
    if not values:
        raise ValueError('Both costmap_raw and odom_nav must be recorded')
odom = rows['/odom_nav']
times = [t for t, _ in odom]
frames = []
for t, grid in rows['/local_costmap/costmap_raw']:
    i = bisect.bisect_right(times, t) - 1
    if i < 0 or t - times[i] > 300_000_000:
        continue
    od = odom[i][1]
    if od.header.frame_id != 'odom' or grid.header.frame_id != 'odom':
        raise ValueError('Expected odom coordinates')
    p, q = od.pose.pose.position, od.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    m = grid.metadata
    frames.append((struct.pack('<3dII3d', p.x, p.y, yaw, m.size_x, m.size_y,
                               m.resolution, m.origin.position.x, m.origin.position.y), bytes(grid.data)))
with args.output.open('wb') as out:
    out.write(struct.pack('<I', len(frames)))
    for meta, data in frames:
        out.write(meta)
        out.write(data)
print(f'Exported {len(frames)} recorded frames to {args.output}')
