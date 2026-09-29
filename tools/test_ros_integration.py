#!/usr/bin/env python3
"""Run with a fresh detector in the same ROS domain. Exercises real DDS I/O."""
import copy
import struct
import sys
import sqlite3
import tempfile
from pathlib import Path

import rclpy
from rclpy.serialization import serialize_message

# In the image, the debug tool is installed as an executable without .py.
import importlib.machinery
import importlib.util
path = Path("/ws/install/lib/metropilot_ros/lidar_debug")
sys.path.insert(0, str(path.parent))
loader = importlib.machinery.SourceFileLoader("lidar_debug", str(path))
spec = importlib.util.spec_from_loader(loader.name, loader)
debug = importlib.util.module_from_spec(spec)
loader.exec_module(debug)


def main():
    rclpy.init()
    node = debug.DebugPublisher()
    index = 0
    checks = 0
    def send(cloud, expected):
        nonlocal checks
        flag, latency = node.send_and_wait(cloud)
        assert flag == expected, f"check {checks}: expected={expected} actual={flag}"
        checks += 1
        return latency
    try:
        node.wait_connected()
        for schema in ("enriched", "xyzi", "xyz"):
            for _ in range(3):
                send(debug.synthetic_cloud(index, schema=schema), False)
                index += 1
            for i in range(6):
                send(debug.synthetic_cloud(index, True, schema=schema), i >= 2)
                index += 1
            node.answers.clear()
            node.spin_for(0.8)
            assert node.answers and node.answers[-1][1] is False, "stale input must clear true flag"
            checks += 1
            send(debug.synthetic_cloud(index, True, schema=schema), False)
            index += 1
            send(debug.synthetic_cloud(index, schema=schema), False)
            index += 1
        # Reject below-rail patches, preserve low above-rail objects in all schemas.
        for schema in ("enriched", "xyzi", "xyz"):
            for bottom, height, detected in ((-0.12, 0.04, False), (0.06, 0.08, True)):
                send(debug.synthetic_cloud(index, schema=schema), False)
                index += 1
                for i in range(4):
                    send(debug.synthetic_cloud(index, True, schema=schema,
                        obstacle_bottom=bottom, obstacle_height=height), detected and i >= 2)
                    index += 1
        send(debug.synthetic_cloud(index), False)
        index += 1
        # Two physical bag parts form one continuous obstacle track over DDS.
        with tempfile.TemporaryDirectory() as folder:
            for part, indices in enumerate((range(2), range(2, 6))):
                db = sqlite3.connect(str(Path(folder) / f"split_{part}.db3"))
                try:
                    db.executescript("CREATE TABLE topics(id INTEGER,name TEXT,type TEXT);"
                        "CREATE TABLE messages(id INTEGER,timestamp INTEGER,topic_id INTEGER,data BLOB);")
                    db.execute("INSERT INTO topics VALUES(1,'/lidar_points','sensor_msgs/msg/PointCloud2')")
                    for i in indices:
                        raw = serialize_message(debug.synthetic_cloud(index + i, True))
                        db.execute("INSERT INTO messages VALUES(?,?,1,?)", (i, i, raw))
                    db.commit()
                finally:
                    db.close()
            paths = debug.discover_bag_files(Path(folder))
            for i, raw, _ in debug.bag_clouds(paths):
                send(raw, i >= 2)
        index += 6
        send(debug.synthetic_cloud(index), False)
        index += 1
        # Same real DDS payload size as the 921,600-slot organizer bag. Origin
        # padding must not increase evidence. Publish raw CDR to avoid Python's
        # byte-array conversion cost obscuring the transport test.
        large = debug.synthetic_cloud(index, True)
        target_size = 921600 * large.point_step
        large.data = bytes(large.data) + bytes(target_size - len(large.data))
        large.width = 921600
        large.row_step = target_size
        raw = serialize_message(large)
        for i in range(3):
            stamp = 1_000_000_000_000 + (index + i) * 100_000_000
            sec, ns = divmod(stamp, 1_000_000_000)
            send(raw[:4] + struct.pack("<iI", sec, ns) + raw[12:], i >= 2)
        index += 3
        send(debug.synthetic_cloud(index), False)
        # Origin-filled cloud, bad schema, unknown frame, and source resets.
        empty = debug.synthetic_cloud(index)
        empty.data = bytes(len(empty.data))
        send(empty, False)
        malformed = copy.deepcopy(empty)
        malformed.row_step = 1
        send(malformed, False)
        wrong = debug.synthetic_cloud(index + 1, True)
        wrong.header.frame_id = "unconfigured_frame"
        send(wrong, False)
        for _ in range(4):
            send(debug.synthetic_cloud(5, True), False)
        for i in range(6, 9):
            send(debug.synthetic_cloud(i, True), i >= 7)
        send(debug.synthetic_cloud(0, True), False)
        send(debug.synthetic_cloud(100, True), False)
        send(debug.synthetic_cloud(101), False)
        print(f"PASS: {checks} ROS integration assertions", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
