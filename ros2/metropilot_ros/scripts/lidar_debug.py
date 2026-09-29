#!/usr/bin/env python3
"""Publish real SQLite ROS2 clouds or a generated rail/obstacle scene."""
import argparse
import csv
import json
import math
import struct
import time
from collections import deque
from pathlib import Path

import numpy as np
from bag_sequence import bag_clouds, discover_bag_files
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool


def synthetic_cloud(index, obstacle=False, distance=12.0, schema="enriched",
                    obstacle_bottom=0.45, obstacle_height=0.30):
    """Deterministic geometry fixture; not a physical lidar range simulation."""
    points = []
    for x in np.arange(1.0, 40.0, 0.10):
        for y in (-0.775, 0.775):
            for jitter in (-0.006, -0.002, 0.002, 0.006):
                points.append((x, y + jitter, 0.0))
        for z in (0.3, 0.8, 1.3, 1.8, 2.3, 2.8):
            points.extend(((x, -2.4, z), (x, 2.4, z)))
    if obstacle:
        for x in np.arange(distance, distance + 0.31, 0.04):
            for y in np.arange(-0.15, 0.16, 0.04):
                for z in np.arange(obstacle_bottom, obstacle_bottom + obstacle_height + 0.01, 0.04):
                    points.append((x, y, z))
    names = ["x", "y", "z"] + ([] if schema == "xyz" else ["intensity"])
    formats = ["<f4"] * len(names)
    if schema == "enriched":
        names += ["ring", "timestamp"]
        formats += ["<u2", "<f8"]
    values = np.zeros(len(points), dtype=np.dtype(list(zip(names, formats))))
    vehicle = np.asarray(points)
    values["x"] = vehicle[:, 1]
    values["y"] = -vehicle[:, 0]
    values["z"] = vehicle[:, 2] - 1.075
    if "intensity" in names:
        values["intensity"] = 32
    if schema == "enriched":
        values["ring"] = np.arange(len(points)) % 128
        # Give even the low-object fixture >=4 acquisition groups; grouping
        # every 128 points gave it only two groups, intentionally too weak for
        # the enriched precision gate irrespective of obstacle height.
        values["timestamp"] = 1000 + index * 0.1 + np.arange(len(points)) // 32 * 0.0001
    cloud = PointCloud2()
    cloud.header.frame_id = "hesai_lidar"
    ns = 1_000_000_000_000 + index * 100_000_000
    cloud.header.stamp.sec, cloud.header.stamp.nanosec = divmod(ns, 1_000_000_000)
    cloud.height, cloud.width = 1, len(points)
    for name in names:
        dtype, offset = values.dtype.fields[name]
        datatype = {"f": PointField.FLOAT64 if dtype.itemsize == 8 else PointField.FLOAT32,
                    "u": PointField.UINT16}[dtype.kind]
        cloud.fields.append(PointField(name=name, offset=offset, datatype=datatype, count=1))
    cloud.point_step = values.dtype.itemsize
    cloud.row_step = cloud.width * cloud.point_step
    cloud.is_dense = True
    cloud.data = values.tobytes()
    return cloud


def source_time_ns(cloud):
    if isinstance(cloud, bytes):
        if len(cloud) < 12 or cloud[:2] not in (b"\x00\x00", b"\x00\x01"):
            raise ValueError("Expected CDR1 serialized PointCloud2")
        sec, ns = struct.unpack_from("<iI" if cloud[1] == 1 else ">iI", cloud, 4)
        return sec * 1_000_000_000 + ns
    return cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec


class DebugPublisher(Node):
    def __init__(self, topic="/lidar_points", output="/obstacle_detected"):
        super().__init__("lidar_debug")
        self.publisher = self.create_publisher(PointCloud2, topic,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.answers = deque(maxlen=100)
        self.subscription = self.create_subscription(Bool, output,
            lambda msg: self.answers.append((time.monotonic(), bool(msg.data))), 100)

    def wait_connected(self, timeout=20):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.publisher.get_subscription_count() and self.count_publishers(self.subscription.topic_name):
                # Allow discovery/transport to settle before the first cloud.
                self.spin_for(0.3)
                return
        raise TimeoutError("Detector not discovered on input/output topics")

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0, deadline - time.monotonic())))

    def send_and_wait(self, cloud, timeout=5.0):
        # Only one request in flight. Bool has no stamp: do not mix publishers
        # or use this method concurrently with another debug application.
        self.spin_for(0.005)
        self.answers.clear()
        begin = time.monotonic()
        self.publisher.publish(cloud)
        while time.monotonic() - begin < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.answers:
                return self.answers[-1][1], (self.answers[-1][0] - begin) * 1000
        raise TimeoutError("No Bool response; check node logs and QoS")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", type=Path, help="SQLite .db3 file or split bag directory")
    parser.add_argument("--skip-unreadable", action="store_true", help="Explicitly allow missing/corrupt parts of a partial bag")
    parser.add_argument("--no-cache", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--source-topic")
    parser.add_argument("--topic", default="/lidar_points")
    parser.add_argument("--output-topic", default="/obstacle_detected")
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--end", type=int)
    parser.add_argument("--scenario", choices=["empty", "obstacle", "sequence"], default="sequence")
    parser.add_argument("--schema", choices=["enriched", "xyzi", "xyz"], default="enriched")
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--distance", type=float, default=12.0)
    parser.add_argument("--rate", type=float, default=10.0, help="Fixed Hz; 0 = bag header timing")
    parser.add_argument("--speed", type=float, default=1.0, help="Multiplier for header timing")
    parser.add_argument("--realtime", action="store_true", help="Stream without waiting for each Bool")
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--report", type=Path, help="CSV (lockstep only)")
    args = parser.parse_args()
    if args.start < 0 or (args.end is not None and args.end < args.start) or args.frames < 1:
        parser.error("Invalid frame interval")
    if not math.isfinite(args.rate) or args.rate < 0 or not math.isfinite(args.speed) or args.speed <= 0:
        parser.error("rate must be >=0 and speed >0")
    if args.realtime and args.report:
        parser.error("Bool has no frame ID: reports require lockstep (omit --realtime)")
    if not math.isfinite(args.distance) or not 1 <= args.distance <= 35:
        parser.error("synthetic --distance must be in [1,35] m")
    rclpy.init()
    node = DebugPublisher(args.topic, args.output_topic)
    output_file = None
    try:
        db_paths = discover_bag_files(args.db, args.skip_unreadable) if args.db else []
        if args.report and args.report.resolve() in {p.resolve() for p in db_paths}:
            raise ValueError("Report must not overwrite an input database")
        node.wait_connected()
        writer = None
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            output_file = args.report.open("w", newline="")
            writer = csv.writer(output_file)
            writer.writerow(["loop", "frame", "source_time_ns", "flag", "latency_ms", "expected_scene"])
        loop_index = 0
        while rclpy.ok():
            if args.db:
                sequence = bag_clouds(db_paths, args.source_topic, args.start, args.end)
            else:
                sequence = ((i, synthetic_cloud(i, args.scenario == "obstacle" or
                    (args.scenario == "sequence" and args.frames // 3 <= i < 2 * args.frames // 3),
                    args.distance, args.schema), args.scenario == "obstacle" or
                    (args.scenario == "sequence" and args.frames // 3 <= i < 2 * args.frames // 3))
                    for i in range(args.frames))
            flags, latencies, sent = [], [], 0
            wall_start, first_stamp = time.monotonic(), None
            for index, cloud, expected in sequence:
                stamp = source_time_ns(cloud)
                if first_stamp is None:
                    first_stamp = stamp
                offset = sent / args.rate if args.rate else max(0, stamp - first_stamp) * 1e-9 / args.speed
                node.spin_for(max(0, wall_start + offset - time.monotonic()))
                if args.realtime:
                    node.publisher.publish(cloud)
                    rclpy.spin_once(node, timeout_sec=0)
                else:
                    flag, latency = node.send_and_wait(cloud)
                    flags.append(flag)
                    latencies.append(latency)
                    if writer:
                        writer.writerow([loop_index, index, stamp, int(flag), round(latency, 3), expected])
                    if sent % 25 == 0:
                        print(f"frame={index} obstacle={flag} roundtrip_ms={latency:.1f}", flush=True)
                sent += 1
            print(json.dumps({"sent": sent, "responses": len(flags), "true": sum(flags),
                "roundtrip_p95_ms": float(np.percentile(latencies, 95)) if latencies else None}), flush=True)
            if not sent:
                raise ValueError("No frames selected; check --start/--end and the source topic")
            if output_file:
                output_file.flush()
            if not args.loop:
                break
            loop_index += 1
            node.spin_for(0.6)
    except KeyboardInterrupt:
        pass
    finally:
        if output_file:
            output_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
