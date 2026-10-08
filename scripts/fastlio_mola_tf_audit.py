#!/usr/bin/env python3
"""Passive TF/pose recorder and same-timestamp comparison, without publishers."""

import argparse
from bisect import bisect_left
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Float32, String
from tf2_msgs.msg import TFMessage


POSE_TOPICS = (
    "/Odometry", "/fastlio/odometry_livox",
    "/lidar_odometry/pose", "/state_estimation/pose",
)


def normalize(q):
    norm = math.sqrt(sum(x * x for x in q))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid orientation quaternion")
    return [x / norm for x in q]


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def rotate(q, v):
    return multiply(multiply(q, [*v, 0.0]), [-q[0], -q[1], -q[2], q[3]])[:3]


def compose(a, b):
    q = normalize(a[3:])
    offset = rotate(q, b[:3])
    return [a[i] + offset[i] for i in range(3)] + normalize(multiply(q, normalize(b[3:])))


def inverse(p):
    q = normalize(p[3:])
    q = [-q[0], -q[1], -q[2], q[3]]
    return rotate(q, [-x for x in p[:3]]) + q


def difference(a, b):
    distance = math.sqrt(sum((a[i]-b[i])**2 for i in range(3)))
    qa, qb = normalize(a[3:]), normalize(b[3:])
    cosine = min(1.0, abs(sum(x*y for x, y in zip(qa, qb))))
    return distance, math.degrees(2.0 * math.acos(cosine))


def interpolate_pose(a, b, fraction):
    qa, qb = normalize(a[3:]), normalize(b[3:])
    dot = sum(x*y for x, y in zip(qa, qb))
    if dot < 0:  # q and -q encode the same orientation; take the short arc.
        qb, dot = [-x for x in qb], -dot
    dot = min(1.0, dot)
    if dot > 0.9995:
        q = normalize([x + fraction*(y-x) for x, y in zip(qa, qb)])
    else:
        theta = math.acos(dot)
        left = math.sin((1-fraction)*theta) / math.sin(theta)
        right = math.sin(fraction*theta) / math.sin(theta)
        q = [left*x + right*y for x, y in zip(qa, qb)]
    return [a[i] + fraction*(b[i]-a[i]) for i in range(3)] + q


class Timeline:
    """Sort for alignment only; arrival-order regressions remain in raw data."""

    def __init__(self, rows):
        # Keep the last received revision of a duplicate timestamp.
        unique = {row["stamp"]: row for row in rows}
        self.stamps = sorted(unique)
        self.rows = [unique[s] for s in self.stamps]

    def sample(self, stamp, max_gap):
        i = bisect_left(self.stamps, stamp)
        if i < len(self.stamps) and self.stamps[i] == stamp:
            return self.rows[i]
        if i == 0 or i == len(self.stamps):
            return None  # Never extrapolate or silently reuse the latest pose.
        a, b = self.rows[i-1], self.rows[i]
        gap = b["stamp"] - a["stamp"]
        if gap > max_gap or (a["frame"], a["child"]) != (b["frame"], b["child"]):
            return None
        return {"stamp": stamp, "frame": a["frame"], "child": a["child"],
                "pose": interpolate_pose(a["pose"], b["pose"], (stamp-a["stamp"])/gap)}


def stream_summary(rows, jump_m, jump_deg):
    steps, angles, dt = [], [], []
    for a, b in zip(rows, rows[1:]):
        if (a["frame"], a["child"]) != (b["frame"], b["child"]):
            continue
        d, angle = difference(a["pose"], b["pose"])
        steps.append(d)
        angles.append(angle)
        dt.append(b["stamp"]-a["stamp"])
    lags = [r["received_ros"]-r["stamp"] for r in rows if "received_ros" in r]
    return {"count": len(rows), "max_step_m": max(steps, default=0.0),
            "max_step_deg": max(angles, default=0.0),
            "position_jump_count": sum(d > jump_m for d in steps),
            "rotation_jump_count": sum(a > jump_deg for a in angles),
            "stamp_regressions": sum(t < -1e-6 for t in dt),
            "duplicate_stamps": sum(abs(t) <= 1e-6 for t in dt),
            "lag_s_min_median_max": [min(lags), statistics.median(lags), max(lags)] if lags else None,
            "frames": sorted({(r["frame"], r["child"]) for r in rows})}


def aligned_comparison(source, target, max_gap):
    """Absolute differences require identical frames; relative motion does too."""
    timeline = Timeline(target)
    pairs, skipped = [], 0
    for a in Timeline(source).rows:
        b = timeline.sample(a["stamp"], max_gap)
        if b is None or (a["frame"], a["child"]) != (b["frame"], b["child"]):
            skipped += 1
            continue
        d, angle = difference(a["pose"], b["pose"])
        pairs.append({"stamp": a["stamp"], "position_difference_m": d,
                      "orientation_difference_deg": angle, "a": a["pose"], "b": b["pose"]})
    relative = []
    for a, b in zip(pairs, pairs[1:]):
        if b["stamp"]-a["stamp"] > max_gap:
            continue
        da = compose(inverse(a["a"]), b["a"])
        db = compose(inverse(a["b"]), b["b"])
        d, angle = difference(da, db)
        relative.append({"stamp": b["stamp"], "translation_difference_m": d,
                         "rotation_difference_deg": angle})
    return {"matched": len(pairs), "skipped": skipped, "samples": pairs,
            "relative_motion_differences": relative}


def reconstructed_map_odom(source, fastlio, max_gap):
    timeline = Timeline(fastlio)
    rows, skipped = [], 0
    for row in Timeline(source).rows:
        odom = timeline.sample(row["stamp"], max_gap)
        if (odom is None or row["frame"] != "map" or odom["frame"] != "odom"
                or row["child"] != odom["child"]):
            skipped += 1
            continue
        rows.append({"stamp": row["stamp"], "frame": "map", "child": "odom",
                     "pose": compose(row["pose"], inverse(odom["pose"]))})
    return {"samples": rows, "skipped": skipped}


def analyze(data, max_gap=0.3, jump_m=0.3, jump_deg=12.0):
    streams = data["streams"]
    summaries = {k: stream_summary(v, jump_m, jump_deg) for k, v in streams.items()}
    raw = streams.get("/lidar_odometry/pose", [])
    smoothed = streams.get("/state_estimation/pose", [])
    fastlio = streams.get("/fastlio/odometry_livox", [])
    reconstructed = {}
    for key, rows in (("raw", raw), ("smoothed", smoothed)):
        result = reconstructed_map_odom(rows, fastlio, max_gap)
        result["summary"] = stream_summary(result["samples"], jump_m, jump_deg)
        reconstructed[key] = result
    return {"streams": summaries,
            "missing_pose_topics": [t for t in POSE_TOPICS if not streams.get(t)],
            "raw_vs_smoothed": aligned_comparison(raw, smoothed, max_gap),
            "reconstructed_map_odom": reconstructed,
            "note": "TF map->odom stamps are future-dated and rebroadcast; receipt lag is not localization latency. Reconstructed transforms use sensor timestamps, no extrapolation. Missing data or quality=1 does not establish correctness."}


def stamp(s):
    return s.sec + s.nanosec * 1e-9


def pose_vector(p, q):
    return [p.x, p.y, p.z, *normalize([q.x, q.y, q.z, q.w])]


class Audit(Node):
    def __init__(self):
        super().__init__("fastlio_mola_tf_audit")
        self.start = time.monotonic()
        self.data = {"streams": {t: [] for t in POSE_TOPICS}, "quality": {},
                     "status": [], "static_tf": {}, "sensor_headers": {}, "invalid_pose_count": 0}
        self.subscriptions_kept = []
        qos = QoSProfile(depth=500, reliability=ReliabilityPolicy.BEST_EFFORT)
        for topic in POSE_TOPICS:
            self.subscribe(Odometry, topic, lambda t, m: self.odometry(t, m), qos)
        self.subscribe(TFMessage, "/tf", lambda t, m: self.tf(m), qos)
        static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscribe(TFMessage, "/tf_static", lambda t, m: self.static(m), static_qos)
        for topic in ("/lidar_odometry/pose_quality", "/state_estimation/pose_quality"):
            self.subscribe(Float32, topic, self.quality, qos)
        self.subscribe(String, "/mola_diagnostics/lidar_odom/status", self.status, qos)
        self.subscribe(Imu, "/livox/imu", self.header, qos)
        self.subscribe(PointCloud2, "/livox/points", self.header, qos)

    def subscribe(self, msg_type, topic, handler, qos):
        # Exactly one argument: Jazzy can pass a dict instead of MessageInfo,
        # and some releases do not expose publisher_gid. Use graph endpoints.
        def callback(message):
            handler(topic, message)
        self.subscriptions_kept.append(self.create_subscription(msg_type, topic, callback, qos))

    def receipt(self):
        return {"received_ros": self.get_clock().now().nanoseconds * 1e-9,
                "elapsed": time.monotonic()-self.start}

    def record(self, key, header, child, position, quaternion, covariance=None, twist=None):
        try:
            p = pose_vector(position, quaternion)
            if not all(math.isfinite(x) for x in p):
                raise ValueError("nonfinite pose")
        except ValueError:
            self.data["invalid_pose_count"] += 1
            return
        row = {**self.receipt(), "stamp": stamp(header.stamp), "frame": header.frame_id,
               "child": child, "pose": p}
        if covariance is not None:
            row["covariance"] = list(covariance)
            row["twist"] = [twist.linear.x, twist.linear.y, twist.linear.z,
                            twist.angular.x, twist.angular.y, twist.angular.z]
        self.data["streams"].setdefault(key, []).append(row)

    def odometry(self, topic, m):
        self.record(topic, m.header, m.child_frame_id, m.pose.pose.position,
                    m.pose.pose.orientation, m.pose.covariance, m.twist.twist)

    def tf(self, message):
        for t in message.transforms:
            self.record(t.header.frame_id+"->"+t.child_frame_id, t.header, t.child_frame_id,
                        t.transform.translation, t.transform.rotation)

    def static(self, message):
        for t in message.transforms:
            key = t.header.frame_id+"->"+t.child_frame_id
            try:
                self.data["static_tf"][key] = pose_vector(t.transform.translation, t.transform.rotation)
            except ValueError:
                self.data["invalid_pose_count"] += 1

    def quality(self, topic, m):
        self.data["quality"].setdefault(topic, []).append({**self.receipt(), "value": m.data})

    def status(self, topic, m):
        self.data["status"].append({**self.receipt(), "topic": topic, "text": m.data})

    def header(self, topic, m):
        self.data["sensor_headers"].setdefault(topic, []).append(
            {**self.receipt(), "stamp": stamp(m.header.stamp), "frame": m.header.frame_id})

    def publishers(self):
        topics = [*POSE_TOPICS, "/tf", "/tf_static"]
        return {t: [{"node": p.node_name, "namespace": p.node_namespace,
                     "gid": bytes(p.endpoint_gid).hex()}
                    for p in self.get_publishers_info_by_topic(t)] for t in topics}


def positive(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=positive, default=40.0)
    parser.add_argument("--label", default="motion")
    parser.add_argument("--output", type=Path, help="JSON output; existing files are never overwritten")
    parser.add_argument("--max-gap", type=positive, default=0.3, help="maximum interpolation gap in seconds")
    parser.add_argument("--jump-m", type=positive, default=0.3)
    parser.add_argument("--jump-deg", type=positive, default=12.0)
    args = parser.parse_args(remove_ros_args()[1:])
    now = datetime.now(timezone.utc)
    output = args.output or Path("runs/tf-diagnostics") / (now.strftime("%Y%m%dT%H%M%S.%fZ") + ".json")
    output.parent.mkdir(parents=True, exist_ok=True)
    # Reserve output before recording; an unwritable path must not lose a run.
    with output.open("x", encoding="utf-8") as destination:
        rclpy.init()
        node = Audit()
        print(f"Recording {args.duration:g}s ({args.label}); no TF or pose publishers.", flush=True)
        interrupted = False
        next_progress = 5.0
        try:
            while time.monotonic()-node.start < args.duration and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                elapsed = time.monotonic()-node.start
                if elapsed >= next_progress:
                    print(f"{elapsed:.0f}s: " + ", ".join(
                        f"{t}={len(node.data['streams'][t])}" for t in POSE_TOPICS), flush=True)
                    next_progress += 5.0
        except (KeyboardInterrupt, ExternalShutdownException):
            interrupted = True
        finally:
            node.data["metadata"] = {"label": args.label, "started_utc": now.isoformat(),
                                     "duration_s": time.monotonic()-node.start,
                                     "interrupted": interrupted,
                                     "max_gap_s": args.max_gap, "jump_m": args.jump_m,
                                     "jump_deg": args.jump_deg}
            node.data["publishers"] = node.publishers() if rclpy.ok() else {}
            node.data["analysis"] = analyze(node.data, args.max_gap, args.jump_m, args.jump_deg)
            json.dump(node.data, destination, ensure_ascii=False, indent=2)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    analysis = node.data["analysis"]
    for key, summary in analysis["streams"].items():
        print(key, json.dumps(summary, ensure_ascii=False))
    print("Same-timestamp raw/smoothed matches:", analysis["raw_vs_smoothed"]["matched"])
    if analysis["missing_pose_topics"]:
        print("WARNING missing streams:", ", ".join(analysis["missing_pose_topics"]))
    print(analysis["note"])
    print(f"Saved: {output.resolve()}")


if __name__ == "__main__":
    main()
