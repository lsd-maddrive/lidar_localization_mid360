#!/usr/bin/env python3
"""Record raw odometry/IMU; Enter marks stationary endpoints, q saves early."""
import argparse
import csv
import json
import math
import select
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2


def fit_segment(rows, origin, direction):
    p = np.asarray([r[2:5] for r in rows], dtype=float) - origin
    if len(p) < 10:
        return {'valid': False, 'reason': 'too few samples'}
    s = p[:, :2] @ direction
    if np.ptp(s) < 2:
        return {'valid': False, 'reason': 'travel below 2 m'}
    a, b = np.linalg.lstsq(np.column_stack((s, np.ones(len(s)))), p[:, 2], rcond=None)[0]
    return dict(valid=True, slope=float(a), tilt_deg=math.degrees(math.atan(a)),
                cm_per_10m=float(1000*a), intercept_m=float(b),
                residual_rms_m=float(np.sqrt(np.mean((p[:, 2]-a*s-b)**2))))


def fit_plane(rows):
    # Spatial sampling prevents long stops from dominating the fitted plane.
    points = []
    for row in rows:
        p = np.asarray(row[2:5], dtype=float)
        if not points or np.linalg.norm(p[:2]-points[-1][:2]) >= 0.20:
            points.append(p)
    if len(points) < 20:
        return dict(valid=False, reason='Too few spatial samples')
    p = np.asarray(points)
    origin = p.mean(axis=0)
    d = p-origin
    singular = np.linalg.svd(d[:, :2], compute_uv=False)
    spread = singular / np.sqrt(len(p))
    if spread[1] < 1.0 or singular[1]/singular[0] < 0.1:
        return dict(valid=False, reason='Insufficient travel in a second direction',
                    horizontal_spread_m=spread.tolist())
    a, b = np.linalg.lstsq(d[:, :2], d[:, 2], rcond=None)[0]
    residual = d[:, 2] - a*d[:, 0] - b*d[:, 1]
    normal = np.array([-a, -b, 1.0])
    normal /= np.linalg.norm(normal)
    return dict(valid=True, spatial_samples=len(p), origin_xyz=origin.tolist(),
                equation='z-origin_z = a*(x-origin_x) + b*(y-origin_y)',
                a=float(a), b=float(b), normal_xyz=normal.tolist(),
                tilt_deg=math.degrees(math.atan(math.hypot(a, b))),
                horizontal_spread_m=spread.tolist(),
                vertical_residual_rms_m=float(np.sqrt(np.mean(residual**2))),
                vertical_residual_abs_p95_m=float(np.percentile(np.abs(residual), 95)),
                note='Fit includes turns. Training residual only; verify on independent travel. No correction applied.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=Path('/home/rei/forklift_ws/experiments/glim_out_back'))
    parser.add_argument('--route', choices=['out-back', 'plane'], default='out-back')
    parser.add_argument('--reference-report', type=Path, help='Validate against a frozen plane; do not refit')
    args = parser.parse_args()
    reference = None
    if args.reference_report:
        reference = json.loads(args.reference_report.read_text())['plane']
        if args.route != 'plane' or not reference.get('valid'):
            parser.error('Reference requires --route plane and a valid plane report')
    if not sys.stdin.isatty():
        parser.error('Запустите в интерактивном терминале: нужны отметки Enter.')
    folder = args.output_dir / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True, exist_ok=False)
    if reference:
        (folder / 'reference_plane.json').write_text(json.dumps(reference, indent=2))
    files = []

    def writer(name, header):
        f = (folder / name).open('w', newline='', buffering=1)
        files.append(f)
        w = csv.writer(f)
        w.writerow(header)
        return w

    odom = writer('odom.csv', ['receive_monotonic', 'stamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'frame', 'child'])
    imu = writer('imu.csv', ['receive_monotonic', 'stamp', 'ax', 'ay', 'az', 'wx', 'wy', 'wz'])
    clouds = writer('cloud_timing.csv', ['receive_monotonic', 'stamp', 'points'])
    rclpy.init()
    node = rclpy.create_node('glim_out_back_test')
    rows, marks = [], []
    recent_imu = deque(maxlen=2000)
    first = None
    previous_stamp = None
    invalid_stream = False
    labels = ['START', 'CORNER', 'FINISH'] if args.route == 'plane' else ['START', 'FAR', 'RETURN']

    def stamp(m):
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def on_odom(m):
        nonlocal first, previous_stamp, invalid_stream
        now = time.monotonic()
        p, q = m.pose.pose.position, m.pose.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in values):
            invalid_stream = True
            return
        t = stamp(m)
        if previous_stamp is not None and t <= previous_stamp:
            invalid_stream = True
        previous_stamp = t
        if rows and (m.header.frame_id, m.child_frame_id) != tuple(rows[0][-2:]):
            invalid_stream = True
        row = [now, t, *values, m.header.frame_id, m.child_frame_id]
        rows.append(row)
        odom.writerow(row)
        if first is None:
            first = now
            print('Одометрия поступает. Стойте 20 секунд, затем Enter для START.', flush=True)

    def on_imu(m):
        a, w = m.linear_acceleration, m.angular_velocity
        row = [time.monotonic(), stamp(m), a.x, a.y, a.z, w.x, w.y, w.z]
        imu.writerow(row)
        recent_imu.append(row)

    node.create_subscription(Odometry, '/glim_ros/odom', on_odom, qos_profile_sensor_data)
    node.create_subscription(Imu, '/livox/imu', on_imu, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/livox/points',
                             lambda m: clouds.writerow([time.monotonic(), stamp(m), m.width*m.height]),
                             qos_profile_sensor_data)
    print(f'Запись: {folder}\nEnter: {" → ".join(labels)}. Перед каждой отметкой стойте 5 секунд.\n'
          'q — закончить досрочно.', flush=True)
    try:
        while rclpy.ok() and len(marks) < 3:
            rclpy.spin_once(node, timeout_sec=0.05)
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            command = sys.stdin.readline()
            if not command or command.strip().lower() == 'q':
                break
            now = time.monotonic()
            window = [r for r in rows if r[0] >= now-5]
            if first is None or now-first < 20 or not window or now-window[-1][0] > 0.5 or len(window) < 35 or window[-1][0]-window[0][0] < 4:
                print('Нужны 20 с данных и свежая полная запись последних 5 с.', flush=True)
                continue
            pts = np.asarray([r[2:5] for r in window])
            im = [r for r in recent_imu if r[0] >= now-5]
            if not im or now-im[-1][0] > 0.5 or np.ptp(pts, axis=0).max() > 0.05 or max(np.linalg.norm(r[5:8]) for r in im) > 0.10:
                print('Не подтверждена остановка: стойте 5 с (размах позиции ≤5 см, гироскоп ≤0.10 рад/с).', flush=True)
                continue
            mark = dict(label=labels[len(marks)], receive_monotonic=now,
                        position_mean=pts.mean(axis=0).tolist(), position_std=pts.std(axis=0).tolist())
            marks.append(mark)
            (folder / 'markers.json').write_text(json.dumps(marks, indent=2))
            print(f'{mark["label"]}: {mark["position_mean"]}', flush=True)
            if len(marks) == 1:
                if args.route == 'plane':
                    print('Вернитесь к месту поворота, стойте 5 с и нажмите Enter.' if reference else
                          'Проедьте 15–30 м, плавно поверните примерно на 90°, остановитесь на 5 с и нажмите Enter.', flush=True)
                else:
                    print('Езжайте вперед 20–30 м, остановитесь на 5 с и нажмите Enter.', flush=True)
            elif len(marks) == 2:
                if args.route == 'plane':
                    print('Продолжите обратный маршрут к исходной области, стойте 5 с и нажмите Enter.' if reference else
                          'Проедьте 15–30 м в новом направлении, стойте 5 с и нажмите Enter.', flush=True)
                else:
                    print('Вернитесь задним ходом к старту, стойте 5 с и нажмите Enter.', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        report = dict(complete=len(marks) == 3, stream_invalid=invalid_stream,
                      samples=len(rows), markers=marks, route=args.route,
                      note='Odometry frame tilt test, not LiDAR-IMU extrinsic calibration. No parameters changed.')
        if len(marks) == 3 and args.route == 'plane':
            segment = [r for r in rows if marks[0]['receive_monotonic'] <= r[0] <= marks[-1]['receive_monotonic']]
            if reference:
                origin = np.asarray(reference['origin_xyz'])
                normal = np.asarray(reference['normal_xyz'])
                points = []
                for row in segment:
                    p = np.asarray(row[2:5])
                    if not points or np.linalg.norm(p[:2]-points[-1][:2]) >= 0.2:
                        points.append(p)
                p = np.asarray(points)
                heights = (p-origin) @ normal
                start = np.asarray(marks[0]['position_mean'])
                end = np.asarray(marks[-1]['position_mean'])
                dz = p[:, 2]-start[2]
                corrected_dz = (p-start) @ normal
                report['validation'] = dict(
                    valid=not invalid_stream and len(points)>=20,
                    reference_report=str(args.reference_report.resolve()), spatial_samples=len(points),
                    fixed_plane_rms_m=float(np.sqrt(np.mean(heights**2))),
                    fixed_plane_abs_p95_m=float(np.percentile(np.abs(heights),95)),
                    raw_height_change_rms_m=float(np.sqrt(np.mean(dz**2))),
                    corrected_height_change_rms_m=float(np.sqrt(np.mean(corrected_dz**2))),
                    endpoint_raw_dz_m=float(end[2]-start[2]),
                    endpoint_corrected_dz_m=float((end-start) @ normal),
                    marker_plane_distances_m=[float((np.asarray(m['position_mean'])-origin) @ normal) for m in marks],
                    note='Frozen normal and origin. No refitting. Relative changes use measured start. Includes turns.')
            else:
                report['plane'] = fit_plane(segment)
            if invalid_stream and not reference:
                report['plane']['valid'] = False
                report['plane']['reason'] = 'Invalid odometry stream; repeat test'
        if len(marks) == 3 and args.route == 'out-back':
            p0, p1, p2 = np.asarray([m['position_mean'] for m in marks])
            distance = float(np.linalg.norm((p1-p0)[:2]))
            report.update(distance_xy_m=distance, endpoint_dz_m=float(p1[2]-p0[2]),
                          return_error_xyz_m=(p2-p0).tolist())
            if distance >= 2:
                direction = (p1-p0)[:2]/distance
                for name, i in [('outbound', 0), ('inbound', 1)]:
                    segment = [r for r in rows if marks[i]['receive_monotonic'] <= r[0] <= marks[i+1]['receive_monotonic']-5]
                    report[name] = fit_segment(segment, p0, direction)
        (folder/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False))
        for f in files:
            f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(json.dumps(report, indent=2), flush=True)
        print(f'Сохранено: {folder}')


if __name__ == '__main__':
    main()
