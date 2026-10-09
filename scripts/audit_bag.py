#!/usr/bin/env python3
"""Inspect Ouster inputs without changing a ROS bag or any localization setting.

Run in ROS Noetic. Pure PointCloud2 inspection functions also work without ROS.
Receipt times select the audit window; header times diagnose sensor clock domains.
"""
import argparse
import bisect
import json
import math
from pathlib import Path
import statistics
import struct

POINTS = '/ouster/points'
IMU = '/ouster/imu'
POINT_TYPES = {1: ('b', 1), 2: ('B', 1), 3: ('h', 2), 4: ('H', 2),
               5: ('i', 4), 6: ('I', 4), 7: ('f', 4), 8: ('d', 8)}
REQUIRED = {'x': 7, 'y': 7, 'z': 7, 'intensity': 7, 't': 6, 'ring': 4}


def field_decoders(msg):
    fields = {}
    prefix = '>' if msg.is_bigendian else '<'
    for field in msg.fields:
        if field.name in fields:
            raise ValueError('duplicate field: ' + field.name)
        if field.datatype not in POINT_TYPES or field.count != 1:
            if field.name in REQUIRED:
                raise ValueError('unsupported required field: ' + field.name)
            continue
        code, size = POINT_TYPES[field.datatype]
        if field.offset < 0 or field.offset + size > msg.point_step:
            raise ValueError('field outside point_step: ' + field.name)
        fields[field.name] = (field.offset, struct.Struct(prefix + code))
    if any(name not in fields for name in REQUIRED):
        raise ValueError('missing required fields: ' + ', '.join(
            name for name in REQUIRED if name not in fields))
    if msg.point_step <= 0 or msg.row_step < msg.width * msg.point_step:
        raise ValueError('invalid point_step/row_step')
    if len(msg.data) < msg.row_step * msg.height:
        raise ValueError('truncated PointCloud2 data')
    return fields


def inspect_cloud(msg, blind=4.0, stride=4, scan_rate=20.0, max_scan_duration=0.1):
    """Count finite points and emulate the configured preprocessor selection.

    This diagnoses input validity; it does not execute FAST-LIO deskew/voxelization.
    Row padding and both byte orders are respected. The t field is nanoseconds.
    """
    if blind < 0 or stride < 1 or scan_rate <= 0 or max_scan_duration <= 0:
        raise ValueError('invalid blind/stride/scan_rate/max_scan_duration')
    fields = field_decoders(msg)
    observed = {f.name: {'datatype': int(f.datatype), 'count': int(f.count),
                         'offset': int(f.offset)} for f in msg.fields}
    mismatches = {name: {'expected': dtype, 'observed': observed[name]['datatype']}
                  for name, dtype in REQUIRED.items()
                  if observed[name]['datatype'] != dtype}
    count = msg.width * msg.height
    finite = nonfinite = beyond = kept = 0
    times, kept_times, ranges = [], [], []
    invalid_rings = invalid_times = 0
    for row in range(msg.height):
        for column in range(msg.width):
            index = row * msg.width + column
            offset = row * msg.row_step + column * msg.point_step
            def value(name):
                position, decoder = fields[name]
                return decoder.unpack_from(msg.data, offset + position)[0]
            xyz = (value('x'), value('y'), value('z'))
            t = int(value('t'))
            times.append(t)
            ring_valid = 0 <= value('ring') < 64
            time_valid = 0 <= t <= max_scan_duration * 1e9
            invalid_rings += int(not ring_valid)
            invalid_times += int(not time_valid)
            if not all(math.isfinite(v) for v in xyz):
                nonfinite += 1
                continue
            finite += 1
            distance = math.sqrt(sum(v * v for v in xyz))
            ranges.append(distance)
            if distance <= blind:
                continue
            beyond += 1
            if ring_valid and time_valid and index % stride == 0:
                kept += 1
                kept_times.append(t)
    warnings = []
    if mismatches:
        warnings.append('Required field datatypes differ from the Ouster frontend.')
    if msg.is_bigendian:
        warnings.append('The Ouster frontend requires little-endian input; big-endian is decoded for inspection only.')
    if not kept:
        warnings.append('No points survive finite XYZ, blind, ring, relative t and point_filter_num filters.')
    if invalid_rings:
        warnings.append('Some ring values are outside the configured 64 channels.')
    if invalid_times:
        warnings.append('Relative point t exceeds max_scan_duration_s; verify driver timestamps.')
    if times and max(times) == 0:
        warnings.append('All per-point times are zero; motion compensation cannot use point timing.')
    return {
        'width': msg.width, 'height': msg.height, 'point_step': msg.point_step,
        'row_step': msg.row_step, 'bigendian': bool(msg.is_bigendian),
        'raw_points': count, 'finite_xyz_points': finite,
        'nonfinite_xyz_points': nonfinite, 'finite_points_outside_blind': beyond,
        'preprocess_candidate_points': kept, 'invalid_ring_points': invalid_rings,
        'invalid_relative_time_points': invalid_times,
        'frontend_schema_compatible': not mismatches and not msg.is_bigendian,
        'range_min_m': min(ranges) if ranges else None,
        'range_max_m': max(ranges) if ranges else None,
        'point_time_min_ms': min(times) / 1e6 if times else None,
        'point_time_max_ms': max(times) / 1e6 if times else None,
        'selected_point_time_max_ms': max(kept_times) / 1e6 if kept_times else None,
        'field_type_mismatches': mismatches, 'fields': observed, 'warnings': warnings,
    }


def timing_summary(samples):
    if not samples:
        return {'count': 0}
    receipts = [s[0] for s in samples]
    stamps = [s[1] for s in samples]
    differences = [b - a for a, b in zip(stamps, stamps[1:])]
    positive = [d for d in differences if d > 0]
    return {
        'count': len(samples), 'first_header_s': stamps[0],
        'last_header_s': stamps[-1], 'header_span_s': stamps[-1] - stamps[0],
        'backward_headers': sum(d < 0 for d in differences),
        'duplicate_headers': sum(d == 0 for d in differences),
        'median_period_s': statistics.median(positive) if positive else None,
        'median_receipt_minus_header_s': statistics.median(
            r - s for r, s in zip(receipts, stamps)),
    }


def audit(path, seconds=12.0, cloud_samples=3, blind=4.0, stride=4, scan_rate=20.0,
          max_scan_duration=0.1):
    try:
        import rosbag
        import rospy
    except ImportError as exc:
        raise RuntimeError('Run inside the ROS Noetic container: ' + str(exc)) from exc
    timing = {POINTS: [], IMU: []}
    clouds, acc_norms = [], []
    invalid_imu = 0
    with rosbag.Bag(str(path), 'r') as bag:
        first = bag.get_start_time()
        next_cloud = first
        for topic, msg, receipt in bag.read_messages(
                topics=list(timing), start_time=rospy.Time.from_sec(first),
                end_time=rospy.Time.from_sec(first + seconds)):
            record = receipt.to_sec()
            stamp = msg.header.stamp.to_sec()
            timing[topic].append((record, stamp))
            if topic == POINTS and record >= next_cloud and len(clouds) < cloud_samples:
                item = inspect_cloud(msg, blind, stride, scan_rate, max_scan_duration)
                item.update(header_s=stamp, receipt_s=record, elapsed_bag_s=record-first)
                clouds.append(item)
                next_cloud = record + seconds / cloud_samples
            elif topic == IMU:
                acc = msg.linear_acceleration
                gyro = msg.angular_velocity
                values = (acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z)
                if not all(math.isfinite(v) for v in values):
                    invalid_imu += 1
                else:
                    norm = math.sqrt(acc.x**2 + acc.y**2 + acc.z**2)
                    acc_norms.append(norm)
                    invalid_imu += int(norm < 1e-6)
    imu_stamps = sorted(item[1] for item in timing[IMU])
    for cloud in clouds:
        end = cloud['header_s'] + (cloud['selected_point_time_max_ms'] or 0.0) / 1000.0
        cloud['imu_samples_in_scan_interval'] = (
            bisect.bisect_right(imu_stamps, end) -
            bisect.bisect_left(imu_stamps, cloud['header_s']))
    summaries = {topic: timing_summary(rows) for topic, rows in timing.items()}
    warnings = []
    for topic, item in summaries.items():
        if not item['count']:
            warnings.append(topic + ' has no messages in the selected window.')
        elif item['backward_headers']:
            warnings.append(topic + ' contains backward header stamps; restart replay nodes.')
    if all(summaries[t]['count'] for t in timing):
        offset_difference = (summaries[POINTS]['median_receipt_minus_header_s'] -
                             summaries[IMU]['median_receipt_minus_header_s'])
        if abs(offset_difference) > 0.08:
            warnings.append('LiDAR/IMU header-to-receipt offsets differ by over 80 ms.')
    else:
        offset_difference = None
    if invalid_imu:
        warnings.append('Non-finite IMU values or near-zero acceleration norm detected.')
    if imu_stamps and clouds and not any(c['imu_samples_in_scan_interval'] for c in clouds):
        warnings.append('No sampled LiDAR scan interval overlaps IMU headers; inspect clock domains/t.')
    return {
        'input_bag': str(Path(path).resolve()), 'window_seconds': seconds,
        'settings': {'blind_m': blind, 'point_filter_num': stride,
                     'scan_rate_hz': scan_rate, 'timestamp_unit': 'nanoseconds',
                     'max_scan_duration_s': max_scan_duration},
        'topic_timing': summaries, 'lidar_minus_imu_receipt_offset_s': offset_difference,
        'imu_acceleration_norm_median': statistics.median(acc_norms) if acc_norms else None,
        'imu_invalid_or_zero_acceleration_messages': invalid_imu,
        'sampled_clouds': clouds, 'warnings': warnings,
        'limitations': [
            'Offline inputs are inspected; callback delivery and actual IMU sample consumption are not measured.',
            'Candidate point counts are before FAST-LIO deskew and voxelization; incompatible schema is rejected by the frontend.',
            'Receipt-time differences do not calibrate hardware synchronization or sensor transport latency.',
            'Only a bounded input window and a few clouds are inspected.',
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--seconds', type=float, default=12.0)
    parser.add_argument('--cloud-samples', type=int, default=3)
    parser.add_argument('--blind', type=float, default=4.0)
    parser.add_argument('--point-filter-num', type=int, default=4)
    parser.add_argument('--scan-rate', type=float, default=20.0)
    parser.add_argument('--max-scan-duration', type=float, default=0.1)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if (not args.bag.is_file() or args.seconds <= 0 or args.cloud_samples < 1 or
            args.blind < 0 or args.point_filter_num < 1 or args.scan_rate <= 0 or
            args.max_scan_duration <= 0):
        parser.error('Check bag path and positive audit settings.')
    if args.output and args.output.resolve() == args.bag.resolve():
        parser.error('Output must differ from the source bag.')
    try:
        report = audit(args.bag, args.seconds, args.cloud_samples, args.blind,
                       args.point_filter_num, args.scan_rate, args.max_scan_duration)
    except (RuntimeError, ValueError, OSError) as exc:
        parser.exit(1, 'Audit failed: ' + str(exc) + '\n')
    result = json.dumps(report, indent=2, allow_nan=False) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(result)
        print('Created input audit:', args.output)
    else:
        print(result, end='')


if __name__ == '__main__':
    main()
