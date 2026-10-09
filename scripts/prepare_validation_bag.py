#!/usr/bin/env python3
"""Create a filtered ROS1 validation bag with one constant Ouster clock shift.

Run inside ROS Noetic. This is an offline clock-domain normalization helper,
not hardware synchronization or latency calibration. The source bag and PCD
are never changed. Every LiDAR point and its relative `t` field is preserved.
"""
import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import tempfile

POINTS = '/ouster/points'
IMU = '/ouster/imu'
REAR = '/ublox_gps_rear/navpvt'
FRONT = '/ublox_gps_front/navpvt'
GNSS_TOPICS = (REAR, FRONT)
TIMING_TOPICS = (POINTS, IMU, REAR, FRONT)
DEFAULT_TOPICS = TIMING_TOPICS + ('/tf_static',)
NS = 1000000000
EPOCH_BOUNDARY_NS = NS * 1000000000


def navpvt_utc_ns(msg):
    """UTC measurement time, including signed subsecond nano; never receipt time.

    u-blox valid bits: validDate, validTime, fullyResolved. Leap seconds cannot
    be represented by Python datetime and are rejected rather than guessed.
    """
    if (int(msg.valid) & 0x07) != 0x07:
        raise ValueError('NavPVT UTC is not valid and fully resolved')
    try:
        utc = datetime(int(msg.year), int(msg.month), int(msg.day),
                       int(msg.hour), int(msg.min), int(msg.sec), tzinfo=timezone.utc)
    except (ValueError, OverflowError) as exc:
        raise ValueError('Invalid UTC calendar or unsupported leap second') from exc
    nano = int(msg.nano)
    if abs(nano) > NS:
        raise ValueError('NavPVT nano outside signed one-second range')
    return int(utc.timestamp()) * NS + nano


def percentile(values, percent):
    values = sorted(values)
    if not values:
        return None
    position = (len(values) - 1) * percent / 100.0
    low = int(math.floor(position))
    high = int(math.ceil(position))
    return values[low] + (values[high] - values[low]) * (position - low)


class TimingStats:
    def __init__(self, window):
        self.window = window
        self.first_offsets = []
        self.last_offsets = deque(maxlen=window)
        self.count = 0
        self.invalid = 0
        self.backward = 0
        self.duplicates = 0
        self.first_stamp_ns = None
        self.last_stamp_ns = None
        self.offset_min = None
        self.offset_max = None

    def add(self, receipt_ns, stamp_ns):
        if stamp_ns <= 0:
            self.invalid += 1
            return
        if self.last_stamp_ns is not None:
            self.backward += int(stamp_ns < self.last_stamp_ns)
            self.duplicates += int(stamp_ns == self.last_stamp_ns)
        offset = (receipt_ns - stamp_ns) / NS
        if len(self.first_offsets) < self.window:
            self.first_offsets.append(offset)
        self.last_offsets.append(offset)
        self.offset_min = offset if self.offset_min is None else min(self.offset_min, offset)
        self.offset_max = offset if self.offset_max is None else max(self.offset_max, offset)
        if self.first_stamp_ns is None:
            self.first_stamp_ns = stamp_ns
        self.last_stamp_ns = stamp_ns
        self.count += 1

    def describe(self):
        if not self.count:
            return {'count': 0, 'invalid_timestamps': self.invalid}
        first_median = statistics.median(self.first_offsets)
        last_median = statistics.median(self.last_offsets)
        residuals = [abs(offset - first_median)
                     for offset in self.first_offsets + list(self.last_offsets)]
        return {
            'count': self.count,
            'invalid_timestamps': self.invalid,
            'backward_timestamps': self.backward,
            'duplicate_timestamps': self.duplicates,
            'first_stamp_seconds': self.first_stamp_ns / NS,
            'last_stamp_seconds': self.last_stamp_ns / NS,
            'first_receipt_minus_measurement_median_seconds': first_median,
            'last_receipt_minus_measurement_median_seconds': last_median,
            'start_to_end_offset_change_seconds': last_median - first_median,
            'offset_min_seconds': self.offset_min,
            'offset_max_seconds': self.offset_max,
            'sample_absolute_residual_p95_seconds': percentile(residuals, 95),
            'sample_absolute_residual_max_seconds': max(residuals),
        }


def choose_offset(stats, max_sensor_difference, max_drift, correct_gnss_latency=True):
    """Validate clock domains and return a single integer-nanosecond offset."""
    summaries = {topic: sample.describe() for topic, sample in stats.items()}
    warnings = []
    for topic in TIMING_TOPICS:
        sample = stats[topic]
        if sample.count < 10:
            raise ValueError('{} needs at least 10 valid measurement timestamps'.format(topic))
        if sample.backward:
            raise ValueError('{} has {} backward timestamps; constant shift cannot repair a reset'.format(
                topic, sample.backward))
        if topic in (POINTS, IMU) and sample.invalid:
            raise ValueError('{} has invalid/zero timestamps'.format(topic))
        if sample.duplicates:
            warnings.append('{} has {} duplicate measurement timestamps'.format(topic, sample.duplicates))
        if sample.invalid:
            warnings.append('{} has {} invalid UTC timestamps (GNSS messages are preserved)'.format(
                topic, sample.invalid))
        drift = summaries[topic]['start_to_end_offset_change_seconds']
        if abs(drift) > max_drift:
            raise ValueError('{} receipt-minus-measurement changed by {:.6f}s, limit {:.6f}s; '
                             'inspect drift, transport latency and clock resets'.format(topic, drift, max_drift))
        if summaries[topic]['sample_absolute_residual_p95_seconds'] > max_drift:
            warnings.append('{} receipt jitter exceeds drift threshold; synchronization is uncertain'.format(topic))

    point_med = statistics.median(stats[POINTS].first_offsets)
    imu_med = statistics.median(stats[IMU].first_offsets)
    if abs(point_med - imu_med) > max_sensor_difference:
        raise ValueError('LiDAR/IMU estimated offsets differ by {:.6f}s, limit {:.6f}s; '
                         'they may not share a clock'.format(abs(point_med - imu_med), max_sensor_difference))
    epoch_points = stats[POINTS].first_stamp_ns >= EPOCH_BOUNDARY_NS
    epoch_imu = stats[IMU].first_stamp_ns >= EPOCH_BOUNDARY_NS
    if epoch_points != epoch_imu:
        raise ValueError('LiDAR and IMU use different timestamp domains')
    if epoch_points:
        if max(abs(point_med), abs(imu_med)) > 10.0:
            raise ValueError('Ouster already has epoch timestamps but is >10s from bag clock; '
                             'do not automatically reinterpret its clock')
        offset_ns = 0
        source = 'already_epoch_no_shift'
    else:
        gnss_offsets = stats[REAR].first_offsets + stats[FRONT].first_offsets
        gnss_med = statistics.median(gnss_offsets)
        if abs(gnss_med) > 10.0:
            raise ValueError('GNSS UTC is >10s from bag receipt clock; receipt anchoring is unsafe')
        delay = gnss_med if correct_gnss_latency else 0.0
        offset_ns = int(round((imu_med - delay) * NS))
        source = 'median_imu_receipt_minus_measurement'
        if correct_gnss_latency:
            source += '_minus_median_gnss_receipt_delay'
        warnings.append('Constant offset inferred from receipt times assumes stable transport delays; '
                        'absolute synchronization error is not calibrated.')
    rear_med = statistics.median(stats[REAR].first_offsets)
    front_med = statistics.median(stats[FRONT].first_offsets)
    if abs(rear_med - front_med) > max_sensor_difference:
        warnings.append('Rear/front GNSS receipt delays differ; use UTC/iTOW to pair receivers, not receipt time.')
    return offset_ns, source, summaries, warnings


def should_shift_static_stamp(stamp_ns, sensor_first_ns, sensor_last_ns, offset_ns):
    """Only shift a positive static stamp plausibly in the same device domain."""
    if not offset_ns or stamp_ns <= 0 or stamp_ns >= EPOCH_BOUNDARY_NS:
        return False
    pad = 10 * NS
    return sensor_first_ns - pad <= stamp_ns <= sensor_last_ns + pad


def audit_bag(path, rosbag_module, sample_window):
    stats = {topic: TimingStats(sample_window) for topic in TIMING_TOPICS}
    with rosbag_module.Bag(str(path), 'r') as bag:
        for topic, msg, receipt in bag.read_messages(topics=list(TIMING_TOPICS)):
            try:
                stamp_ns = msg.header.stamp.to_nsec() if topic in (POINTS, IMU) else navpvt_utc_ns(msg)
            except (AttributeError, TypeError, ValueError, OverflowError):
                stats[topic].invalid += 1
                continue
            stats[topic].add(receipt.to_nsec(), stamp_ns)
    return stats


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path, help='Original validation bag; never modified')
    parser.add_argument('output', type=Path, help='New filtered output bag; must not exist')
    parser.add_argument('--sample-window', type=int, default=200,
                        help='First/last valid timestamps retained for each topic (default: 200)')
    parser.add_argument('--max-sensor-offset-difference', type=float, default=0.08,
                        help='Maximum median LiDAR/IMU clock-offset difference in seconds')
    parser.add_argument('--max-drift', type=float, default=0.08,
                        help='Maximum first-to-last median offset change in seconds')
    parser.add_argument('--include-metadata', action='store_true', help='Also preserve /ouster/metadata')
    parser.add_argument('--no-gnss-latency-correction', action='store_true',
                        help='Anchor device clock to bag receipt, without median GNSS delay subtraction')
    args = parser.parse_args(argv)
    if args.sample_window < 10 or args.max_drift <= 0 or args.max_sensor_offset_difference <= 0:
        parser.error('sample-window must be >=10 and thresholds must be positive')
    if not args.input.is_file():
        parser.error('Input bag does not exist')
    report_path = args.output.with_suffix('.time_alignment.json')
    if args.input.resolve() == args.output.resolve() or args.output.exists() or report_path.exists():
        parser.error('Output bag and report must be new files distinct from input')
    # ROS imports are lazy so pure clock logic can be tested without a ROS install.
    try:
        import rosbag
        import rospy
    except ImportError as exc:
        parser.error('Run inside the Noetic container after sourcing ROS: {}'.format(exc))
    print('Auditing selected topics across the entire validation bag...', flush=True)
    try:
        stats = audit_bag(args.input, rosbag, args.sample_window)
        offset_ns, source, summaries, warnings = choose_offset(
            stats, args.max_sensor_offset_difference, args.max_drift,
            not args.no_gnss_latency_correction)
    except (ValueError, rosbag.bag.ROSBagException) as exc:
        raise SystemExit('No output written: {}'.format(exc))
    print('Common LiDAR/IMU header shift [s]: {:.9f}'.format(offset_ns / NS), flush=True)
    for warning in warnings:
        print('WARNING:', warning, flush=True)
    topics = list(DEFAULT_TOPICS)
    if args.include_metadata:
        topics.append('/ouster/metadata')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = tempfile.NamedTemporaryFile(prefix=args.output.name + '.', suffix='.partial',
                                            dir=str(args.output.parent), delete=False)
    temporary_path = Path(temporary.name)
    temporary.close()
    counts = {topic: 0 for topic in topics}
    shifted_static = 0
    report_created = False
    output_committed = False
    try:
        with rosbag.Bag(str(args.input), 'r') as src, rosbag.Bag(str(temporary_path), 'w') as dst:
            for topic, msg, receipt, connection in src.read_messages(
                    topics=topics, return_connection_header=True):
                if topic in (POINTS, IMU):
                    shifted_ns = msg.header.stamp.to_nsec() + offset_ns
                    if shifted_ns <= 0:
                        raise ValueError('Normalized timestamp is not positive')
                    msg.header.stamp = rospy.Time(shifted_ns // NS, shifted_ns % NS)
                elif topic == '/tf_static':
                    for transform in msg.transforms:
                        stamp_ns = transform.header.stamp.to_nsec()
                        if should_shift_static_stamp(stamp_ns, stats[IMU].first_stamp_ns,
                                                     stats[IMU].last_stamp_ns, offset_ns):
                            shifted_ns = stamp_ns + offset_ns
                            transform.header.stamp = rospy.Time(shifted_ns // NS, shifted_ns % NS)
                            shifted_static += 1
                # NavPVT measurement fields, LiDAR data/fields/t, connection metadata
                # (including tf_static latching), and bag record times are untouched.
                dst.write(topic, msg, receipt, connection_header=connection)
                counts[topic] += 1
                if sum(counts.values()) % 10000 == 0:
                    print('Written selected messages:', sum(counts.values()), flush=True)
        report = {
            'input': str(args.input.resolve()), 'output': str(args.output.resolve()),
            'ouster_offset_nanoseconds': offset_ns, 'ouster_offset_seconds': offset_ns / NS,
            'offset_source': source, 'topics_written': counts,
            'static_transform_stamps_shifted': shifted_static,
            'timing_audit': summaries, 'warnings': warnings,
            'thresholds': {'max_sensor_offset_difference_seconds': args.max_sensor_offset_difference,
                           'max_start_to_end_offset_change_seconds': args.max_drift},
            'preserved': ['source bag', 'prior PCD', 'LiDAR point data/fields/relative t',
                          'Ouster inter-message header intervals', 'NavPVT UTC and iTOW',
                          'bag record times', 'connection headers/latching'],
            'limitations': [
                'Clock-domain normalization estimates a constant epoch offset from receipt times.',
                'Different sensor transport delays and hardware clock drift are not calibrated.',
                'First/last-window agreement cannot exclude drift changes inside the recording.',
                'Initialization should be stationary until valid dual RTK and IMU initialization are ready.',
                'GNSS UTC validity audit is independent of RTK FIX gating in the localization node.',
                'This bag enables ROS /clock playback; it does not establish live hardware synchronization.',
            ],
        }
        # Exclusive creation protects existing files even if a concurrent process
        # creates them after the initial check. Partial output is removed on error.
        with report_path.open('x') as output_report:
            report_created = True
            json.dump(report, output_report, indent=2)
            output_report.write('\n')
        import os
        os.link(str(temporary_path), str(args.output))
        output_committed = True
        temporary_path.unlink()
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        if report_created and not output_committed:
            report_path.unlink(missing_ok=True)
        raise
    print('Created:', args.output, flush=True)
    print('Timing report:', report_path, flush=True)
    print('Replay only this filtered validation bag with rosbag play --clock.', flush=True)


if __name__ == '__main__':
    main()
