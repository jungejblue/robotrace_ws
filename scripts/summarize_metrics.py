#!/usr/bin/env python3
"""Summarize gpu_map_localizer metrics CSV using Python's standard library.

Latency summaries describe recorded processing attempts, not every input scan.
The first logged row is shown separately because first submap work can differ
from steady processing. CUDA context construction before CSV logging is not
measured here and must not be reported as end-to-end startup latency.
"""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
import statistics

REQUIRED_METRICS = ('preprocess_ms', 'registration_ms', 'validation_ms', 'total_ms')
OPTIONAL_METRICS = ('scan_age_s', 'result_age_s')


def quantile(values, probability):
    """Linear interpolation at (n-1)*p; return None for an empty sequence."""
    if not 0.0 <= probability <= 1.0:
        raise ValueError('quantile probability must be between 0 and 1')
    values = sorted(values)
    if not values:
        return None
    position = (len(values) - 1) * probability
    low, high = math.floor(position), math.ceil(position)
    return values[low] + (values[high] - values[low]) * (position - low)


def finite_number(value):
    try:
        number = float(value)
    except (ValueError, TypeError):
        return None
    return number if math.isfinite(number) else None


def metric_summary(rows, column):
    values = []
    invalid = 0
    for row in rows:
        value = finite_number(row.get(column))
        if value is None:
            invalid += 1
        else:
            values.append(value)
    return {
        'count': len(values), 'invalid_or_missing_count': invalid,
        'min': min(values) if values else None,
        'max': max(values) if values else None,
        'mean': statistics.mean(values) if values else None,
        'p50': quantile(values, 0.50), 'p95': quantile(values, 0.95),
        'p99': quantile(values, 0.99),
    }


def summarize(path):
    with Path(path).open(newline='') as source:
        reader = csv.DictReader(source)
        columns = reader.fieldnames or []
        rows = [row for row in reader if any(value for key, value in row.items() if key is not None)]
    state_counts = Counter((row.get('state') or 'UNSPECIFIED').strip() for row in rows)
    metrics = [column for column in REQUIRED_METRICS if column in columns]
    metrics.extend(column for column in OPTIONAL_METRICS if column in columns)
    warnings = []
    missing = [column for column in REQUIRED_METRICS if column not in columns]
    if missing:
        warnings.append('Missing expected timing columns: ' + ', '.join(missing))
    if 'result_age_s' not in columns:
        warnings.append('result_age_s unavailable; scan_age_s is age before processing, not age at completion.')
    if any(None in row for row in rows):
        warnings.append('Some CSV rows have extra fields; inspect CSV formatting and reason strings.')
    last_dropped = finite_number(rows[-1].get('dropped_scans')) if rows else None
    first = None
    if rows:
        first = {
            'stamp': finite_number(rows[0].get('stamp')),
            'state': rows[0].get('state'), 'reason': rows[0].get('reason'),
            'source_points': finite_number(rows[0].get('source_points')),
            'target_points': finite_number(rows[0].get('target_points')),
            'metrics': {column: finite_number(rows[0].get(column)) for column in metrics},
        }
    return {
        'input_csv': str(Path(path).resolve()),
        'row_count': len(rows), 'state_counts': dict(sorted(state_counts.items())),
        'tracking_fraction': state_counts.get('TRACKING', 0) / len(rows) if rows else None,
        'tracking_fraction_denominator': 'CSV-recorded processing attempts; not all input scans or wall time',
        'dropped_scans_last': int(last_dropped) if last_dropped is not None else None,
        'metric_units': {column: 'seconds' if column.endswith('_s') else 'milliseconds' for column in metrics},
        'all_rows': {column: metric_summary(rows, column) for column in metrics},
        'first_logged_row': first,
        'steady_rows_excluding_first': {column: metric_summary(rows[1:], column) for column in metrics},
        'steady_row_count': max(len(rows) - 1, 0),
        'warnings': warnings,
        'limitations': [
            'The first logged row is not total program initialization latency.',
            'CUDA context creation and map loading before CSV logging are omitted.',
            'CSV states count recorded attempts, not periods while waiting for GNSS or ROS inputs.',
            'Rejected and error rows are included in processing latency summaries.',
            'Percentiles use linear interpolation at position (n-1)*p; small sample tails are uncertain.',
            'CSV timings alone do not measure CPU utilization, memory, thermals, or absolute localization accuracy.',
        ],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path, help='gpu_map_localizer CSV metrics file')
    parser.add_argument('--output', type=Path, help='Write JSON summary here; otherwise print JSON')
    args = parser.parse_args(argv)
    if args.output and args.csv.resolve() == args.output.resolve():
        parser.error('JSON output must differ from input CSV')
    try:
        result = summarize(args.csv)
    except (OSError, csv.Error) as exc:
        parser.error(str(exc))
    rendered = json.dumps(result, indent=2, allow_nan=False) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered)
        print('Created metrics summary:', args.output)
    else:
        print(rendered, end='')


if __name__ == '__main__':
    main()
