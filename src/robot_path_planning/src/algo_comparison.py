#!/usr/bin/env python

"""
Algorithm comparison utilities for a UGV (Unmanned Ground Vehicle) using ROS.
Processes and analyzes run logs to compute statistics and comparisons between different algorithms.
"""

import argparse
import csv
import json
import math
import os

import rospkg


def resolve_package_path():
    try:
        return rospkg.RosPack().get_path('robot_path_planning')
    except Exception:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_runs(log_dir):
    runs = []
    if not os.path.isdir(log_dir):
        return runs

    for file_name in sorted(os.listdir(log_dir)):
        if not file_name.endswith('.json'):
            continue

        file_path = os.path.join(log_dir, file_name)
        with open(file_path, 'r') as input_file:
            try:
                runs.append(json.load(input_file))
            except ValueError:
                continue
    return runs


def parse_csv_values(value):
    return [item.strip() for item in str(value).split(',') if item.strip()]


def filter_runs_by_status(runs, include_statuses):
    if include_statuses is None:
        return runs
    include_set = set(include_statuses)
    return [run for run in runs if run.get('status') in include_set]


def choose_runs(runs, mode, include_statuses):
    grouped = {}
    for run in runs:
        grouped.setdefault(run.get('algo_name', 'unknown'), []).append(run)

    selected_rows = []
    for algo_name in sorted(grouped.keys()):
        algo_runs = sorted(grouped[algo_name], key=lambda item: item.get('finished_at_epoch', 0))
        filtered_runs = filter_runs_by_status(algo_runs, include_statuses)
        if not filtered_runs:
            continue

        if mode == 'latest':
            selected_rows.append(build_row_from_runs(filtered_runs[-1:]))
        else:
            selected_rows.append(build_row_from_runs(filtered_runs))
    return selected_rows


def clean_numeric(values):
    return [value for value in values if value is not None]


def average(values):
    cleaned = clean_numeric(values)
    if not cleaned:
        return None
    return sum(cleaned) / float(len(cleaned))


def max_value(values):
    cleaned = clean_numeric(values)
    if not cleaned:
        return None
    return max(cleaned)


def stddev_sample(values):
    cleaned = clean_numeric(values)
    count = len(cleaned)
    if count < 2:
        return None

    mean_value = sum(cleaned) / float(count)
    variance = sum((value - mean_value) ** 2 for value in cleaned) / float(count - 1)
    return math.sqrt(variance)


def ci95(values):
    cleaned = clean_numeric(values)
    count = len(cleaned)
    if count < 2:
        return None

    standard_deviation = stddev_sample(cleaned)
    return 1.96 * (standard_deviation / math.sqrt(float(count)))


def add_stats(row, field_name, values):
    row[field_name] = average(values)
    row[field_name + '_std'] = stddev_sample(values)
    row[field_name + '_ci95'] = ci95(values)


def build_row_from_runs(runs):
    latest_run = runs[-1]
    summaries = [run.get('summary', {}) for run in runs]

    row = {
        'algo_name': latest_run.get('algo_name', ''),
        'status': latest_run.get('status', ''),
        'sample_count': len(runs),
        'max_replan_latency_sec': max_value([summary.get('max_replan_latency_sec') for summary in summaries]),
        'peak_cpu_pct': max_value([summary.get('peak_cpu_pct') for summary in summaries]),
        'peak_cpu_capacity_pct': max_value([summary.get('peak_cpu_capacity_pct') for summary in summaries]),
        'peak_memory_mb': max_value([summary.get('peak_memory_mb') for summary in summaries]),
    }

    add_stats(row, 'travel_time_sec', [run.get('travel_time_sec') for run in runs])
    add_stats(row, 'travel_distance_m', [run.get('travel_distance_m') for run in runs])
    add_stats(row, 'planned_path_distance_m', [summary.get('planned_path_distance_m') for summary in summaries])
    add_stats(row, 'time_per_meter_sec', [summary.get('time_per_meter_sec') for summary in summaries])
    add_stats(row, 'avg_speed_mps', [summary.get('avg_speed_mps') for summary in summaries])
    add_stats(row, 'avg_replan_latency_sec', [summary.get('avg_replan_latency_sec') for summary in summaries])
    add_stats(row, 'avg_cpu_pct', [summary.get('avg_cpu_pct') for summary in summaries])
    add_stats(row, 'avg_cpu_capacity_pct', [summary.get('avg_cpu_capacity_pct') for summary in summaries])
    add_stats(row, 'avg_memory_mb', [summary.get('avg_memory_mb') for summary in summaries])
    add_stats(row, 'avg_process_coverage', [summary.get('avg_process_coverage') for summary in summaries])
    add_stats(row, 'path_updates', [summary.get('path_updates') for summary in summaries])
    add_stats(row, 'obstacle_events', [summary.get('obstacle_events') for summary in summaries])
    add_stats(row, 'replan_events', [summary.get('replan_events') for summary in summaries])

    return row


def format_value(value):
    if value is None:
        return ''
    return round(value, 4)


def write_outputs(log_dir, rows, include_statuses, mode):
    csv_path = os.path.join(log_dir, 'algo_comparison.csv')
    markdown_path = os.path.join(log_dir, 'algo_comparison.md')

    status_filter_text = 'all'
    if include_statuses is not None:
        status_filter_text = ','.join(include_statuses)

    columns = [
        'algo_name',
        'status',
        'sample_count',
        'travel_time_sec',
        'travel_time_sec_std',
        'travel_time_sec_ci95',
        'travel_distance_m',
        'travel_distance_m_std',
        'travel_distance_m_ci95',
        'planned_path_distance_m',
        'planned_path_distance_m_std',
        'planned_path_distance_m_ci95',
        'time_per_meter_sec',
        'time_per_meter_sec_std',
        'time_per_meter_sec_ci95',
        'avg_speed_mps',
        'avg_speed_mps_std',
        'avg_speed_mps_ci95',
        'avg_replan_latency_sec',
        'avg_replan_latency_sec_std',
        'avg_replan_latency_sec_ci95',
        'max_replan_latency_sec',
        'avg_cpu_pct',
        'avg_cpu_pct_std',
        'avg_cpu_pct_ci95',
        'peak_cpu_pct',
        'avg_cpu_capacity_pct',
        'avg_cpu_capacity_pct_std',
        'avg_cpu_capacity_pct_ci95',
        'peak_cpu_capacity_pct',
        'avg_memory_mb',
        'avg_memory_mb_std',
        'avg_memory_mb_ci95',
        'peak_memory_mb',
        'avg_process_coverage',
        'avg_process_coverage_std',
        'avg_process_coverage_ci95',
        'path_updates',
        'path_updates_std',
        'obstacle_events',
        'obstacle_events_std',
        'replan_events',
        'replan_events_std',
    ]

    with open(csv_path, 'wb') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(columns)
        for row in rows:
            writer.writerow([format_value(row.get(column)) if column not in ['algo_name', 'status', 'sample_count'] else row.get(column, '') for column in columns])

    with open(markdown_path, 'w') as markdown_file:
        markdown_file.write('# Planner Comparison\n\n')
        markdown_file.write('- mode: {0}\n'.format(mode))
        markdown_file.write('- included_statuses: {0}\n\n'.format(status_filter_text))
        markdown_file.write('| Algo | Runs | Sim Time Mean ± CI95 (s) | Distance Mean ± CI95 (m) | Avg Speed Mean ± CI95 (m/s) | Avg Replan Mean ± CI95 (s) | Avg CPU Mean ± CI95 (%) | Avg CPU Capacity Mean ± CI95 (%) | Avg RAM Mean ± CI95 (MB) |\n')
        markdown_file.write('| --- | --- | --- | --- | --- | --- | --- | --- | --- |\n')
        for row in rows:
            markdown_file.write('| {0} | {1} | {2} ± {3} | {4} ± {5} | {6} ± {7} | {8} ± {9} | {10} ± {11} | {12} ± {13} | {14} ± {15} |\n'.format(
                row['algo_name'],
                row['sample_count'],
                format_value(row['travel_time_sec']),
                format_value(row['travel_time_sec_ci95']),
                format_value(row['travel_distance_m']),
                format_value(row['travel_distance_m_ci95']),
                format_value(row['avg_speed_mps']),
                format_value(row['avg_speed_mps_ci95']),
                format_value(row['avg_replan_latency_sec']),
                format_value(row['avg_replan_latency_sec_ci95']),
                format_value(row['avg_cpu_pct']),
                format_value(row['avg_cpu_pct_ci95']),
                format_value(row['avg_cpu_capacity_pct']),
                format_value(row['avg_cpu_capacity_pct_ci95']),
                format_value(row['avg_memory_mb']),
                format_value(row['avg_memory_mb_ci95'])
            ))

    return csv_path, markdown_path


def main():
    parser = argparse.ArgumentParser(description='Create a planner comparison table from resource monitor logs')
    parser.add_argument('--mode', choices=['latest', 'average'], default='latest', help='Use only the latest run per algorithm or average all runs per algorithm')
    parser.add_argument('--include-status', default='reached', help='Comma-separated run statuses to include. Use all to include every status.')
    args = parser.parse_args()

    include_statuses = parse_csv_values(args.include_status)
    if not include_statuses or 'all' in include_statuses:
        include_statuses = None

    package_path = resolve_package_path()
    log_dir = os.path.join(package_path, 'resource_logs')

    rows = choose_runs(load_runs(log_dir), args.mode, include_statuses)
    csv_path, markdown_path = write_outputs(log_dir, rows, include_statuses, args.mode)

    print('Comparison CSV: {0}'.format(csv_path))
    print('Comparison Markdown: {0}'.format(markdown_path))


if __name__ == '__main__':
    main()