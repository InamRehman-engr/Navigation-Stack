#!/usr/bin/env python

"""
This script monitors the resources and performance of the robot during navigation.
It tracks the robot's pose, goals, paths, obstacles, and system resource usage.
"""

import csv
import json
import math
import os
import time

import rospy
import rosgraph
import rospkg
import xmlrpclib
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan


def resolve_package_path():
	try:
		return rospkg.RosPack().get_path('robot_path_planning')
	except Exception:
		return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class ResourceMonitor(object):
	def __init__(self):
		rospy.init_node('resource_monitor', anonymous=False)

		self.algo_name = rospy.get_param('~algo_name', 'unknown')
		self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.35)
		self.obstacle_distance_threshold = rospy.get_param('~obstacle_distance_threshold', 0.90)
		self.clear_distance_threshold = rospy.get_param('~clear_distance_threshold', 1.10)
		self.sample_period = rospy.get_param('~sample_period', 1.0)
		self.front_sector_degrees = rospy.get_param('~front_sector_degrees', 30.0)
		self.min_motion_segment_m = rospy.get_param('~min_motion_segment_m', 0.01)
		self.max_motion_speed_mps = rospy.get_param('~max_motion_speed_mps', 3.0)
		self.path_topics = self._parse_csv_param(rospy.get_param('~path_topics', ''))
		self.tracked_nodes = self._parse_csv_param(rospy.get_param('~tracked_nodes', ''))

		package_path = resolve_package_path()
		self.log_dir = os.path.join(package_path, 'resource_logs')
		if not os.path.isdir(self.log_dir):
			os.makedirs(self.log_dir)

		self.summary_csv_path = os.path.join(self.log_dir, 'run_metrics.csv')
		self.master = rosgraph.Master(rospy.get_name())
		self.clock_ticks = float(os.sysconf(os.sysconf_names['SC_CLK_TCK']))
		self.page_size = float(os.sysconf(os.sysconf_names['SC_PAGE_SIZE']))
		cpu_count = os.sysconf(os.sysconf_names['SC_NPROCESSORS_ONLN'])
		self.cpu_core_count = float(cpu_count) if cpu_count > 0 else 1.0

		self.current_pose = None
		self.current_pose_source = None
		self.current_goal = None
		self.goal_active = False
		self.goal_start_wall = None
		self.goal_start_sim = None
		self.goal_index = 0
		self.last_path_update = None
		self.last_path_distance = None
		self.travel_distance_m = 0.0
		self.last_motion_pose = None
		self.last_motion_time = None
		self.last_path_signature = None

		self.path_update_events = []
		self.obstacle_events = []
		self.resource_samples = []
		self.pid_cache = {}
		self.prev_proc_cpu = {}

		rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
		rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, self.poseupdate_callback)
		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.poseupdate_callback)
		rospy.Subscriber('/odom', Odometry, self.odom_callback)
		rospy.Subscriber('/scan', LaserScan, self.scan_callback)

		self.path_subscribers = []
		for topic_name in self.path_topics:
			self.path_subscribers.append(rospy.Subscriber(topic_name, Path, self.path_callback, callback_args=topic_name))

		self.sample_timer = rospy.Timer(rospy.Duration(self.sample_period), self.sample_resources)
		rospy.on_shutdown(self.handle_shutdown)

		rospy.loginfo('Resource monitor started for %s.', self.algo_name)

	def _parse_csv_param(self, value):
		if isinstance(value, list):
			return [item for item in value if item]
		return [item.strip() for item in str(value).split(',') if item.strip()]

	def goal_callback(self, msg):
		if self.goal_active:
			self.finalize_run('interrupted_new_goal')

		self.goal_index += 1
		self.current_goal = msg.pose.position
		self.goal_active = True
		self.goal_start_wall = time.time()
		self.goal_start_sim = self.get_current_sim_time()
		self.last_path_update = None
		self.last_path_distance = None
		self.travel_distance_m = 0.0
		self.last_motion_pose = self.copy_position(self.current_pose)
		self.last_motion_time = self.get_current_sim_time()
		self.last_path_signature = None
		self.path_update_events = []
		self.obstacle_events = []
		self.resource_samples = []
		self.prev_proc_cpu = {}
		rospy.loginfo('Monitoring run %s for %s.', self.goal_index, self.algo_name)

	def poseupdate_callback(self, msg):
		self.update_pose(msg.pose.pose, 'map', msg.header.stamp.to_sec())
		self.current_pose_source = 'map'
		self.check_goal_reached()

	def odom_callback(self, msg):
		if self.current_pose_source == 'map':
			return
		self.update_pose(msg.pose.pose, 'odom', msg.header.stamp.to_sec())
		self.current_pose_source = 'odom'
		self.check_goal_reached()

	def update_pose(self, pose, source_name, pose_time_sec=None):
		self.current_pose = pose
		self.current_pose_source = source_name

		if not self.goal_active:
			return

		current_position = self.copy_position(pose)
		if current_position is None:
			return

		if self.last_motion_pose is not None:
			segment_distance = self.compute_distance_between_positions(self.last_motion_pose, current_position)
			segment_speed = None
			if pose_time_sec is not None and self.last_motion_time is not None:
				delta_t = pose_time_sec - self.last_motion_time
				if delta_t > 0.0:
					segment_speed = segment_distance / delta_t

			if segment_distance >= self.min_motion_segment_m:
				if segment_speed is None or segment_speed <= self.max_motion_speed_mps:
					self.travel_distance_m += segment_distance

		self.last_motion_pose = current_position
		if pose_time_sec is not None:
			self.last_motion_time = pose_time_sec

	def check_goal_reached(self):
		if not self.goal_active or self.current_goal is None or self.current_pose is None:
			return

		current_sim_time = self.get_current_sim_time()
		if self.goal_start_sim is None or current_sim_time is None:
			return

		if current_sim_time - self.goal_start_sim < 1.0:
			return

		dx = self.current_goal.x - self.current_pose.position.x
		dy = self.current_goal.y - self.current_pose.position.y
		distance = math.hypot(dx, dy)

		if distance <= self.goal_tolerance:
			self.finalize_run('reached')

	def scan_callback(self, msg):
		if not self.goal_active:
			return

		event_time = self.get_current_sim_time()
		if event_time is None:
			return

		min_front = self.compute_front_min_distance(msg)
		if min_front is None:
			return

		if min_front <= self.obstacle_distance_threshold:
			if not self.obstacle_events or self.obstacle_events[-1].get('resolved', True):
				self.obstacle_events.append({
					'detected_at': event_time,
					'front_distance': min_front,
					'baseline_path_signature': self.last_path_signature,
					'replan_latency_sec': None,
					'path_topic': None,
					'front_distance_cleared': None,
					'cleared_at': None,
					'resolve_reason': None,
					'resolved': False,
				})
				rospy.loginfo('Obstacle event recorded for {0} at {1:.2f} m.'.format(self.algo_name, min_front))
		elif self.obstacle_events and not self.obstacle_events[-1].get('resolved', True):
			if min_front >= self.clear_distance_threshold:
				self.obstacle_events[-1]['front_distance_cleared'] = min_front
				self.obstacle_events[-1]['cleared_at'] = event_time
				self.obstacle_events[-1]['resolve_reason'] = 'cleared_without_replan'
				self.obstacle_events[-1]['resolved'] = True

	def compute_front_min_distance(self, scan_msg):
		front_values = []
		half_sector = math.radians(self.front_sector_degrees)

		for index, distance in enumerate(scan_msg.ranges):
			if math.isnan(distance) or math.isinf(distance):
				continue
			angle = scan_msg.angle_min + (index * scan_msg.angle_increment)
			if -half_sector <= angle <= half_sector:
				front_values.append(distance)

		if not front_values:
			return None
		return min(front_values)

	def path_callback(self, msg, topic_name):
		if not self.goal_active:
			return

		event_time = self.get_current_sim_time()
		path_signature = self.compute_path_signature(msg)
		self.last_path_update = event_time
		path_distance = self.compute_path_distance(msg)
		self.last_path_distance = path_distance
		self.path_update_events.append({
			'topic': topic_name,
			'received_at_sim_sec': event_time,
			'path_points': len(msg.poses),
			'path_distance_m': path_distance,
			'path_signature': path_signature,
		})

		if self.obstacle_events and not self.obstacle_events[-1].get('resolved', True):
			obstacle_event = self.obstacle_events[-1]
			baseline_signature = obstacle_event.get('baseline_path_signature')
			if baseline_signature is None or path_signature != baseline_signature:
				obstacle_event['replan_latency_sec'] = event_time - obstacle_event['detected_at']
				obstacle_event['path_topic'] = topic_name
				obstacle_event['path_points'] = len(msg.poses)
				obstacle_event['path_distance_m'] = path_distance
				obstacle_event['resolve_reason'] = 'replanned'
				obstacle_event['resolved'] = True
				rospy.loginfo('Replan event for {0} observed on {1} after {2:.3f} s.'.format(self.algo_name, topic_name, obstacle_event['replan_latency_sec']))

		self.last_path_signature = path_signature

	def sample_resources(self, _event):
		if not self.goal_active:
			return

		sample_time = time.time()
		total_cpu = 0.0
		total_rss = 0.0
		active_processes = 0

		for node_name in self.tracked_nodes:
			pid = self.resolve_pid(node_name)
			if pid is None:
				continue

			proc_sample = self.read_process_sample(pid, sample_time)
			if proc_sample is None:
				continue

			total_cpu += proc_sample['cpu_pct']
			total_rss += proc_sample['rss_mb']
			active_processes += 1

		tracked_count = len(self.tracked_nodes)
		coverage = 1.0
		if tracked_count > 0:
			coverage = float(active_processes) / float(tracked_count)

		total_cpu_capacity_pct = total_cpu / self.cpu_core_count
		self.resource_samples.append({
			'time': sample_time,
			'sim_time_sec': self.get_current_sim_time(),
			'cpu_pct': total_cpu,
			'cpu_capacity_pct': total_cpu_capacity_pct,
			'rss_mb': total_rss,
			'process_count': active_processes,
			'tracked_process_count': tracked_count,
			'process_coverage': coverage,
		})

	def resolve_pid(self, node_name):
		cached_pid = self.pid_cache.get(node_name)
		if cached_pid and os.path.exists('/proc/{0}'.format(cached_pid)):
			return cached_pid

		try:
			uri = self.master.lookupNode(node_name)
			server = xmlrpclib.ServerProxy(uri)
			code, _msg, pid = server.getPid(rospy.get_name())
			if code == 1:
				self.pid_cache[node_name] = pid
				return pid
		except Exception as exc:
			rospy.logwarn_throttle(10.0, 'Could not resolve PID for {0}: {1}'.format(node_name, exc))

		return None

	def read_process_sample(self, pid, sample_time):
		stat_path = '/proc/{0}/stat'.format(pid)
		try:
			with open(stat_path, 'r') as stat_file:
				fields = stat_file.read().split()
		except IOError:
			return None

		if len(fields) < 24:
			return None

		total_ticks = float(fields[13]) + float(fields[14])
		rss_pages = float(fields[23])
		rss_mb = (rss_pages * self.page_size) / (1024.0 * 1024.0)

		previous = self.prev_proc_cpu.get(pid)
		cpu_pct = 0.0
		total_proc_seconds = total_ticks / self.clock_ticks
		if previous is not None:
			delta_proc = total_proc_seconds - previous['proc_seconds']
			delta_wall = sample_time - previous['wall_time']
			if delta_wall > 0.0:
				cpu_pct = max(0.0, (delta_proc / delta_wall) * 100.0)

		self.prev_proc_cpu[pid] = {
			'proc_seconds': total_proc_seconds,
			'wall_time': sample_time,
		}

		return {
			'cpu_pct': cpu_pct,
			'rss_mb': rss_mb,
		}

	def get_current_sim_time(self):
		current_time = rospy.Time.now()
		if current_time is None:
			return None
		return current_time.to_sec()

	def copy_position(self, pose):
		if pose is None:
			return None
		return (pose.position.x, pose.position.y)

	def compute_distance_between_positions(self, first_position, second_position):
		dx = second_position[0] - first_position[0]
		dy = second_position[1] - first_position[1]
		return math.hypot(dx, dy)

	def compute_path_distance(self, path_msg):
		if not path_msg.poses or len(path_msg.poses) < 2:
			return 0.0

		total_distance = 0.0
		previous_pose = path_msg.poses[0].pose.position
		for pose_stamped in path_msg.poses[1:]:
			current_pose = pose_stamped.pose.position
			total_distance += math.hypot(current_pose.x - previous_pose.x, current_pose.y - previous_pose.y)
			previous_pose = current_pose
		return total_distance

	def compute_path_signature(self, path_msg):
		if not path_msg.poses:
			return (0, 0.0, 0.0, 0.0, 0.0, 0.0)

		first_point = path_msg.poses[0].pose.position
		last_point = path_msg.poses[-1].pose.position
		path_distance = self.compute_path_distance(path_msg)
		return (
			len(path_msg.poses),
			round(path_distance, 3),
			round(first_point.x, 3),
			round(first_point.y, 3),
			round(last_point.x, 3),
			round(last_point.y, 3),
		)

	def finalize_run(self, status):
		if not self.goal_active:
			return

		finish_time = time.time()
		finish_sim_time = self.get_current_sim_time()
		travel_time = None
		if self.goal_start_sim is not None and finish_sim_time is not None:
			travel_time = finish_sim_time - self.goal_start_sim

		cpu_values = [sample['cpu_pct'] for sample in self.resource_samples]
		cpu_capacity_values = [sample['cpu_capacity_pct'] for sample in self.resource_samples]
		rss_values = [sample['rss_mb'] for sample in self.resource_samples]
		coverage_values = [sample.get('process_coverage') for sample in self.resource_samples if sample.get('process_coverage') is not None]
		replan_values = [event['replan_latency_sec'] for event in self.obstacle_events if event.get('replan_latency_sec') is not None]
		avg_speed = None
		time_per_meter = None
		if travel_time is not None and travel_time > 0.0:
			avg_speed = self.travel_distance_m / travel_time
			if self.travel_distance_m > 0.0:
				time_per_meter = travel_time / self.travel_distance_m

		run_data = {
			'algo_name': self.algo_name,
			'status': status,
			'goal_index': self.goal_index,
			'started_at_epoch': self.goal_start_wall,
			'finished_at_epoch': finish_time,
			'started_at_sim_sec': self.goal_start_sim,
			'finished_at_sim_sec': finish_sim_time,
			'travel_time_sec': travel_time,
			'travel_distance_m': self.travel_distance_m,
			'goal_tolerance_m': self.goal_tolerance,
			'tracked_nodes': self.tracked_nodes,
			'path_topics': self.path_topics,
			'obstacle_events': self.obstacle_events,
			'path_update_events': self.path_update_events,
			'resource_samples': self.resource_samples,
			'summary': {
				'resource_samples': len(self.resource_samples),
				'avg_process_coverage': self.safe_average(coverage_values),
				'min_process_coverage': min(coverage_values) if coverage_values else None,
				'path_updates': len(self.path_update_events),
				'obstacle_events': len(self.obstacle_events),
				'replan_events': len(replan_values),
				'planned_path_distance_m': self.last_path_distance,
				'travel_distance_m': self.travel_distance_m,
				'avg_speed_mps': avg_speed,
				'time_per_meter_sec': time_per_meter,
				'avg_replan_latency_sec': self.safe_average(replan_values),
				'max_replan_latency_sec': max(replan_values) if replan_values else None,
				'avg_cpu_pct': self.safe_average(cpu_values),
				'peak_cpu_pct': max(cpu_values) if cpu_values else None,
				'avg_cpu_capacity_pct': self.safe_average(cpu_capacity_values),
				'peak_cpu_capacity_pct': max(cpu_capacity_values) if cpu_capacity_values else None,
				'avg_memory_mb': self.safe_average(rss_values),
				'peak_memory_mb': max(rss_values) if rss_values else None,
			},
		}

		self.write_run_json(run_data)
		self.append_summary_csv(run_data)
		rospy.loginfo('Saved monitor run for %s with status %s.', self.algo_name, status)

		self.goal_active = False
		self.goal_start_wall = None
		self.goal_start_sim = None
		self.last_motion_pose = None
		self.last_motion_time = None

	def write_run_json(self, run_data):
		timestamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(run_data['finished_at_epoch']))
		file_name = '{0}_{1}.json'.format(self.algo_name, timestamp)
		file_path = os.path.join(self.log_dir, file_name)
		with open(file_path, 'w') as output_file:
			json.dump(run_data, output_file, indent=2, sort_keys=True)

	def append_summary_csv(self, run_data):
		file_exists = os.path.exists(self.summary_csv_path)
		with open(self.summary_csv_path, 'ab') as csv_file:
			writer = csv.writer(csv_file)
			if not file_exists:
				writer.writerow([
					'algo_name',
					'status',
					'goal_index',
					'travel_time_sec',
					'travel_distance_m',
					'planned_path_distance_m',
					'time_per_meter_sec',
					'avg_speed_mps',
					'path_updates',
					'obstacle_events',
					'replan_events',
					'avg_replan_latency_sec',
					'max_replan_latency_sec',
					'resource_samples',
					'avg_process_coverage',
					'min_process_coverage',
					'avg_cpu_pct',
					'peak_cpu_pct',
					'avg_cpu_capacity_pct',
					'peak_cpu_capacity_pct',
					'avg_memory_mb',
					'peak_memory_mb',
					'finished_at_epoch',
				])

			summary = run_data['summary']
			writer.writerow([
				run_data['algo_name'],
				run_data['status'],
				run_data['goal_index'],
				self.safe_round(run_data['travel_time_sec']),
				self.safe_round(run_data['travel_distance_m']),
				self.safe_round(summary['planned_path_distance_m']),
				self.safe_round(summary['time_per_meter_sec']),
				self.safe_round(summary['avg_speed_mps']),
				summary['path_updates'],
				summary['obstacle_events'],
				summary['replan_events'],
				self.safe_round(summary['avg_replan_latency_sec']),
				self.safe_round(summary['max_replan_latency_sec']),
				summary['resource_samples'],
				self.safe_round(summary['avg_process_coverage']),
				self.safe_round(summary['min_process_coverage']),
				self.safe_round(summary['avg_cpu_pct']),
				self.safe_round(summary['peak_cpu_pct']),
				self.safe_round(summary['avg_cpu_capacity_pct']),
				self.safe_round(summary['peak_cpu_capacity_pct']),
				self.safe_round(summary['avg_memory_mb']),
				self.safe_round(summary['peak_memory_mb']),
				self.safe_round(run_data['finished_at_epoch']),
			])

	def safe_average(self, values):
		if not values:
			return None
		return sum(values) / float(len(values))

	def safe_round(self, value):
		if value is None:
			return ''
		return round(value, 4)

	def handle_shutdown(self):
		if self.goal_active:
			self.finalize_run('shutdown')


def main():
	monitor = ResourceMonitor()
	rospy.spin()


if __name__ == '__main__':
	main()
