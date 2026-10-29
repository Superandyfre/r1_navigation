#!/usr/bin/env python3
"""CogNav <-> nav2/NeuPAN file bridge (ROS 2 Humble, system Python 3.10).

CogNav runs on Python 3.8 and cannot import rclpy, so the two halves talk
through a spool directory in /dev/shm:

    this process (ROS side)                     the driver (CogNav side)
    --------------------------------------      --------------------------
    RGB-D + TF + localization health   -->      obs/<stamp_ns>/{rgb.npy,depth.npy,meta.json}
    map pose + scan + odom speed       -->      live/motion.json
    plan/*.json  <-- nav2 FollowPath goal       <--  CogNav reference plan

This node NEVER publishes a velocity command. Global planning stays in CogNav
(FMM); motion stays with nav2: FollowPath -> NeuPANController (n1_16_node) ->
velocity_smoother -> r1_collision_guard -> /cmd_vel.

The frozen `candidate_transform` is written as a copy of the base_from_candidate
sample taken at capture time, because this candidate camera chain is not yet a
validated production extrinsic (`accepted_as_production_tf=false`, which the
CogNav contract only accepts under an explicit offline opt-in). A production
deployment must freeze a genuinely validated, physically measured transform
instead and set the acceptance flag after the 5-pose ArUco test.
"""
import argparse
import hashlib
import json
import math
import os
import shutil
import time
import uuid
from collections import deque
from pathlib import Path

from robot_live.protocol import (GoalLifecycle, SettledObservation, SCHEMA, IDENTITY,
                                 identity, lease_reason, read_json, write_json)
from robot_live.shutdown import StopSignals, finish_bridge
from robot_live.neupan_failstop import NeuPANFailstop

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path as PathMsg
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from tf2_ros import Buffer, TransformListener

OBS_KEEP = 6


def ros_stamp(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def camera_record(message):
    return {'width': message.width, 'height': message.height,
            'frame_id': message.header.frame_id,
            'stamp_ns': ros_stamp(message.header.stamp),
            'k': [float(v) for v in message.k], 'd': [float(v) for v in message.d],
            'r': [float(v) for v in message.r], 'p': [float(v) for v in message.p],
            'distortion_model': message.distortion_model,
            'binning_x': int(message.binning_x), 'binning_y': int(message.binning_y),
            'roi': {k: int(getattr(message.roi, k))
                    for k in ('x_offset', 'y_offset', 'width', 'height')}}


def image_numpy(message, channels):
    """Exact buffer reinterpretation; never an implicit colour conversion."""
    if message.is_bigendian:
        raise ValueError('big-endian image buffer is not supported')
    height, width = int(message.height), int(message.width)
    if channels == 3:
        raw = np.frombuffer(message.data, dtype=np.uint8)
        row = int(message.step)
        if raw.size != height * row or row < width * 3:
            raise ValueError('rgb buffer shape mismatch')
        return raw.reshape(height, row)[:, :width * 3].reshape(height, width, 3).copy()
    raw = np.frombuffer(message.data, dtype=np.uint16)
    row = int(message.step) // 2
    if raw.size != height * row or row < width:
        raise ValueError('depth buffer shape mismatch')
    return raw.reshape(height, row)[:, :width].copy()


def quat_yaw(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class Bridge(Node):
    def __init__(self, args):
        super().__init__('cognav_bridge')
        self.a = args
        self.root = Path(args.spool)
        for sub in ('obs', 'live', 'plan', 'ack'):
            (self.root / sub).mkdir(parents=True, exist_ok=True)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.msgs = {}
        self.receipts = {}
        self.session = uuid.uuid4().hex
        self.lifecycle = GoalLifecycle()
        self.owner_id=str(uuid.uuid4())
        self.owner_sequence=0
        self.owner_pub=self.create_publisher(String,'/neupan_execution_owner',20)
        self.owner_timer=self.create_timer(.1,self.publish_execution_owner)
        self.neupan = NeuPANFailstop()
        self.neupan_subscriptions = []
        for kind, topic in [('worker', args.worker_health_topic),
                            ('controller', args.controller_status_topic), ('guard', args.guard_topic)]:
            def observe(message, kind=kind):
                try:
                    self.neupan.observe(kind, json.loads(message.data), time.monotonic())
                except (ValueError, TypeError):
                    return
            self.neupan_subscriptions.append(self.create_subscription(String, topic, observe, 20))
        # Set by finish_bridge() before the shutdown drain: while true no new
        # goal may be dispatched, so the drain cannot resurrect a stopped task.
        self.stopping = False
        self.settled = SettledObservation()
        self.execution_sequence = 0
        self.last_measured_pose = None
        self.last_execution_log_key = None
        # RGB and aligned depth are separate topics; at low frame rates the newest
        # sample of one can be a frame ahead of the other, so buffer a few and pair
        # by stamp instead of only keeping the latest of each.
        self.queues = {'rgb': deque(maxlen=8), 'depth': deque(maxlen=8),
                       'rgb_info': deque(maxlen=8), 'depth_info': deque(maxlen=8)}
        self.last_obs_stamp = 0
        self.last_motion_log = 0.0
        self.counts = {'obs_written': 0, 'obs_rejected': 0, 'motion_written': 0,
                       'plans_seen': 0, 'goals_sent': 0, 'goals_refused': 0}
        self.last_reject = ''
        # Ignore all pre-start plans. Driver must handshake this new session and
        # publish a new sequence; an old unexpired spool file is not authority.
        self.last_seq = max([int(p.stem) for p in (self.root / 'plan').glob('*.json')
                             if p.stem.isdigit()] or [0])
        self.goal_handle = None
        self.goal_seq = None
        self.goal_state = 'idle'

        for topic, key, qos in (
                (args.rgb_topic, 'rgb', qos_profile_sensor_data),
                (args.depth_topic, 'depth', qos_profile_sensor_data),
                (args.rgb_info_topic, 'rgb_info', qos_profile_sensor_data),
                (args.depth_info_topic, 'depth_info', qos_profile_sensor_data),
                (args.scan_topic, 'scan', qos_profile_sensor_data),
                (args.odom_topic, 'odom', 20),
                (args.health_topic, 'health', 10),
                (args.guard_topic, 'guard', 10)):
            self._subscribe(topic, key, qos)

        self.client = ActionClient(self, FollowPath, args.follow_path_action)
        self.create_timer(1.0 / max(args.max_obs_hz, 0.5), self.write_observation)
        self.create_timer(1.0 / args.motion_hz, self.write_motion)
        self.create_timer(0.1, self.poll_plan)
        self.create_timer(1.0, self.write_status)
        self.write_status()
        self.get_logger().info('bridge up, spool=%s candidate=%s' % (self.root, args.candidate_frame))

    def _subscribe(self, topic, key, qos):
        if key in self.queues:
            queue = self.queues[key]
            kind = Image if key in ('rgb', 'depth') else CameraInfo

            def cb(msg):
                stamp = (int(msg.header.stamp.sec) * 1_000_000_000
                         + int(msg.header.stamp.nanosec))
                queue.append((stamp, msg, self.get_clock().now().nanoseconds))
        else:
            kind = LaserScan if key == 'scan' else Odometry if key == 'odom' else String

            def cb(msg):
                self.msgs[key] = (msg, self.get_clock().now().nanoseconds)
                self.receipts[key] = time.monotonic()
        self.create_subscription(kind, topic, cb, qos)

    @staticmethod
    def _nearest(queue, stamp, tolerance_ns):
        best = None
        for entry in queue:
            delta = abs(entry[0] - stamp)
            if delta <= tolerance_ns and (best is None or delta < best[0]):
                best = (delta, entry)
        return None if best is None else best[1]

    # ---------------- observation spool ----------------

    def _tf(self, target, source, stamp_ns):
        # Depth arrives a few ms before the pose TF; wait briefly instead of
        # discarding the frame as an extrapolation error.
        msg = self.buf.lookup_transform(target, source, Time(nanoseconds=int(stamp_ns)),
                                        Duration(seconds=0.15))
        t = msg.transform.translation
        q = msg.transform.rotation
        # lookup_stamp_ns is the exact capture time we asked for; the returned
        # sample stamp is reported as 0 because tf2 already interpolated here.
        return {'target_frame': target, 'source_frame': source,
                'lookup_stamp_ns': int(stamp_ns), 'returned_tf_stamp_ns': 0,
                'translation_xyz_m': [float(t.x), float(t.y), float(t.z)],
                'quaternion_xyzw': [float(q.x), float(q.y), float(q.z), float(q.w)]}

    def write_observation(self):
        now_ns = self.get_clock().now().nanoseconds
        if 'health' not in self.msgs or not self.queues['depth'] or not self.queues['rgb']:
            return
        ds, depth_msg = None, None
        rgb_entry = None
        for stamp, candidate, _ in reversed(self.queues['depth']):
            if stamp <= self.last_obs_stamp:
                break
            entry = self._nearest(self.queues['rgb'], stamp, 40_000_000)
            if entry is not None:
                ds, depth_msg, rgb_entry = stamp, candidate, entry
                break
        if depth_msg is None:
            if self.queues['depth'][-1][0] > self.last_obs_stamp:
                return self._reject('no rgb frame within 40 ms of any buffered depth frame')
            return
        if now_ns - ds > 2_000_000_000 or ds - now_ns > 100_000_000:
            return
        rs, rgb_msg, _ = rgb_entry
        ci_entry = self._nearest(self.queues['rgb_info'], rs, 100_000_000)
        di_entry = self._nearest(self.queues['depth_info'], ds, 100_000_000)
        if ci_entry is None or di_entry is None:
            return self._reject('camera_info is not time-aligned with the pair')
        ci, di = ci_entry[1], di_entry[1]
        health, health_recv = self.msgs['health']
        if rgb_msg.encoding != 'rgb8' or depth_msg.encoding != '16UC1':
            return self._reject('unexpected encodings %s/%s' % (rgb_msg.encoding, depth_msg.encoding))
        if (ci.width, ci.height) != (rgb_msg.width, rgb_msg.height) or \
                (di.width, di.height) != (depth_msg.width, depth_msg.height):
            return self._reject('camera_info grid differs from image')
        if di.header.frame_id != rgb_msg.header.frame_id or ci.header.frame_id != di.header.frame_id:
            return self._reject('camera_info frames differ')

        try:
            data = json.loads(health.data)
        except ValueError:
            return self._reject('localization health is not JSON')
        if (data.get('state') != 'READY' or data.get('ready') is not True
                or data.get('fault_latched') is not False or data.get('preflight_ready') is not True
                or data.get('operator_relocalization_pending') is not False):
            return self._reject('localization not READY (%s)' % data.get('state'))
        age = (now_ns - health_recv) / 1e9
        if not -0.1 <= age <= 1.0:
            return self._reject('localization health is stale (%.2fs)' % age)

        try:
            rgb = image_numpy(rgb_msg, 3)
            depth = image_numpy(depth_msg, 1)
            base = self._tf('base_link', self.a.candidate_frame, ds)
            mapping = self._tf('map', 'base_link', ds)
            direct = self._tf('map', self.a.candidate_frame, ds)
        except Exception as error:  # TF not yet available / buffer miss
            return self._reject('capture rejected: %s' % error)

        candidate = dict(base)
        candidate.update({'accepted_as_production_tf': False,
                          'calibration_validation_passed': False,
                          'calibration_version': self.a.calibration_version,
                          'source_frame': self.a.candidate_frame,
                          'actual_image_frame_unchanged': rgb_msg.header.frame_id})
        meta = {
            'schema': 'cognav_bridge_observation_v1',
            'capture_monotonic': time.monotonic() - (self.get_clock().now().nanoseconds - ds) / 1e9,
            'observation_id': str(ds),
            'rgb_encoding': rgb_msg.encoding,
            'depth_encoding': depth_msg.encoding,
            'size_wh': [int(rgb_msg.width), int(rgb_msg.height)],
            'depth_scale_m': 0.001,
            'rgb_stamp_ns': rs,
            'depth_stamp_ns': ds,
            'rgb_frame_id': rgb_msg.header.frame_id,
            'rgb_camera_info': camera_record(ci),
            'depth_camera_info': camera_record(di),
            'rgb_pixel_sha256': hashlib.sha256(rgb.tobytes()).hexdigest(),
            'localization_health_at_capture': {
                'data': data,
                'checked_at_ros_time_ns': int(now_ns),
                'receipt_ros_time_ns': int(health_recv),
                'age_at_check_sec': float(age)},
            'map_transforms': {'base_from_candidate': base,
                               'map_from_base': mapping,
                               'map_from_candidate': direct},
            'candidate_transform': candidate,
        }

        tmp = self.root / 'obs' / ('.tmp' + str(ds))
        if tmp.exists():
            shutil.rmtree(tmp, ignore_errors=True)
        tmp.mkdir(parents=True)
        np.save(str(tmp / 'rgb.npy'), rgb)
        np.save(str(tmp / 'depth.npy'), depth)
        (tmp / 'meta.json').write_text(json.dumps(meta), encoding='utf-8')
        final = self.root / 'obs' / str(ds)
        if final.exists():
            shutil.rmtree(final, ignore_errors=True)
        tmp.rename(final)
        self.last_obs_stamp = ds
        self.counts['obs_written'] += 1
        self._prune_observations()

    def _prune_observations(self):
        dirs = sorted((p for p in (self.root / 'obs').iterdir()
                       if p.is_dir() and p.name.isdigit()), key=lambda p: int(p.name))
        for old in dirs[:-OBS_KEEP]:
            shutil.rmtree(old, ignore_errors=True)

    def _reject(self, reason):
        self.counts['obs_rejected'] += 1
        if reason != self.last_reject:
            self.last_reject = reason
            self.get_logger().warn('observation skipped: ' + reason)

    # ---------------- motion spool ----------------

    def publish_execution_owner(self):
        now = time.monotonic()
        if self.stopping or self.neupan.reason(now):
            return
        with self.lifecycle.lock:
            ctx=self.lifecycle.current
            if not ctx or ctx['state']!='executing' or ctx.get('cancel_requested') or not ctx.get('accepted'):
                return
            # Renewal must independently require the parent driver's authority;
            # do not depend on the scheduling order of the separate watchdog.
            if lease_reason(ctx, read_json(self.root / 'live' / 'lease.json'),
                            self.session, now, self.a.lease_timeout):
                return
            motion = getattr(self, 'motion_payload', {})
            if not (motion.get('inputs_valid') is True and self._health_ready()
                    and 0 <= now - motion.get('received_at', 0) <= self.a.input_max_age
                    and 0 <= now - motion.get('obstacles_received_at', 0) <= self.a.input_max_age):
                return
            attempt=ctx.get('navigation_attempt_id')
        record=self.neupan.records.get('controller',{}).get('row',{})
        if (record.get('navigation_attempt_id')!=attempt or not record.get('execution_active')
                or not record.get('authority_boot_id')):
            return
        self.owner_sequence+=1
        owner={k:record[k] for k in ['authority_boot_id','controller_boot_id','navigation_attempt_id']}
        owner.update(schema='r1_neupan_execution_owner_v1',owner_id=self.owner_id,
                     sequence=self.owner_sequence,monotonic=time.monotonic())
        self.owner_pub.publish(String(data=json.dumps(owner)))

    def write_motion(self):
        now = time.monotonic()
        payload = {'written_at': now, 'received_at': 0., 'obstacles_received_at': 0.,
                   'pose': [0.0, 0.0, 0.0], 'obstacles': [],
                   'localization_ready': False, 'inputs_valid': False,
                   'calibration_validated': False, 'input_error': '',
                   'linear_speed_m_s': None, 'angular_speed_rad_s': None}
        try:
            pose_msg = self.buf.lookup_transform('map', 'base_link', Time())
            tf_age = (self.get_clock().now().nanoseconds - ros_stamp(pose_msg.header.stamp)) / 1e9
            if not 0 <= tf_age <= self.a.input_max_age:
                raise ValueError('map/base TF stale or future')
            t = pose_msg.transform.translation
            q = pose_msg.transform.rotation
            payload['pose'] = [float(t.x), float(t.y), quat_yaw([q.x, q.y, q.z, q.w])]
            payload['tf_age_s'] = tf_age
            if not self._health_ready():
                raise ValueError('localization health unavailable')
            odom = self.msgs.get('odom')
            scan = self.msgs.get('scan')
            if odom is None or scan is None:
                raise ValueError('scan/odom unavailable')
            odom_at = self._source_time('odom', now)
            scan_at = self._source_time('scan', now)
            tw = odom[0].twist.twist
            payload['linear_speed_m_s'] = float(math.hypot(tw.linear.x, tw.linear.y))
            payload['angular_speed_rad_s'] = float(tw.angular.z)
            payload['obstacles'] = self._scan_in_map(scan[0])
            if not payload['obstacles']:
                raise ValueError('scan has no usable finite returns')
            payload['received_at'] = min(odom_at, now - tf_age)
            payload['obstacles_received_at'] = scan_at
            payload['scan_stamp_ns'] = ros_stamp(scan[0].header.stamp)
            payload['odom_stamp_ns'] = ros_stamp(odom[0].header.stamp)
            payload['localization_ready'] = True
            payload['inputs_valid'] = True
        except Exception as error:
            payload['input_error'] = str(error)
        self.motion_payload = payload
        write_json(self.root / 'live' / 'motion.json', payload)
        self.counts['motion_written'] += 1

    def _source_time(self, key, now):
        message = self.msgs[key][0]
        header_age = (self.get_clock().now().nanoseconds - ros_stamp(message.header.stamp)) / 1e9
        recv = self.receipts.get(key, 0.)
        if not 0 <= header_age <= self.a.input_max_age or not 0 <= now - recv <= self.a.input_max_age:
            raise ValueError(key + ' stale/future header or receipt')
        return min(recv, now - header_age)

    def _scan_in_map(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        if not ranges.size or not math.isfinite(msg.angle_increment) or msg.angle_increment == 0.0:
            return []
        angle = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        if not valid.any():
            return []
        stamp = ros_stamp(msg.header.stamp)
        tf = self.buf.lookup_transform('map', msg.header.frame_id, Time(nanoseconds=stamp))
        t = tf.transform.translation
        q = tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rot = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        local = np.column_stack((ranges[valid] * np.cos(angle[valid]),
                                 ranges[valid] * np.sin(angle[valid]), np.zeros(int(valid.sum()))))
        world = local @ rot.T + np.array([t.x, t.y, t.z])
        if world.shape[0] > self.a.max_obstacles:
            step = int(math.ceil(world.shape[0] / float(self.a.max_obstacles)))
            world = world[::step]
        return [[round(float(p[0]), 3), round(float(p[1]), 3)] for p in world]

    # ---------------- plan -> FollowPath ----------------

    def _health_ready(self):
        item = self.msgs.get('health')
        if item is None:
            return False
        if self.get_clock().now().nanoseconds - item[1] > 1_000_000_000:
            return False
        try:
            data = json.loads(item[0].data)
        except ValueError:
            return False
        return (data.get('state') == 'READY' and data.get('ready') is True
                and data.get('operator_relocalization_pending') is False
                and data.get('preflight_ready') is True and data.get('fault_latched') is False)

    def poll_plan(self):
        self._execution_watchdog()
        if self.stopping:
            # Shutdown in progress: cancel-only, never dispatch a new goal.
            return
        files = sorted((p for p in (self.root / 'plan').glob('*.json') if p.stem.isdigit()),
                       key=lambda p: int(p.stem))
        if not files:
            return
        newest = files[-1]
        seq = int(newest.stem)
        if seq <= self.last_seq:
            return
        try:
            payload = json.loads(newest.read_text(encoding='utf-8'))
        except ValueError:
            return
        kind = payload.get('kind')
        try:
            identity(payload)
            if payload['bridge_session'] != self.session:
                raise ValueError('old bridge session')
        except ValueError as error:
            self.last_seq = seq
            self.lifecycle.cancel('invalid_new_plan')
            self._ack(seq, payload, sent=False, note=str(error))
            return
        if kind in ('hold', 'verified_stop', 'stop'):
            self.last_seq = seq
            self.counts['plans_seen'] += 1
            self._cancel_goal('plan_' + str(kind))
            self.goal_state = 'cancel_requested_by_%s' % kind
            self._ack(seq, payload, sent=False, note='cancel requested; stop not yet measured')
            return
        veto = self.neupan.reason(time.monotonic())
        if veto:
            self.lifecycle.cancel('neupan:' + veto)
            self.last_seq = seq
            self.counts['goals_refused'] += 1
            self._ack(seq, payload, sent=False, note='neupan:' + veto)
            return
        reason = lease_reason(payload, read_json(self.root / 'live' / 'lease.json'),
                              self.session, time.monotonic(), self.a.lease_timeout)
        if reason:
            self.lifecycle.cancel(reason)
            # Do not consume it: a fresh valid matching lease may arrive next tick.
            return
        if kind not in ('navigate', 'observe'):
            self.lifecycle.cancel('unknown_plan_kind')
            self.last_seq = seq
            return
        current = self.lifecycle.current
        if self.lifecycle.busy():
            if current['identity'] != identity(payload):
                self.lifecycle.cancel('superseded')
            return  # wait for terminal result, including late pending acceptance
        if payload.get('expires_at') is not None and payload['expires_at'] < time.monotonic():
            self.last_seq = seq
            self._ack(seq, payload, sent=False, note='plan already expired')
            return
        if not self._health_ready():
            self.last_seq = seq
            self.counts['goals_refused'] += 1
            self._ack(seq, payload, sent=False, note='localization not READY')
            return
        try:
            arr = np.load(str(self.root / 'plan' / payload['path_file']))['path']
        except Exception as error:
            self.last_seq = seq
            self._ack(seq, payload, sent=False, note='unreadable path: %s' % error)
            return
        if (arr.ndim != 2 or arr.shape[1] != 3 or len(arr) < 1
                or not np.isfinite(arr).all()):
            self.last_seq = seq
            self._ack(seq, payload, sent=False, note='bad path shape')
            return
        if not self.client.server_is_ready():
            # Do not consume the plan: retry once the FollowPath server is up.
            self.goal_state = 'waiting_for_follow_path_server'
            return

        if kind == 'observe':
            path = self._observe_path(arr[0])
        else:
            path = arr
        if len(path) < 2:
            self.last_seq = seq
            self._ack(seq, payload, sent=False, note='need two poses')
            return
        self.last_seq = seq
        self.counts['plans_seen'] += 1
        ok = self._send(seq, path, payload)
        self._ack(seq, payload, sent=ok, note='follow_path sent' if ok else 'goal rejected')

    def _execution_watchdog(self):
        now = time.monotonic()
        veto = self.neupan.reason(now)
        if veto and self.lifecycle.busy():
            self.lifecycle.cancel('neupan:' + veto)
        ctx = self.lifecycle.current
        motion = getattr(self, 'motion_payload', {})
        fresh = (motion.get('inputs_valid') is True and self._health_ready()
                 and 0 <= now - motion.get('received_at', 0) <= self.a.input_max_age
                 and 0 <= now - motion.get('obstacles_received_at', 0) <= self.a.input_max_age)
        reason = 'no_goal' if ctx is None else lease_reason(
            ctx, read_json(self.root / 'live' / 'lease.json'), self.session, now, self.a.lease_timeout)
        if ctx is not None and self.lifecycle.busy():
            if reason or not fresh:
                self.lifecycle.cancel(reason or 'robot_inputs_stale')
        row = self.lifecycle.result()
        if row is None:
            return
        linear, angular = motion.get('linear_speed_m_s'), motion.get('angular_speed_rad_s')
        stopped = (fresh and linear is not None and angular is not None
                   and math.isfinite(linear) and math.isfinite(angular)
                   and abs(linear) < .02 and abs(angular) < .03)
        aligned = False
        if ctx['kind'] == 'observe' and fresh:
            error = (ctx['target_yaw'] - motion['pose'][2] + math.pi) % (2 * math.pi) - math.pi
            aligned = abs(error) < math.radians(5)
        ready, ready_at = self.settled.update(ctx['identity'], now,
            fresh and not veto and not reason and not row['cancel_requested'] and row['state'] in ('executing', 'succeeded'),
            aligned, linear, angular)
        event = 'idle'
        if row['state'] in ('aborted', 'failed', 'rejected', 'cancelling', 'cancelled'):
            event = 'path_blocked'
        elif ctx['kind'] == 'observe':
            event = 'final_observation' if ctx['uid'] else 'recovery_observation'
        elif row['state'] == 'succeeded':
            event = 'goal_reached'
        elif fresh and row['state'] == 'executing':
            pose = np.asarray(motion['pose'][:2])
            if self.last_measured_pose is not None and np.linalg.norm(pose-self.last_measured_pose) > .003:
                event = 'moving'
            self.last_measured_pose = pose
        guard = self.msgs.get('guard')
        guard_data = None
        if guard is not None and now - self.receipts.get('guard', 0.) <= .6:
            try:
                guard_data = json.loads(guard[0].data)
            except ValueError:
                pass
        if guard_data and guard_data.get('state') in ('FAULT', 'STOP', 'BLOCKED'):
            event = 'path_blocked'
            ready, ready_at = False, None
            # A later guard failure must restart the entire stopped/aligned dwell.
            self.settled.since = None
        elif guard_data is None:
            ready, ready_at = False, None
            self.settled.since = None
        self.execution_sequence += 1
        row.update(feedback_seq=self.execution_sequence, navigation_event=event,
                   ready=ready, ready_at=ready_at, measured_stopped=bool(stopped),
                   pose=motion.get('pose'), guard=guard_data,
                   last_action=1 if fresh and linear is not None and abs(linear)>.02 else
                               (2 if angular is None or angular>=0 else 3))
        write_json(self.root / 'live' / 'execution.json', row)
        log_key = (ctx['identity'], row['state'], row['reason'], event, ready,
                   None if guard_data is None else guard_data.get('reason'))
        if log_key != self.last_execution_log_key:
            with (self.root / 'live' / 'execution_events.jsonl').open('a', encoding='utf-8') as handle:
                handle.write(json.dumps(row, allow_nan=False) + '\n')
            self.last_execution_log_key = log_key

    def _observe_path(self, pose):
        """Turn the single (x, y, yaw) observe pose into a short turning arc.

        A two-pose path at one XY has zero length, so NeuPAN reports "arrived",
        returns no optimized trajectory, and n1_16 raises on the empty array
        inside its own error handler -- which latches the planner node dead for
        the rest of the session (measured on the R1, 2026-10-25).  A short
        forward arc stays plannable while still turning the robot to the
        requested heading.  COGNAV_OBSERVE_ARC_M=0 restores the old same-XY
        two-pose behaviour for A/B comparison.
        """
        try:
            cur = self.buf.lookup_transform('map', 'base_link', Time())
        except Exception:
            return np.zeros((0, 3))
        t = cur.transform.translation
        q = cur.transform.rotation
        yaw_start = quat_yaw([q.x, q.y, q.z, q.w])
        yaw_end = float(pose[2])
        x0, y0 = float(t.x), float(t.y)
        arc = float(os.environ.get('COGNAV_OBSERVE_ARC_M', '0.25'))
        if arc <= 0.0:
            return np.array([[x0, y0, yaw_start], [x0, y0, yaw_end]])
        half_turn = 0.5 * math.atan2(math.sin(yaw_end - yaw_start),
                                     math.cos(yaw_end - yaw_start))
        return np.array([
            [x0, y0, yaw_start],
            [x0 + 0.5 * arc * math.cos(yaw_start),
             y0 + 0.5 * arc * math.sin(yaw_start), yaw_start + half_turn],
            [x0 + arc * math.cos(yaw_start),
             y0 + arc * math.sin(yaw_start), yaw_end],
        ])

    def _cancel_goal(self, reason):
        self.lifecycle.cancel(reason)

    def _send(self, seq, path, payload):
        if self.stopping or self.neupan.reason(time.monotonic()):
            self.counts['goals_refused'] += 1
            return False
        ctx = self.lifecycle.begin(payload)
        ctx['target_yaw'] = float(path[-1, 2])
        msg = PathMsg()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for row in np.asarray(path, dtype=float):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(row[0])
            ps.pose.position.y = float(row[1])
            ps.pose.position.z = 0.0
            half = float(row[2]) * 0.5
            ps.pose.orientation.z = math.sin(half)
            ps.pose.orientation.w = math.cos(half)
            msg.poses.append(ps)
        goal = FollowPath.Goal()
        goal.path = msg
        goal.controller_id = self.a.controller_id
        goal.goal_checker_id = self.a.goal_checker_id
        try:
            future = self.client.send_goal_async(goal, feedback_callback=
                lambda message, bound=ctx: self.lifecycle.action_feedback(bound, message.feedback))
        except Exception as error:
            self.lifecycle.fail_send(ctx, error)
            return False
        self.goal_seq = seq
        self.goal_state = 'goal_pending'
        future.add_done_callback(lambda result, bound=ctx: self.lifecycle.accepted(bound, result))
        self.counts['goals_sent'] += 1
        self.get_logger().info('follow_path seq=%d poses=%d end=(%.2f,%.2f,%.2f)' %
                               (seq, len(msg.poses), path[-1][0], path[-1][1], path[-1][2]))
        return True

    def _ack(self, seq, payload, sent, note):
        row = {'seq': seq, 'kind': payload.get('kind'), 'version': payload.get('version'),
               'sent': bool(sent), 'note': note, 'at': time.monotonic()}
        row.update({k: payload.get(k) for k in IDENTITY})
        row['schema'] = SCHEMA
        target = self.root / 'ack' / ('%06d.json' % seq)
        tmp = self.root / 'ack' / ('.%06d.tmp' % seq)
        tmp.write_text(json.dumps(row), encoding='utf-8')
        os.replace(str(tmp), str(target))

    # ---------------- status ----------------

    def write_status(self):
        row = {'node': 'cognav_bridge', 'at': time.monotonic(),
               'schema': SCHEMA, 'bridge_session': self.session,
               'localization_ready': self._health_ready(),
               'last_observation_ns': self.last_obs_stamp,
               'last_plan_seq': self.last_seq, 'goal_state':
                   self.lifecycle.current['state'] if self.lifecycle.current else self.goal_state,
               'follow_path_server': self.client.server_is_ready(),
               'counts': dict(self.counts), 'neupan_veto': self.neupan.reason(time.monotonic()),
               'neupan_fatal_latched': self.neupan.fault}
        tmp = self.root / 'live' / '.status.tmp'
        tmp.write_text(json.dumps(row), encoding='utf-8')
        os.replace(str(tmp), str(self.root / 'live' / 'bridge.json'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spool', default='/dev/shm/cognav_bridge')
    parser.add_argument('--rgb-topic', default='/head/camera/color/image_raw')
    parser.add_argument('--depth-topic', default='/head/camera/aligned_depth_to_color/image_raw')
    parser.add_argument('--rgb-info-topic', default='/head/camera/color/camera_info')
    parser.add_argument('--depth-info-topic', default='/head/camera/aligned_depth_to_color/camera_info')
    parser.add_argument('--scan-topic', default='/scan_neupan')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--health-topic', default='/r1_localization_health')
    parser.add_argument('--worker-health-topic', default='/neupan_worker_health')
    parser.add_argument('--controller-status-topic', default='/neupan_controller_status')
    parser.add_argument('--guard-topic', default='/r1_collision_guard/status')
    parser.add_argument('--input-max-age', type=float, default=.5)
    parser.add_argument('--lease-timeout', type=float, default=.6)
    parser.add_argument('--candidate-frame', default='cognav_d455_color_optical_test')
    parser.add_argument('--calibration-version', default='candidate_tf_fixed_20261021_v1')
    parser.add_argument('--follow-path-action', default='/follow_path')
    parser.add_argument('--controller-id', default='FollowPath')
    parser.add_argument('--goal-checker-id', default='general_goal_checker')
    parser.add_argument('--max-obs-hz', type=float, default=3.0)
    parser.add_argument('--motion-hz', type=float, default=20.0)
    parser.add_argument('--max-obstacles', type=int, default=360)
    args = parser.parse_args()
    if not 0 < args.input_max_age <= 2 or not .1 <= args.lease_timeout <= 2:
        parser.error('Invalid robot input/lease age budget')

    # Install our own handlers before rclpy so SIGINT/SIGTERM both reach the
    # shutdown drain. Handlers only set a flag; ROS calls stay on the main thread.
    with StopSignals() as stop:
        node, executor = None, None
        try:
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            node = Bridge(args)
            executor = MultiThreadedExecutor(num_threads=2)
            executor.add_node(node)
            while rclpy.ok() and not stop.requested.is_set():
                executor.spin_once(timeout_sec=.1)
        except KeyboardInterrupt:
            stop.request()
        finally:
            finish_bridge(node, executor, rclpy, stop.signum)


if __name__ == '__main__':
    main()
