#!/usr/bin/env python3
"""Replay a TurtleBot3 rosbag in RViz and screen-record it to a video.

    source /opt/ros/jazzy/setup.bash
    python3 ~/R7021E-group6/tools/rviz_replay/rviz_replay.py <bag dir> [--out run.webm] [--rate 2]

The video shows the robot model, the path so far (from /odom), the current goal
(/new_position, if the bag has it) and the laser scan, top-down in the odom frame, framed
to fit the whole run. Works for any lab's bag with /odom, /tf and /scan.

Needs: gstreamer1.0-plugins-good (ximagesrc, vp8enc, webmmux), x11-utils (xwininfo),
python3-xlib, xacro, joint_state_publisher, turtlebot3_description. Run it on a desktop
session: it records the RViz window, so do not cover it while it runs.

Run with --helper only from this script: it is the small node that turns /odom into a path
and /new_position into a marker for RViz.
"""

import argparse
import math
import os
import pathlib
import re
import signal
import subprocess
import sys
import tempfile
import time

HERE = pathlib.Path(__file__).resolve().parent


# ---------- the helper node: /odom -> path, /new_position -> marker ----------

def helper():
    import rclpy
    from geometry_msgs.msg import Pose, PoseStamped
    from nav_msgs.msg import Odometry, Path
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from visualization_msgs.msg import Marker

    class Helper(Node):
        def __init__(self):
            super().__init__('replay_helper')
            self.path = Path()
            self.path.header.frame_id = 'odom'
            self.path_pub = self.create_publisher(Path, '/replay_path', 10)
            self.goal_pub = self.create_publisher(Marker, '/replay_goal', 10)
            self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
            self.create_subscription(Pose, '/new_position', self.on_goal, 10)

        def on_odom(self, msg):
            p = msg.pose.pose.position
            if self.path.poses:
                last = self.path.poses[-1].pose.position
                if math.hypot(p.x - last.x, p.y - last.y) < 0.005:
                    return
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            self.path.poses.append(pose)
            self.path.header.stamp = msg.header.stamp
            self.path_pub.publish(self.path)

        def on_goal(self, msg):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.type = 'goal', Marker.SPHERE
            m.pose.position.x, m.pose.position.y = msg.position.x, msg.position.y
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.08
            m.color.r, m.color.g, m.color.b, m.color.a = 0.92, 0.41, 0.2, 1.0
            self.goal_pub.publish(m)

    rclpy.init()
    node = Helper()
    try:
        rclpy.spin(node)
    except BaseException:
        pass


# ---------- framing the view from the bag ----------

def run_extent(bag):
    """Box around the path and the laser hits (2-98 percentile, within 2.5 m), and the topics."""
    import numpy as np
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    topics = {t.name for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/odom', '/scan']))
    pose, xs, ys, scans = None, [], [], 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/odom':
            m = deserialize_message(data, Odometry).pose.pose
            pose = (m.position.x, m.position.y,
                    math.atan2(2 * (m.orientation.w * m.orientation.z + m.orientation.x * m.orientation.y),
                               1 - 2 * (m.orientation.y ** 2 + m.orientation.z ** 2)))
            xs.append(pose[0]); ys.append(pose[1])
        elif pose is not None and scans % 10 == 0:
            m = deserialize_message(data, LaserScan)
            r = np.array(m.ranges); a = m.angle_min + m.angle_increment * np.arange(len(r)) + pose[2]
            ok = (r > m.range_min) & (r < min(m.range_max, 2.5))
            xs.extend(pose[0] + r[ok] * np.cos(a[ok])); ys.extend(pose[1] + r[ok] * np.sin(a[ok]))
        if topic == '/scan':
            scans += 1
    if not xs:
        raise SystemExit('no /odom in %s' % bag)
    x0, x1 = np.percentile(xs, [2, 98]); y0, y1 = np.percentile(ys, [2, 98])
    return x0, x1, y0, y1, topics


def rviz_config(path, cx, cy, scale, width, height, title):
    text = (HERE / 'replay.rviz.in').read_text()
    for key, value in dict(CX=cx, CY=cy, SCALE=scale, WIDTH=width, HEIGHT=height, TITLE=title).items():
        text = text.replace('@%s@' % key, str(value))
    path.write_text(text)


# ---------- screen capture, as in record_demo.py ----------

def find_rviz_window(timeout=60):
    """(main window id, 3D view id). The view is the largest 'rviz2' child window."""
    end = time.time() + timeout
    while time.time() < end:
        out = subprocess.run(['xwininfo', '-root', '-tree'], capture_output=True, text=True).stdout
        main_id, best, area = None, None, 0
        for line in out.splitlines():
            size = re.search(r'\)\s+(\d+)x(\d+)', line)
            wid = re.search(r'(0x[0-9a-f]+)', line)
            if not (size and wid) or '("rviz2" "rviz2")' not in line:
                continue
            w, h = int(size.group(1)), int(size.group(2))
            if '- RViz' in line:
                main_id = int(wid.group(1), 16)
            elif w * h > area:
                best, area = int(wid.group(1), 16), w * h
        if main_id and best and area > 200 * 200:
            return main_id, best
        time.sleep(1)
    raise SystemExit('RViz window did not appear')


def raise_window(win_id):
    from Xlib import X, display
    from Xlib.protocol import event
    d = display.Display()
    root = d.screen().root
    win = d.create_resource_object('window', win_id)
    ev = event.ClientMessage(window=win, client_type=d.intern_atom('_NET_ACTIVE_WINDOW'),
                             data=(32, [1, X.CurrentTime, 0, 0, 0]))
    root.send_event(ev, event_mask=X.SubstructureRedirectMask | X.SubstructureNotifyMask)
    win.configure(stack_mode=X.Above)
    d.sync()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('bag')
    parser.add_argument('--out', default=None, help='video file, default <bag>.webm')
    parser.add_argument('--rate', type=float, default=1.0, help='playback speed')
    parser.add_argument('--title', default=None)
    parser.add_argument('--helper', action='store_true', help=argparse.SUPPRESS)
    args, _ = parser.parse_known_args()  # ROS adds --ros-args to the helper
    if args.helper:
        return helper()

    bag = pathlib.Path(args.bag).resolve()
    out = pathlib.Path(args.out or str(bag) + '.webm').resolve()
    x0, x1, y0, y1, topics = run_extent(bag)
    width, height = 1280, 960
    span = max(x1 - x0 + 0.6, (y1 - y0 + 0.6) * width / height, 2.0)
    tmp = pathlib.Path(tempfile.mkdtemp(prefix='rviz_replay_'))
    config = tmp / 'replay.rviz'
    rviz_config(config, (x0 + x1) / 2, (y0 + y1) / 2, round(width / span, 1), width, height,
                args.title or bag.name)

    env = dict(os.environ, ROS_DOMAIN_ID=os.environ.get('REPLAY_DOMAIN_ID', '77'))
    urdf = subprocess.run(['xacro', '/opt/ros/jazzy/share/turtlebot3_description/urdf/turtlebot3_burger.urdf',
                           'namespace:='], capture_output=True, text=True, check=True).stdout
    (tmp / 'rsp.yaml').write_text('robot_state_publisher:\n  ros__parameters:\n    use_sim_time: true\n'
                                  '    robot_description: |\n'
                                  + ''.join('      ' + line + '\n' for line in urdf.splitlines()))
    sim = ['--ros-args', '-p', 'use_sim_time:=true']
    procs = [
        subprocess.Popen(['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                          '--ros-args', '--params-file', str(tmp / 'rsp.yaml')], env=env, start_new_session=True),
        subprocess.Popen(['ros2', 'run', 'joint_state_publisher', 'joint_state_publisher'] + sim, env=env,
                         start_new_session=True),
        subprocess.Popen([sys.executable, __file__, '--helper'] + sim, env=env, start_new_session=True),
        subprocess.Popen(['rviz2', '-d', str(config)] + sim, env=env, start_new_session=True,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
    ]
    video = None
    try:
        time.sleep(5)
        win_main, win = find_rviz_window()
        raise_window(win_main)
        time.sleep(1)
        video = subprocess.Popen(['gst-launch-1.0', '-e', 'ximagesrc', 'xid=%d' % win, 'use-damage=false',
                                  '!', 'video/x-raw,framerate=15/1', '!', 'videoconvert', '!',
                                  'vp8enc', 'deadline=1', 'cpu-used=8', '!', 'webmmux', '!',
                                  'filesink', 'location=%s' % out],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2)
        play = [t for t in ('/odom', '/scan', '/tf', '/tf_static', '/new_position') if t in topics]
        subprocess.run(['ros2', 'bag', 'play', str(bag), '--clock', '50', '--rate', str(args.rate),
                        '--topics'] + play, env=env, check=True)
        time.sleep(2)
    finally:
        if video:
            video.send_signal(signal.SIGINT)
            video.wait(timeout=60)
        # Each child runs in its own process group: 'ros2 run' does not pass a signal on to
        # the node it started, so signal the whole group.
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for p in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
    print('saved %s (%.1f MB)' % (out, out.stat().st_size / 1e6))


if __name__ == '__main__':
    main()
