#!/usr/bin/env python3
"""Record a rosbag and a screen-capture video of RViz at the same time.

One-time setup on a new machine (no sudo needed):

    pip install --user python-xlib

Usage: start your lab's launch file yourself (with rviz:=true), then run this in
another terminal:

    source /opt/ros/jazzy/setup.bash
    source <workspace>/install/setup.bash
    python3 scripts/record_demo.py bags/task1-position

It finds the RViz window, raises it, starts `ros2 bag record` (the required topic
list) and a GStreamer screen capture of that window together, and stops both cleanly
on Ctrl-C. Run your task (publish goals, etc.) in a third terminal while this runs.

Why GStreamer's ximagesrc and not ffmpeg's x11grab: x11grab captures a fixed screen
region from the (possibly composited) root window, which on this desktop returns a
black frame for a window under a compositing window manager. ximagesrc captures by
window ID directly and does not have that problem. Confirmed the hard way.

Why VP8/WebM and not H.264: no GStreamer H.264 encoder plugin is installed here and
that needs sudo to fix. VP8 (`vp8enc`) plays in any modern browser and video player,
which is all a report or a submission needs.
"""

import argparse
import re
import signal
import subprocess
import sys
import time

BAG_TOPICS = [
    '/odom', '/cmd_vel', '/scan', '/new_position', '/tf', '/tf_static',
    '/goal_marker', '/closest_wall_point',
]


def find_rviz_window_id() -> int:
    """Return the top-level (decorated) RViz window's X id, via xwininfo."""
    out = subprocess.run(
        ['xwininfo', '-root', '-tree'], capture_output=True, text=True, check=True
    ).stdout
    for line in out.splitlines():
        if 'RViz' in line and 'mutter-x11-frames' in line:
            match = re.search(r'(0x[0-9a-f]+)', line)
            if match:
                return int(match.group(1), 16)
    raise SystemExit('No RViz window found. Is it running with rviz:=true?')


def raise_window(win_id: int) -> None:
    """Bring the window to the front so screen capture doesn't grab whatever
    happens to be on top of it. Needs python-xlib; see the module docstring."""
    from Xlib import X, display
    from Xlib.protocol import event

    d = display.Display()
    root = d.screen().root
    win = d.create_resource_object('window', win_id)
    net_active = d.intern_atom('_NET_ACTIVE_WINDOW')
    ev = event.ClientMessage(
        window=win, client_type=net_active, data=(32, [1, X.CurrentTime, 0, 0, 0]))
    root.send_event(ev, event_mask=X.SubstructureRedirectMask | X.SubstructureNotifyMask)
    win.configure(stack_mode=X.Above)
    win.set_input_focus(X.RevertToParent, X.CurrentTime)
    d.sync()
    time.sleep(1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('out_basename',
                         help='bag goes to <out_basename>/, video to <out_basename>.webm')
    args = parser.parse_args()

    win_id = find_rviz_window_id()
    raise_window(win_id)

    bag_proc = subprocess.Popen(
        ['ros2', 'bag', 'record', '-o', args.out_basename] + BAG_TOPICS)
    video_proc = subprocess.Popen([
        'gst-launch-1.0', '-e', 'ximagesrc', f'xid={win_id}', '!', 'videoconvert',
        '!', 'vp8enc', '!', 'webmmux', '!', 'filesink',
        f'location={args.out_basename}.webm',
    ])

    print(f'Recording bag to {args.out_basename}/ and video to '
          f'{args.out_basename}.webm -- run your task now. Ctrl-C here to stop both.')

    def stop(*_):
        bag_proc.send_signal(signal.SIGINT)
        video_proc.send_signal(signal.SIGINT)
        bag_proc.wait(timeout=10)
        video_proc.wait(timeout=10)
        print('Stopped. Verify with: ros2 bag info', args.out_basename)
        sys.exit(0)

    signal.signal(signal.SIGINT, stop)
    signal.pause()


if __name__ == '__main__':
    main()
