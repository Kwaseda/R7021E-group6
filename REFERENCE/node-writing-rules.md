# The basic rules for writing a ROS 2 node

You said you don't know these at all. They are not many, and they are not deep. Nearly
every node in this course is the same eight-part shape with different maths in slot 6.

Read this once now, then once more after you've typed your first node — it means more
the second time.

---

## The shape. Every node. No exceptions.

```python
#!/usr/bin/env python3
# 1. imports: rclpy, Node, then one line per message type you actually use
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class MyNode(Node):            # 2. a class that inherits from Node
    def __init__(self):
        super().__init__('my_node')          # 3. the node's name in the ROS graph

        self.declare_parameter('k_p', 0.8)   # 4. parameters, with defaults
        self.k_p = self.get_parameter('k_p').value

        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)       # 5. wiring
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_timer(0.05, self.control_tick)

        self.x = 0.0                         # state the callbacks will fill in
        self.y = 0.0
        self.yaw = 0.0
        self.goal = None                     # None means "nothing told me yet"

    def odom_cb(self, msg):                  # 6a. callbacks only STORE
        self.x = msg.pose.pose.position.x
        ...

    def control_tick(self):                  # 6b. the timer does the WORK
        if self.goal is None:
            return
        ... compute v, omega ...
        self.pub.publish(cmd)

    def stop(self):                          # 7. a way to make the robot stop
        self.pub.publish(TwistStamped())

def main():                                  # 8. init, spin, clean up
    rclpy.init()
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Memorise the eight slots, not the code. When you sit down to write a node, write the
eight comments first and then fill them in. That alone prevents most of the mess.

---

## What actually runs — the process model

Worth getting straight before the rules, because most of them follow from it.

**A launch file cannot run your code. It can only start processes.** Each `Node(...)` entry
is one operating system process — the same thing `ros2 run` starts, with its own pid. So a
launch file is *further* from your code than `ros2 run` is, not closer, and it needs `main()`
even more.

The chain, every time:

```
launch / ros2 run  →  wrapper script  →  main()  →  YourNode()  →  rclpy.spin()
                      (from setup.py)             (registers      (callbacks
                                                   pubs & subs)    run here)
```

Three consequences:

1. **Defining a class is not running it.** `class Receiver(Node)` is a blueprint. Importing
   the module defines it and stops. `main()` is the only thing that calls `YourNode()`, and
   that constructor is the moment publishers and subscribers actually register with ROS. No
   `main`, no node — and the failure reads `AttributeError: module 'x' has no attribute
   'main'`, which means the import *worked* and the function simply wasn't there.
2. **Without `spin()` the process just exits.** `main()` would build the node and return.
   Spin is the infinite loop that asks "has a message arrived, has a timer fired?" and calls
   your callbacks. Callbacks never run without it.
3. **Nodes never call each other.** No shared memory, no function calls, no guarantee the
   other end exists. Two blind processes connected only by DDS moving messages into a queue.

That last one has a sharp edge: **publishing to a topic nobody subscribes to is legal and
silent.** Observed 2026-10-03 — a sender logged 70 seconds of successful publishing while
its receiver had died on startup. No warning anywhere. This is the number one cause of "my
robot won't move": a controller publishing `/cmd_vel` at 20 Hz into nothing.

Measured, same date, both processes on one laptop: publish to callback took **129 µs**. Over
Wi-Fi to the robot that becomes milliseconds with jitter — the real content of sim-to-real
question 1.

## The rules, and why each one exists

### 1. Callbacks store. Timers compute.
A subscription callback runs whenever a message lands — at the sensor's rate, which you
don't control, and which differs between sim and the real robot. If you put your control
law in the `/odom` callback, your control rate becomes whatever the odometry rate happens
to be, and your code behaves differently on the robot than in Gazebo for reasons you will
never find.

So: `odom_cb` assigns `self.x`, `self.y`, `self.yaw` and returns. The timer reads them and
does the maths. Your loop rate is then a number you chose and can state out loud.

This is the single most common beginner mistake and the easiest to be asked about.

### 2. Guard on "I haven't been told yet"
`self.goal = None` in `__init__`, and `if self.goal is None: return` at the top of the
timer. Without it, the node starts driving toward (0, 0) the instant it launches, because
zero is what an uninitialised float is. Your old run sheet documents exactly this as
"robot drives off the moment the node starts — normal". It is not normal. It is a missing
guard.

### 3. Clamp every command before you publish it
```python
v     = max(-V_MAX, min(V_MAX, v))
omega = max(-W_MAX, min(W_MAX, omega))
```
...except clamping them *independently* bends the path, because the robot then turns at a
different ratio to its forward speed than you asked for. Scale both by one common factor
instead:
```python
scale = min(1.0, V_MAX / abs(v) if v else 1.0, W_MAX / abs(omega) if omega else 1.0)
v, omega = v * scale, omega * scale
```
This is your own finding from last time and it is good work: unclamped, the controller
commanded 0.92 m/s and 19.95 rad/s and flipped the robot in Gazebo. Keep it.

### 4. Stop the robot on the way out
`rclpy.spin()` ends on Ctrl-C. If the last thing you published was "drive forward", the
robot keeps driving forward, because nothing told it otherwise. A `/cmd_vel` message is a
standing order, not a pulse. Hence the `try/except KeyboardInterrupt/finally: node.stop()`
in slot 8. This is a safety rule on real hardware, not a nicety.

### 5. Normalise every angle difference
```python
ang = (ang + np.pi) % (2 * np.pi) - np.pi
```
Without it, a robot at +179° being asked to go to −179° computes a 358° error and spins
almost all the way round instead of 2° the short way. Any time you subtract two angles,
this line follows it.

### 6. Quaternion to yaw is four lines, inline
```python
q = msg.pose.pose.orientation
yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
```
Odometry reports orientation as a quaternion. A ground robot only has yaw. This does not
need a `geometry.py`, a helper class, or a dependency on `tf_transformations`. It needs
four lines in the callback that uses it.

### 7. Filter the laser scan before you trust it
A LaserScan's `ranges` contains `0.0` and `inf` for beams that hit nothing or hit something
closer than `range_min`. `np.argmin(ranges)` on raw data will happily return a beam reading
0.0 and tell you the wall is on top of you. Always:
```python
r = np.array(msg.ranges)
valid = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
```
...and only then take the minimum, over `r[valid]`. Then check `valid.any()` before using
the result, because sometimes no beam is valid at all.

Get the angle of beam `i` from `msg.angle_min + i * msg.angle_increment`. Do **not** use
`np.linspace(angle_min, angle_max, len(ranges))` — a 360-beam scan covers 360°, so the last
beam is one increment *before* `angle_max`, and linspace spreads them one increment too
wide. The error is small and it is still an error.

### 8. Parameters, not magic numbers
```python
self.declare_parameter('wall_distance', 0.4)
self.wall_distance = self.get_parameter('wall_distance').value
```
A number typed in the middle of your maths is a number you cannot change without editing
code, and cannot tune in a lab session without rebuilding. Declared at the top, you can
override it from the command line:
```bash
python3 wall_follower.py --ros-args -p wall_distance:=0.35
```
This is **not** the same as a YAML file. The default lives in the code, one line from the
maths that uses it, with a comment saying why that value. That was the instructor's actual
objection: not "parameters are bad" but "I cannot find them".

### 9. `self.get_logger().info()`, never `print()`
The logger stamps the message with the node name and time and goes through ROS. `print()`
goes to whichever terminal happens to own stdout, with nothing identifying it. When five
nodes are running, unlabelled output is worthless.

Log the things that are *evidence*: "NEW SETPOINT (0.5, 0.0)", "wall acquired at x=.. y=..",
"LAP COMPLETE: 14.2 m in 142 s". Those lines go in your report. Do not log every tick —
at 20 Hz you will not be able to read your own terminal.

### 10. Import only what you use
Four of your classmate's files start with `from std_msgs.msg import String` and never use a
String. It is harmless and it is noise, and noise is what you were marked down for. If you
delete an experiment, delete its import.

---

## How a node gets runnable

Four steps, in order, every time. Learn this loop; it is most of the tutorial's point.

1. **Register** it in `setup.py`, under `entry_points`:
   ```python
   'console_scripts': [
       'controller_node = r7021e_lab1.position_controller:main',
   ],
   ```
   Read that as: *the command `controller_node` runs `main()` in
   `r7021e_lab1/position_controller.py`.* The name on the left is what you type; it does not
   have to match the filename, and when it doesn't, you will confuse yourself. Make it match.

2. **Build**, from the workspace root (the folder containing `src/`):
   ```bash
   colcon build --symlink-install
   ```
   `--symlink-install` means later edits to the `.py` take effect without rebuilding. You
   still must rebuild after changing `setup.py`, `package.xml`, or adding a new file.

3. **Source**, in every terminal that wants to see it:
   ```bash
   source install/setup.bash
   ```
   Skip this and you get `Package 'r7021e_lab1' not found`. It is per-terminal, every time.

4. **Run**:
   ```bash
   ros2 run r7021e_lab1 controller_node
   ```

And because every node in this folder has a shebang and the executable bit, there is always
the no-build escape hatch:
```bash
python3 ~/R7021E-group6/lab1/ros2_ws/src/r7021e_lab1/r7021e_lab1/position_controller.py
```
Same node, same behaviour, no build, no sourcing. Both still need `ROS_DOMAIN_ID` exported.

---

## The design idea worth stealing (the concept, not the code)

Your classmate's Lab 1 is four nodes totalling **300 lines**, and they all meet at one
topic:

```
   you, by hand   ─┐
   position_sender ├─→  /new_position  ──→  controller  ──→  /cmd_vel  ──→  robot
   wall_follow    ─┘                           ↑
                                            /odom
   /scan ──→ closest_wallpt ──→ /wall_pt ──→ wall_follow
```

One controller. Three different things that publish a goal to it. Task A is you typing a
goal; Task B is a node walking a goal around a figure-8; Task C is a node that watches the
nearest wall point and publishes a goal 0.4 m off the wall and ahead along it — so wall
following becomes "chase a carrot", using the same controller, with no new control law at all.

**That is the lesson.** Not the formulas — the fact that *the topic is the interface*. Each
node does one job, reads one or two topics, writes one. You can run any of them alone. You
can test the controller with a hand-typed goal before the trajectory node exists. You can
explain each in a sentence.

Your old Lab 1 did the same three tasks with 2 packages, 5 YAML files, 14 modules and
2505 lines. Eight times the code, and when you were asked to explain it you couldn't.
