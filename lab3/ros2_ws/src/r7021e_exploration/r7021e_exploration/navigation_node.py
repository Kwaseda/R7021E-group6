#!/usr/bin/env python3
"""Navigation node: pick a frontier, plan to it with RRT*, publish the path."""

import math
import time
from typing import List, Optional, Tuple

from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
import tf2_ros
from visualization_msgs.msg import Marker

# Every value here is a ROS parameter. Change it with --ros-args -p name:=value.
DEFAULTS = {
    'map_topic': 'map',
    'frontier_topic': 'frontiers',
    'path_topic': 'path',
    'global_frame': 'map',
    'base_frame': 'base_link',
    'tf_timeout_sec': 0.5,
    # goal handling
    'replan_period': 1.0,        # s between planning cycles
    'arrival_tolerance': 0.20,   # m, goal reached
    'stall_distance': 0.05,      # m the robot must move ...
    'stall_timeout': 8.0,        # ... within this many seconds
    'max_path_age': 20.0,        # s, then plan again
    'max_empty_cycles': 10,      # cycles with no goal before we stop
    'retire_radius': 0.30,       # m around a crossed-off goal
    # map
    'inflation_radius': 0.105,   # m, robot radius (the planner treats the robot as a point)
    'occupied_threshold': 50,
    'waypoint_spacing': 0.10,    # m between points on the published path
    # RRT*
    'max_iterations': 1500,
    'retry_iterations': 15000,
    'step_size': 0.30,
    'goal_bias': 0.10,
    'goal_tolerance': 0.15,
    'rewire_radius': 0.60,
    'extra_iterations': 200,     # iterations to keep going after the first solution
    # frontier scoring: H = length - info_weight * I + turn_cost * turn
    'min_cluster_size': 5,
    'max_candidates': 6,
    'info_radius': 0.75,         # m, count frontier cells inside this radius
    'info_weight': 0.10,         # m per frontier cell
    'turn_cost': 0.15,           # m per rad
}


def quat_to_yaw(q: Quaternion) -> float:
    """Extract planar yaw (rad) from a Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert planar yaw (rad) to Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def wrap(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# ----------------- Map helpers (no ROS calls, so they can be tested alone) -----------------

class Grid:
    """An OccupancyGrid as a numpy array. Row = y, column = x, origin yaw = 0."""

    def __init__(self, msg: OccupancyGrid) -> None:
        self.res = msg.info.resolution
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y
        self.h = msg.info.height
        self.w = msg.info.width
        self.data = np.array(msg.data, dtype=np.int16).reshape(self.h, self.w)

    def cell(self, x: float, y: float) -> Tuple[int, int]:
        """Return the (row, col) of a world point."""
        return (int(math.floor((y - self.oy) / self.res)),
                int(math.floor((x - self.ox) / self.res)))

    def world(self, row, col):
        """Return the world coordinates of a cell centre. Works on arrays and floats."""
        return self.ox + (col + 0.5) * self.res, self.oy + (row + 0.5) * self.res


def shifted(a: np.ndarray, dy: int, dx: int) -> np.ndarray:
    """Return a boolean array moved by (dy, dx) cells and padded with False."""
    h, w = a.shape
    out = np.zeros_like(a)
    out[max(dy, 0):h + min(dy, 0), max(dx, 0):w + min(dx, 0)] = \
        a[max(-dy, 0):h + min(-dy, 0), max(-dx, 0):w + min(-dx, 0)]
    return out


def dilate(a: np.ndarray, k: int) -> np.ndarray:
    """Grow the True cells of a by a disc of k cells."""
    out = a.copy()
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            if (dy or dx) and dy * dy + dx * dx <= k * k:
                out |= shifted(a, dy, dx)
    return out


def planning_mask(grid: Grid, radius: float, occupied: int = 50) -> np.ndarray:
    """Return True where the robot centre may go: known free and clear of walls."""
    occ = grid.data > occupied
    has_neighbour = np.zeros_like(occ)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy or dx:
                has_neighbour |= shifted(occ, dy, dx)
    occ &= has_neighbour                       # drop single-cell scan noise
    k = int(math.ceil(radius / grid.res))      # round up: 0.105 m is 3 cells
    free = (grid.data >= 0) & (grid.data <= occupied)
    return free & ~dilate(occ, k)


def is_free(mask: np.ndarray, grid: Grid, x: float, y: float) -> bool:
    """Tell whether a world point is inside the map and True in the mask."""
    r, c = grid.cell(x, y)
    return 0 <= r < grid.h and 0 <= c < grid.w and bool(mask[r, c])


def segment_free(mask: np.ndarray, grid: Grid, p, q) -> bool:
    """Check a straight edge along its whole length, every half cell."""
    n = max(2, int(math.dist(p, q) / (0.5 * grid.res)) + 2)
    t = np.linspace(0.0, 1.0, n)
    rows = np.floor((p[1] + t * (q[1] - p[1]) - grid.oy) / grid.res).astype(int)
    cols = np.floor((p[0] + t * (q[0] - p[0]) - grid.ox) / grid.res).astype(int)
    if rows.min() < 0 or cols.min() < 0 or rows.max() >= grid.h or cols.max() >= grid.w:
        return False
    return bool(mask[rows, cols].all())


def nearest_free(mask: np.ndarray, grid: Grid, x: float, y: float, reach: float = 1.0):
    """Return the centre of the free cell nearest to (x, y), or None."""
    r0, c0 = grid.cell(x, y)
    k = int(reach / grid.res)
    r_lo, c_lo = max(r0 - k, 0), max(c0 - k, 0)
    r_hi, c_hi = max(r0 + k + 1, 0), max(c0 + k + 1, 0)
    cells = np.argwhere(mask[r_lo:r_hi, c_lo:c_hi])
    if len(cells) == 0:
        return None
    px, py = grid.world(cells[:, 0] + r_lo, cells[:, 1] + c_lo)
    i = int(np.hypot(px - x, py - y).argmin())
    return float(px[i]), float(py[i])


def reachable(mask: np.ndarray, grid: Grid, root) -> np.ndarray:
    """Return the cells of mask that connect to the root point (8 neighbours)."""
    seen = np.zeros_like(mask)
    r, c = grid.cell(*root)
    seen[r, c] = True
    stack = [(r, c)]
    while stack:
        r, c = stack.pop()
        for rr in range(max(r - 1, 0), min(r + 2, grid.h)):
            for cc in range(max(c - 1, 0), min(c + 2, grid.w)):
                if mask[rr, cc] and not seen[rr, cc]:
                    seen[rr, cc] = True
                    stack.append((rr, cc))
    return seen


# ----------------- RRT* -----------------

def rrt_star(mask, grid, start, goal, prm, rng, budget):
    """Grow an RRT* from start and return (path, nodes, parent).

    The path is the cheapest start-to-goal list of points, or None.
    The tree keeps growing after the first hit, then the shortest branch wins.
    """
    free_cells = np.argwhere(mask)
    xy = np.zeros((budget + 1, 2))
    xy[0] = start
    cost = np.zeros(budget + 1)
    parent = [-1] * (budget + 1)
    kids = [[] for _ in range(budget + 1)]
    goal = np.asarray(goal, dtype=float)
    n = 1
    hits: List[int] = []
    first_hit = 0

    def shift_cost(j, delta):
        stack = [j]
        while stack:
            i = stack.pop()
            cost[i] += delta
            stack.extend(kids[i])

    for it in range(budget):
        if hits and it >= first_hit + prm['extra_iterations']:
            break
        # sample: goal with 10 % chance, else a random known-free cell
        if rng.random() < prm['goal_bias']:
            s = goal
        else:
            r, c = free_cells[rng.integers(len(free_cells))]
            s = np.array(grid.world(r + rng.random() - 0.5, c + rng.random() - 0.5))
        # steer one step from the nearest node
        d2 = ((xy[:n] - s) ** 2).sum(axis=1)
        i = int(d2.argmin())
        dist = math.sqrt(d2[i])
        if dist < 1e-6:
            continue
        new = xy[i] + (s - xy[i]) * min(1.0, prm['step_size'] / dist)
        if not is_free(mask, grid, new[0], new[1]):
            continue
        # cheapest collision-free parent among the neighbours
        dn = np.hypot(xy[:n, 0] - new[0], xy[:n, 1] - new[1])
        near = np.flatnonzero(dn <= prm['rewire_radius'])
        near = near[np.argsort(cost[near] + dn[near])]
        best = next((j for j in near if segment_free(mask, grid, xy[j], new)), None)
        if best is None:
            continue
        k = n
        n += 1
        xy[k] = new
        parent[k] = int(best)
        cost[k] = cost[best] + dn[best]
        kids[best].append(k)
        # rewire neighbours through the new node when that is cheaper
        for j in near:
            c = cost[k] + dn[j]
            if j != best and c < cost[j] - 1e-9 and segment_free(mask, grid, new, xy[j]):
                kids[parent[j]].remove(j)
                parent[j] = k
                kids[k].append(int(j))
                shift_cost(int(j), c - cost[j])
        reached = math.hypot(new[0] - goal[0], new[1] - goal[1]) <= prm['goal_tolerance']
        if reached and segment_free(mask, grid, new, goal):
            if not hits:
                first_hit = it
            hits.append(k)

    nodes, parents = xy[:n].copy(), parent[:n]
    if not hits:
        return None, nodes, parents
    g = min(hits, key=lambda j: cost[j] + math.dist(xy[j], goal))
    path = []
    while g != -1:
        path.append((float(xy[g, 0]), float(xy[g, 1])))
        g = parent[g]
    path.reverse()
    if math.dist(path[-1], goal) > 1e-6:
        path.append((float(goal[0]), float(goal[1])))
    return path, nodes, parents


def plan_to(mask, grid, root, goal, prm, rng):
    """Run RRT*, then once more with a bigger budget if the goal was not reached."""
    for budget in (prm['max_iterations'], prm['retry_iterations']):
        path, nodes, parents = rrt_star(mask, grid, root, goal, prm, rng, budget)
        if path is not None:
            break
    return path, nodes, parents


# ----------------- Frontiers and scoring -----------------

def cluster_cells(cells: np.ndarray, min_size: int) -> List[List[int]]:
    """Group (row, col) cells into 8-connected clusters. Return lists of cell indices."""
    todo = {(int(r), int(c)): i for i, (r, c) in enumerate(cells)}
    clusters = []
    while todo:
        seed, i = todo.popitem()
        stack, group = [seed], [i]
        while stack:
            r, c = stack.pop()
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    j = todo.pop((r + dr, c + dc), None)
                    if j is not None:
                        stack.append((r + dr, c + dc))
                        group.append(j)
        if len(group) >= min_size:
            clusters.append(group)
    return clusters


def cluster_goal(points: np.ndarray, mask, grid, robot):
    """Return the cluster middle if it is free, else the free cluster point nearest the robot."""
    mid = points.mean(axis=0)
    if is_free(mask, grid, mid[0], mid[1]):
        return float(mid[0]), float(mid[1])
    ok = [p for p in points if is_free(mask, grid, p[0], p[1])]
    if not ok:
        return None
    p = min(ok, key=lambda q: math.dist(q, robot))
    return float(p[0]), float(p[1])


def path_length(path) -> float:
    """Return the length of a point list."""
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def densify(path, spacing: float):
    """Insert points so no two neighbours are further apart than spacing."""
    out = [path[0]]
    for a, b in zip(path, path[1:]):
        n = max(1, int(math.ceil(math.dist(a, b) / spacing)))
        for i in range(1, n + 1):
            t = i / n
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    return out


def score(path, yaw, fxy: np.ndarray, prm):
    """Return (H, length, info, turn) for a path. The lowest H wins."""
    length = path_length(path)
    goal = np.asarray(path[-1])
    gap = np.hypot(fxy[:, 0] - goal[0], fxy[:, 1] - goal[1])
    info = int(np.count_nonzero(gap <= prm['info_radius']))
    turn = 0.0
    if len(path) > 1:
        turn = abs(wrap(math.atan2(path[1][1] - path[0][1], path[1][0] - path[0][0]) - yaw))
    h = length - prm['info_weight'] * info + prm['turn_cost'] * turn
    return h, length, info, turn


# ----------------- The node -----------------

class PathPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('path_planner_node')

        self.p = {k: self.declare_parameter(k, v).value for k, v in DEFAULTS.items()}
        self.global_frame: str = self.p['global_frame']
        self.base_frame: str = self.p['base_frame']
        self.tf_timeout = Duration(seconds=self.p['tf_timeout_sec'])
        self.rng = np.random.default_rng()

        default_qos = QoSProfile(depth=10)

        # Publishers
        self.path_pub = self.create_publisher(Path, self.p['path_topic'], default_qos)
        self.tree_pub = self.create_publisher(Marker, 'rrt_tree', default_qos)
        self.goal_pub = self.create_publisher(Marker, 'goal_marker', default_qos)

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.p['map_topic'], self._on_map, default_qos)
        self.frontier_sub = self.create_subscription(
            OccupancyGrid, self.p['frontier_topic'], self._on_frontier, default_qos)

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State
        self._latest_map: Optional[OccupancyGrid] = None
        self._latest_frontier: Optional[OccupancyGrid] = None
        self.goal: Optional[Tuple[float, float]] = None
        self.retired: List[Tuple[float, float]] = []
        self.tree = None
        self.path_time = self.get_clock().now()
        self.stall_ref = (0.0, 0.0, self.path_time)
        self.chosen_once = False
        self.empty_cycles = 0
        self.goals_chosen = 0
        self.t_first_plan: Optional[Time] = None
        self.done = False

        self.timer = self.create_timer(self.p['replan_period'], self._tick)
        self.get_logger().info('PathPlannerNode initialized.')

    # ----------------- TF Helper -----------------

    def get_robot_pose(self, target_frame: Optional[str] = None, source_frame: Optional[str] = None
                       ) -> Optional[Tuple[float, float, float]]:
        """Look up the robot pose (x, y, yaw) in target_frame, or None after the timeout."""
        tgt = target_frame or self.global_frame
        src = source_frame or self.base_frame

        try:
            transform = self.tf_buffer.lookup_transform(
                tgt, src, Time(), timeout=self.tf_timeout
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup {tgt} <- {src} failed: {e}')
            return None

        t = transform.transform.translation
        r = transform.transform.rotation
        yaw = quat_to_yaw(r)

        return (t.x, t.y, yaw)

    # ----------------- Callbacks -----------------

    def _on_frontier(self, msg: OccupancyGrid) -> None:
        """Store the latest frontier map."""
        self._latest_frontier = msg

    def _on_map(self, msg: OccupancyGrid) -> None:
        """Store the latest map. The timer does the planning."""
        self._latest_map = msg

    def _tick(self) -> None:
        """Run once per replan_period: check the goal, plan again when needed."""
        if self.done or self._latest_map is None or self._latest_frontier is None:
            return
        pose = self.get_robot_pose()
        if pose is None:
            return
        now = self.get_clock().now()
        xy = (pose[0], pose[1])

        if self.goal is not None:
            if math.dist(xy, self.goal) < self.p['arrival_tolerance']:
                self.get_logger().info(f'arrived at ({self.goal[0]:.2f}, {self.goal[1]:.2f})')
                self._retire()
            else:
                sx, sy, st = self.stall_ref
                if math.dist(xy, (sx, sy)) > self.p['stall_distance']:
                    self.stall_ref = (xy[0], xy[1], now)
                elif (now - st).nanoseconds * 1e-9 > self.p['stall_timeout']:
                    self.get_logger().warn('stalled, crossing off the goal')
                    self._retire()
            if self.goal is not None:
                if (now - self.path_time).nanoseconds * 1e-9 <= self.p['max_path_age']:
                    return
                self.get_logger().info('path is old, planning again')

        path = self.plan_path(pose, self._latest_map, self._latest_frontier)
        if path is not None:
            self.path_pub.publish(path)
            self._publish_markers()
            return

        if self.goal is not None:       # old path, and nothing better was found
            self._retire()
        if self.chosen_once:
            self.empty_cycles += 1
            if self.empty_cycles >= self.p['max_empty_cycles']:
                self._finish(pose)

    def _retire(self) -> None:
        """Cross the current goal off so we do not pick it again."""
        if self.goal is not None:
            self.retired.append(self.goal)
        self.goal = None

    def _finish(self, pose) -> None:
        """Stop the robot: the follower holds a one-point path at the robot pose."""
        self.done = True
        self.path_pub.publish(self._make_path([(pose[0], pose[1])]))
        elapsed = (self.get_clock().now() - self.t_first_plan).nanoseconds * 1e-9
        self.get_logger().info(
            f'exploration finished: {self.goals_chosen} goals, {elapsed:.0f} s since first plan')

    # ----------------- Planning -----------------

    def plan_path(self,
                  start: Tuple[float, float, float],
                  map_msg: OccupancyGrid,
                  frontier_msg: Optional[OccupancyGrid]):
        """Plan to the best frontier. Return a Path and set self.goal, or return None."""
        if frontier_msg is None:
            return None
        t0 = time.perf_counter()
        prm = self.p
        robot = (start[0], start[1])
        grid = Grid(map_msg)
        mask = planning_mask(grid, prm['inflation_radius'], prm['occupied_threshold'])

        # The robot is often inside the wall padding. Root the tree at the nearest free cell.
        root = robot if is_free(mask, grid, robot[0], robot[1]) \
            else nearest_free(mask, grid, robot[0], robot[1])
        if root is None:
            self.get_logger().warn('no free cell near the robot')
            return None
        mask = reachable(mask, grid, root)    # a goal must connect to the robot

        fgrid = Grid(frontier_msg)
        cells = np.argwhere(fgrid.data > 50)
        if len(cells) == 0:
            return None
        fx, fy = fgrid.world(cells[:, 0], cells[:, 1])
        fxy = np.column_stack((fx, fy))

        goals = []
        for group in cluster_cells(cells, prm['min_cluster_size']):
            g = cluster_goal(fxy[group], mask, grid, robot)
            if g is None or math.dist(g, robot) < prm['arrival_tolerance']:
                continue
            if any(math.dist(g, r) < prm['retire_radius'] for r in self.retired):
                continue
            goals.append(g)
        goals.sort(key=lambda g: math.dist(g, robot))
        goals = goals[:prm['max_candidates']]

        best = None
        for g in goals:
            path, nodes, parents = plan_to(mask, grid, root, g, prm, self.rng)
            if path is None:
                self.get_logger().info(f'goal ({g[0]:.2f}, {g[1]:.2f}) not reachable, crossed off')
                self.retired.append(g)
                continue
            if math.dist(robot, root) > 1e-6:
                path = [robot] + path
            h, length, info, turn = score(path, start[2], fxy, prm)
            if best is None or h < best[0]:
                best = (h, g, path, nodes, parents, length, info, turn)
        if best is None:
            return None

        h, g, path, nodes, parents, length, info, turn = best
        now = self.get_clock().now()
        self.goal = g
        self.tree = (nodes, parents)
        self.path_time = now
        self.stall_ref = (robot[0], robot[1], now)
        self.chosen_once = True
        self.empty_cycles = 0
        self.goals_chosen += 1
        if self.t_first_plan is None:
            self.t_first_plan = now
        known = np.count_nonzero(grid.data >= 0) * grid.res ** 2
        self.get_logger().info(
            f't={now.nanoseconds * 1e-9:.1f} goal=({g[0]:.2f}, {g[1]:.2f}) H={h:.2f} '
            f'L={length:.2f} I={info} turn={turn:.2f} candidates={len(goals)} '
            f'known={known:.1f}m2 plan_time={time.perf_counter() - t0:.2f}s')
        return self._make_path(densify(path, prm['waypoint_spacing']))

    # ----------------- Messages -----------------

    def _make_path(self, points) -> Path:
        msg = Path()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        yaw = 0.0
        for i, pt in enumerate(points):
            if i + 1 < len(points):
                yaw = math.atan2(points[i + 1][1] - pt[1], points[i + 1][0] - pt[0])
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.orientation = yaw_to_quaternion(yaw)
            msg.poses.append(pose)
        return msg

    def _publish_markers(self) -> None:
        stamp = self.get_clock().now().to_msg()
        tree = Marker()
        tree.header.frame_id = self.global_frame
        tree.header.stamp = stamp
        tree.ns, tree.id = 'rrt_tree', 0
        tree.type, tree.action = Marker.LINE_LIST, Marker.ADD
        tree.pose.orientation.w = 1.0
        tree.scale.x = 0.01
        tree.color.g, tree.color.b, tree.color.a = 0.8, 0.8, 0.6
        nodes, parents = self.tree
        for i in range(1, len(nodes)):
            for j in (i, parents[i]):
                tree.points.append(Point(x=float(nodes[j, 0]), y=float(nodes[j, 1])))
        self.tree_pub.publish(tree)

        goal = Marker()
        goal.header = tree.header
        goal.ns, goal.id = 'goal', 0
        goal.type, goal.action = Marker.SPHERE, Marker.ADD
        goal.pose.position.x, goal.pose.position.y = self.goal
        goal.pose.orientation.w = 1.0
        goal.scale.x = goal.scale.y = goal.scale.z = 0.2
        goal.color.r, goal.color.a = 1.0, 1.0
        self.goal_pub.publish(goal)


def main() -> None:
    rclpy.init()
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
