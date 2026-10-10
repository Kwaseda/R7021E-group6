#!/usr/bin/env python3
"""Next-best-view exploration: grow one RRT* tree, drive to the leaf that sees most frontier."""

import math
import time

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from visualization_msgs.msg import Marker

DEFAULTS = {
    'inflation_radius': 0.105,   # m, Burger radius
    'iterations': 1500,          # RRT* samples per plan
    'max_iterations': 8000,      # keep growing up to this if no leaf sees a frontier
    'step_size': 0.30,           # m
    'rewire_radius': 0.60,       # m
    'info_radius': 0.75,         # m, short laser range for the gain
    'info_weight': 0.10,         # m per frontier cell
    'turn_cost': 0.15,           # m per rad
    'arrival_tolerance': 0.20,   # m
    'stall_timeout': 8.0,        # s without 0.05 m of progress
    'max_path_age': 20.0,        # s, then plan again
    'max_empty_cycles': 10,      # plans in a row with no frontier leaf, then stop
    'seen_radius': 0.50,         # m, frontier this near a spot the robot stood on stops counting
}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Grid:
    """OccupancyGrid as a numpy array. Row = y, column = x."""

    def __init__(self, msg):
        self.res = msg.info.resolution
        self.ox, self.oy = msg.info.origin.position.x, msg.info.origin.position.y
        self.h, self.w = msg.info.height, msg.info.width
        self.data = np.array(msg.data, dtype=np.int16).reshape(self.h, self.w)

    def cell(self, x, y):
        return int(math.floor((y - self.oy) / self.res)), int(math.floor((x - self.ox) / self.res))

    def world(self, r, c):
        return self.ox + (c + 0.5) * self.res, self.oy + (r + 0.5) * self.res


def free_mask(grid, radius):
    """True where the robot centre may go: known free, and not within radius of a wall."""
    occ = grid.data > 50
    k = int(math.ceil(radius / grid.res))
    pad = np.pad(occ, k)
    grown = np.zeros_like(occ)
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            if dy * dy + dx * dx <= k * k:                  # a disc of k cells
                grown |= pad[k + dy:k + dy + grid.h, k + dx:k + dx + grid.w]
    return (grid.data >= 0) & (grid.data <= 50) & ~grown


def segment_free(mask, grid, a, b):
    n = max(1, int(math.dist(a, b) / (grid.res / 2)))
    for i in range(n + 1):
        t = i / n
        r, c = grid.cell(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        if not (0 <= r < grid.h and 0 <= c < grid.w and mask[r, c]):
            return False
    return True


def rrt_star(mask, grid, root, prm, rng, enough):
    """Grow an RRT* tree from root over free space. Stop early when enough(nodes) says so."""
    nodes, parent, cost = [root], [-1], [0.0]
    free = np.argwhere(mask)
    if len(free) == 0:
        return np.array(nodes), parent, cost
    for it in range(prm['max_iterations']):
        if it >= prm['iterations'] and it % 500 == 0 and enough(np.array(nodes), parent):
            break
        r, c = free[rng.integers(len(free))]
        sample = grid.world(r, c)
        pts = np.array(nodes)
        near_i = int(np.argmin(np.hypot(pts[:, 0] - sample[0], pts[:, 1] - sample[1])))
        d = math.dist(nodes[near_i], sample)
        if d < 1e-6:
            continue
        s = min(1.0, prm['step_size'] / d)
        new = (nodes[near_i][0] + s * (sample[0] - nodes[near_i][0]),
               nodes[near_i][1] + s * (sample[1] - nodes[near_i][1]))
        if not segment_free(mask, grid, nodes[near_i], new):
            continue
        dist = np.hypot(pts[:, 0] - new[0], pts[:, 1] - new[1])
        near = np.flatnonzero(dist <= prm['rewire_radius'])
        best, best_cost = near_i, cost[near_i] + math.dist(nodes[near_i], new)
        for j in near:                                   # choose the cheapest parent
            cj = cost[j] + dist[j]
            if cj < best_cost and segment_free(mask, grid, nodes[j], new):
                best, best_cost = int(j), cj
        nodes.append(new)
        parent.append(best)
        cost.append(best_cost)
        k = len(nodes) - 1
        for j in near:                                   # rewire through the new node
            cj = best_cost + dist[j]
            if cj < cost[j] and segment_free(mask, grid, new, nodes[j]):
                parent[j], cost[j] = k, cj
    return np.array(nodes), parent, cost


def branch(nodes, parent, i):
    out = []
    while i != -1:
        out.append((float(nodes[i, 0]), float(nodes[i, 1])))
        i = parent[i]
    return out[::-1]


class NavigationNode(Node):

    def __init__(self):
        super().__init__('path_planner_node')
        self.p = {k: self.declare_parameter(k, v).value for k, v in DEFAULTS.items()}
        self.path_pub = self.create_publisher(Path, 'path', 10)
        self.tree_pub = self.create_publisher(Marker, 'rrt_tree', 10)
        self.goal_pub = self.create_publisher(Marker, 'goal_marker', 10)
        self.inflated_pub = self.create_publisher(OccupancyGrid, 'inflated_map', 10)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, 10)
        self.create_subscription(OccupancyGrid, 'frontiers', self.on_frontiers, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.on_click, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.map = self.frontiers = self.goal = self.clicked = None
        self.rng = np.random.default_rng()
        self.empty_cycles, self.planned_once, self.done = 0, False, False
        self.goal_time = self.stall_ref = None
        self.visited = []                                # spots the robot reached or stalled at
        self.create_timer(1.0, self.tick)

    # callbacks only store
    def on_map(self, msg):
        self.map = msg

    def on_frontiers(self, msg):
        self.frontiers = msg

    def on_click(self, msg):
        self.clicked = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'goal from RViz: ({self.clicked[0]:.2f}, {self.clicked[1]:.2f})')

    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time(),
                                                timeout=Duration(seconds=0.5))
        except Exception:
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    def tick(self):
        if self.done or self.map is None or self.frontiers is None:
            return
        pose = self.pose()
        if pose is None:
            return
        now = self.get_clock().now()
        xy = pose[:2]
        if self.goal is not None and self.clicked is None:
            age = (now - self.goal_time).nanoseconds * 1e-9
            if math.dist(xy, self.goal) < self.p['arrival_tolerance']:
                self.get_logger().info(f'arrived at ({self.goal[0]:.2f}, {self.goal[1]:.2f})')
            elif math.dist(xy, self.stall_ref[:2]) > 0.05:
                self.stall_ref = (xy[0], xy[1], now)
                if age < self.p['max_path_age']:
                    return
                self.get_logger().info('path is old, planning again')
            elif (now - self.stall_ref[2]).nanoseconds * 1e-9 > self.p['stall_timeout']:
                self.get_logger().warn('stalled, planning again')
                self.visited.append(self.goal)           # its frontier stops counting too
            else:
                return
            self.visited.append(xy)
            self.goal = None
        self.plan(pose, now)

    def plan(self, pose, now):
        t0 = time.perf_counter()
        prm = self.p
        grid = Grid(self.map)
        mask = free_mask(grid, prm['inflation_radius'])
        self.publish_inflated(grid, mask)
        root = pose[:2]
        r, c = grid.cell(*root)
        if not (0 <= r < grid.h and 0 <= c < grid.w and mask[r, c]):
            cells = np.argwhere(mask)                    # robot inside the padding
            if len(cells) == 0:
                return
            wx, wy = grid.world(cells[:, 0], cells[:, 1])
            i = int(np.argmin(np.hypot(wx - root[0], wy - root[1])))
            root = (float(wx[i]), float(wy[i]))
        fg = Grid(self.frontiers)
        fc = np.argwhere(fg.data > 50)
        fx, fy = fg.world(fc[:, 0], fc[:, 1])
        for vx, vy in self.visited:                      # it stayed although we stood near it
            keep = np.hypot(fx - vx, fy - vy) > prm['seen_radius']
            fx, fy, fc = fx[keep], fy[keep], fc[keep]

        def gain(nodes):
            if len(fc) == 0:
                return np.zeros(len(nodes), dtype=int)
            d = np.hypot(nodes[:, 0, None] - fx[None, :], nodes[:, 1, None] - fy[None, :])
            return (d <= prm['info_radius']).sum(axis=1)

        def leaves_of(parent):
            has_child = np.zeros(len(parent), dtype=bool)
            for p in parent:
                if p >= 0:
                    has_child[p] = True
            return np.flatnonzero(~has_child)

        if self.clicked is not None:                     # Task 2: plan to a known goal
            goal, self.clicked = self.clicked, None
            nodes, parent, cost = rrt_star(mask, grid, root, prm, self.rng,
                                           lambda n, p: np.min(np.hypot(
                                               n[:, 0] - goal[0], n[:, 1] - goal[1])) < 0.15)
            d = np.hypot(nodes[:, 0] - goal[0], nodes[:, 1] - goal[1])
            ok = np.flatnonzero(d < 0.15)
            if len(ok) == 0:
                self.get_logger().info(f'goal ({goal[0]:.2f}, {goal[1]:.2f}) not reachable')
                return
            best = int(ok[np.argmin(np.array(cost)[ok])])  # shortest branch to the goal
            info, h, turn = 0, cost[best], 0.0
        else:                                            # Task 4: next best view
            nodes, parent, cost = rrt_star(mask, grid, root, prm, self.rng,
                                           lambda n, p: gain(n[leaves_of(p)]).max() > 0)
            leaves = leaves_of(parent)
            leaves = leaves[np.hypot(nodes[leaves, 0] - pose[0], nodes[leaves, 1] - pose[1])
                            > prm['arrival_tolerance']]
            g = gain(nodes[leaves]) if len(leaves) else np.array([])
            if len(g) == 0 or g.max() == 0:
                if self.planned_once:
                    self.empty_cycles += 1
                    if self.empty_cycles >= prm['max_empty_cycles']:
                        self.finish(pose)
                return
            best, h, info, turn = None, math.inf, 0, 0.0
            for i, gi in zip(leaves, g):
                if gi == 0:
                    continue
                b = branch(nodes, parent, int(i))
                first = math.atan2(b[1][1] - b[0][1], b[1][0] - b[0][0]) if len(b) > 1 else 0
                tr = abs(math.atan2(math.sin(first - pose[2]), math.cos(first - pose[2])))
                hi = cost[i] - prm['info_weight'] * gi + prm['turn_cost'] * tr
                if hi < h:
                    best, h, info, turn = int(i), hi, int(gi), tr
        path = branch(nodes, parent, best)
        if math.dist(path[0], pose[:2]) > 1e-6:
            path = [pose[:2]] + path
        self.goal = path[-1]
        self.goal_time = now
        self.stall_ref = (pose[0], pose[1], now)
        self.empty_cycles, self.planned_once = 0, True
        self.publish(path, nodes, parent)
        known = np.count_nonzero(grid.data >= 0) * grid.res ** 2
        self.get_logger().info(
            f't={now.nanoseconds * 1e-9:.1f} goal=({self.goal[0]:.2f}, {self.goal[1]:.2f}) '
            f'H={h:.2f} L={cost[best]:.2f} I={info} turn={turn:.2f} nodes={len(nodes)} '
            f'known={known:.1f}m2 plan_time={time.perf_counter() - t0:.2f}s')

    def finish(self, pose):
        self.done = True
        self.publish([pose[:2]], None, None)              # a one-point path holds the robot still
        self.get_logger().info('exploration finished')

    # publishing
    def publish(self, points, nodes, parent):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        dense = [points[0]]
        for a, b in zip(points, points[1:]):                 # a point every 0.10 m
            n = max(1, int(math.ceil(math.dist(a, b) / 0.10)))
            dense += [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
                      for i in range(1, n + 1)]
        for x, y in dense:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)
        if nodes is None:
            return
        tree = Marker()
        tree.header = msg.header
        tree.ns, tree.type, tree.action = 'rrt_tree', Marker.LINE_LIST, Marker.ADD
        tree.pose.orientation.w, tree.scale.x = 1.0, 0.01
        tree.color.g, tree.color.b, tree.color.a = 0.8, 0.8, 0.6
        for i in range(1, len(nodes)):
            for j in (i, parent[i]):
                tree.points.append(Point(x=float(nodes[j, 0]), y=float(nodes[j, 1])))
        self.tree_pub.publish(tree)
        goal = Marker()
        goal.header = msg.header
        goal.ns, goal.type, goal.action = 'goal', Marker.SPHERE, Marker.ADD
        goal.pose.position.x, goal.pose.position.y = points[-1]
        goal.pose.orientation.w = 1.0
        goal.scale.x = goal.scale.y = goal.scale.z = 0.2
        goal.color.r, goal.color.a = 1.0, 1.0
        self.goal_pub.publish(goal)

    def publish_inflated(self, grid, mask):
        out = OccupancyGrid()
        out.header, out.info = self.map.header, self.map.info
        out.data = np.where((grid.data >= 0) & ~mask, 100, 0).astype(np.int8).ravel().tolist()
        self.inflated_pub.publish(out)


def main():
    rclpy.init()
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
