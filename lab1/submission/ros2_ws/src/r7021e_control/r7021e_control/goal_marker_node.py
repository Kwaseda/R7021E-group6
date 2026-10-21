"""Republish the current goal on /new_position as an RViz-drawable marker.

    /new_position  geometry_msgs/Pose         the goal the controller chases
    /goal_marker   visualization_msgs/Marker  the same goal, drawable in RViz

/new_position is a bare geometry_msgs/Pose: no header, so no frame and no
timestamp, and RViz has no display that can draw one directly. The topic name and
type are fixed by the course, so the message itself can't change -- this node
republishes it as something RViz can draw instead.

Its own node rather than a branch inside controller_node, for the same reason
scan_monitor_node is: the control loop has no use for a drawable marker.

Drawn in the odom frame, since /new_position carries no frame of its own and the
controller already treats it as being in the odometry frame.
"""

from geometry_msgs.msg import Pose
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker


class GoalMarkerNode(Node):
    """Publishes a Marker at the position most recently seen on /new_position."""

    def __init__(self) -> None:
        super().__init__('goal_marker_node')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('marker_scale', 0.08)

        self.frame_id = self.get_parameter('frame_id').value
        self.marker_scale = self.get_parameter('marker_scale').value

        self.create_subscription(Pose, '/new_position', self.on_goal, 10)
        self.marker_pub = self.create_publisher(Marker, '/goal_marker', 10)

        self.get_logger().info(
            'goal_marker_node up. drawing /new_position as a %.2f m sphere in %s'
            % (self.marker_scale, self.frame_id)
        )

    def on_goal(self, msg: Pose) -> None:
        """Redraw the marker at the new goal position."""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = msg.position.x
        marker.pose.position.y = msg.position.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.scale.z = self.marker_scale
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        # No lifetime set (defaults to zero, "forever"): stays visible until the
        # next goal replaces it.
        self.marker_pub.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalMarkerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy raises ExternalShutdownException on Ctrl-C, not KeyboardInterrupt.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
