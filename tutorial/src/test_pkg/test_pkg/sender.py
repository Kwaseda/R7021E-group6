import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Sender(Node):
    def __init__(self):
        super().__init__('sender')

        topic_name = '/short_topic'

        self.declare_parameter('message', 'Test message')

        self.publisher = self.create_publisher(String, topic_name, 10)

        timer_period = 1.0  # seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0

    def timer_callback(self):
        msg = String()
        # msg.data = f'Hello, world! {self.i}'

        msg.data = self.get_parameter('message').get_parameter_value().string_value

        self.publisher.publish(msg)

        # Log the message being published
        self.get_logger().info(f'Publishing: "{msg.data}" in the topic: {self.publisher.topic_name}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)   # Initialize the ROS 2 client
    sender_node = Sender()   # Create an instance of the Sender node
    rclpy.spin(sender_node)  # Keep the node alive and running until interrupted
    sender_node.destroy_node()  # Clean up the node after spinning
    rclpy.shutdown()    # Shutdown the client

if __name__ == '__main__':
    main()  # Call the main function to run the node