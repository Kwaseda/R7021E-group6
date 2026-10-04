import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Receiver(Node):
    def __init__(self):
        super().__init__('receiver')

        topic_name = '/short_topic'

        self.subscriber = self.create_subscription(
            String,     # msg type
            topic_name,     
            self.msg_callback,  # callback function
            10  # queue size
        )

    def msg_callback(self, msg):
        """Callback function for incoming messages."""

        # Log the received message and the topic name
        self.get_logger().info(f'Received: "{msg.data}" from the topic: {self.subscriber.topic_name}')

def main(args=None):
    rclpy.init(args=args)   # Initialize the ROS 2 client
    receiver_node = Receiver()   # Create an instance of the Receiver node
    rclpy.spin(receiver_node)  # Keep the node alive and running until interrupted
    receiver_node.destroy_node()  # Clean up the node after spinning
    rclpy.shutdown()    # Shutdown the client

if __name__ == '__main__':
    main()  # Call the main function to run the node 