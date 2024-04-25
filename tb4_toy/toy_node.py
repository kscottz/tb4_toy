import rclpy
from rclpy.node import Node


from nav_msgs.msg import Odometry
#from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String


class ToyNode(Node):

    def __init__(self):
        super().__init__('toy_node')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

    def odom_callback(self, msg):
        self.get_logger().info("I'm at ({0},{1})".format(msg.pose.pose.position.x,
                                                         msg.pose.pose.position.y))


def main(args=None):
    rclpy.init(args=args)

    toy_node = ToyNode()

    rclpy.spin(toy_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    toy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
