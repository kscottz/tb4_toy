import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

class ToyNode(Node):

    def __init__(self):
        super().__init__('toy_node')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.position_x = 0.00
        self.position_y = 0.00
        self.publisher = self.create_publisher(Twist, '/cmd_vel',10)
        self.srv = self.create_service(Trigger, 'do_loopy', self.do_loopy_callback)
        
    def odom_callback(self, msg):
        self.get_logger().info("I'm at ({0},{1})".format(msg.pose.pose.position.x,
                                                         msg.pose.pose.position.y))
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        
    def do_loopy_callback(self, request, response):
        self.get_logger().info('Incoming request')
        self.get_logger().info("Looping at ({0},{1})".format(self.position_x,
                                                             self.position_y))

        

        return response


def main(args=None):
    rclpy.init(args=args)

    toy_node = ToyNode()

    rclpy.spin(toy_node)

    # Destroy the node explicitly
    toy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
