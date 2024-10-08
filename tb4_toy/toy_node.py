import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from irobot_create_msgs.msg import HazardDetectionVector
from rcl_interfaces.msg import ParameterDescriptor

class ToyNode(Node):

    def __init__(self):
        super().__init__('toy_node')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.bump_subscription = self.create_subscription(
            HazardDetectionVector,
            '/hazard_detection',
            self.hazard_callback,
            10)
        self.bump_subscription  # prevent unused variable warning

        # Current position
        self.position_x = 0.00
        self.position_y = 0.00
        # Loop start position
        self.start_x = 0.00
        self.start_y = 0.00
        # Last Location
        self.last_x = 0.00
        self.last_y = 0.00
        # Distance traveled, distance to the goal, and gate variable
        self.distance = 0.00
        self.nearness = 0.00
        self.do_loop = False

        # Create our parameters
        self._init_params()
        
        # the publisher that will send our velocity command
        self.publisher = self.create_publisher(Twist, '/cmd_vel',10)
        self.srv = self.create_service(Trigger, 'do_loopy', self.do_loopy_callback)
        # how fast our loop will run
        timer_period = 0.1 # 10 Hz
        # Create the timer for our loop callback
        self.timer = self.create_timer(timer_period, self.loop_callback)

    def _init_params(self):
        """
        Initialize all of the parameters used by this node
        """
        self.domain = "world" # This is the namespace of our parameters
        param_list = [] # List of params to set
        
        # CREATE PARAMETERS
        # initialize the value in our class
        self.z_angular_velocity = -3.0
        # create a description of our parameter
        descript = ParameterDescriptor(description='Controls the angular velocity and a direction of a robot when running the do loopy service.')
        # tell ROS hey, we have a parameter
        self.declare_parameter('z_angular_velocity', domain, descript)
        # Set the parameter the first time
        ang_param = rclpy.parameter.Parameter('z_angular_velocity',rclpy.Parameter.Type.FLOAT,domain)
        self.get_logger().info('Initializing Toy Node Z angular velocity to: {0}'.format(self.z_angular_velocity))
        param_list.append(ang_param)        
        self.set_parameters(param_list)

    def _update_params(self):
        """
        Update all of the parameters used by this node
        """
        # get the parameter from the parameter server
        self.z_angular_velocity = self.get_parameter('z_angular_velocity').get_parameter_value().float_value
        self.get_logger().info('Toy Node updated Z angular velocity to: {0}'.format(self.z_angular_velocity))
        
        
    def odom_callback(self, msg):
        """
        Update the current robot position from odomettry
        """
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y

    def hazard_callback(self, msg):
        """
        Check for built in hazard sensors
        """
        # If we get a hazard detection
        if len(msg.detections) > 0:
            # Parse the message
            detect_type = msg.detections[0].type
            detect_frame = msg.header.frame_id
            if detect_type > 0: # We're bumping or about to bump
                # scream bloody murder
                self.get_logger().info('Got signal {0} on frame {1}.'.format(detect_type, detect_frame))
                self.get_logger().info('ABORT LOOPING')
                # Stop the loop and set velocity to zero
                self.do_loop = False
                msg = Twist()
                msg.linear.x = 0.00
                msg.linear.y = 0.00
                msg.linear.z = 0.00
                msg.angular.x = 0.00
                msg.angular.y = 0.00
                msg.angular.z = 0.00
                # Send message
                self.publisher.publish(msg)            
             
    def update_distance_and_nearness(self):
        """
        A function that keeps track of distance traveled and proximity to the goal 
        """
        # distance = sum of distance covered in each update
        self.distance += ((self.last_x-self.position_x)**2)+((self.last_y-self.position_y)**2)**0.5
        # nearness = distance between current position and start
        self.nearness = ((self.position_x-self.start_x)**2)+((self.position_y-self.start_y)**2)**0.5
        # update our last known position
        self.last_x = self.position_x
        self.last_y = self.position_y
        
    def loop_callback(self):
        """
        Main looping callback. If triggered update values and send velocity command
        """
        # if we are active
    
        
        if self.do_loop:
            # get the distance traveled and nearness to the goal
            self.update_distance_and_nearness()
            # print the current status
            self.get_logger().info("Traveled {:.2f}m, {:.2f}m to our goal.".format(self.distance,
                                                                             self.nearness))
            # if we meet our criteria stop
            if self.distance > 0.1 and self.nearness < 0.05:
                self.do_loop = False
                self.get_logger().info("Stopping Looping")
            else: # otherwise send the command
                msg = Twist()
                msg.linear.x = 0.20
                msg.linear.y = 0.00
                msg.linear.z = 0.00
                msg.angular.x = 0.00
                msg.angular.y = 0.00
                msg.angular.z = -3.0
                self.publisher.publish(msg)
        else: # updating params in the middle of an activity can have undefined behavior
            self._update_params() # only update if we're not in the middle of doing some work
        
    def do_loopy_callback(self, request, response):
        """
        Service call entry point for looping
        """
        self.get_logger().info('Incoming loop request!')
        self.get_logger().info("Our loops starts at ({:.2f},{:.2f})".format(self.position_x,
                                                                      self.position_y))
        # Set our initial position
        self.start_x = self.position_x
        self.start_y = self.position_y
        # Also set the last position
        self.last_x = self.position_x
        self.last_y = self.position_y
        # Reset the distance traveled
        self.distance = 0.00
        # Toggle the gaurd variable
        self.do_loop = not self.do_loop
        # Send a message to the service requester
        response.message = "Kicking off looping!"
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
