import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point
import time

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Subscription to ground truth pose
        self.gt_pose_sub = self.create_subscription(
            Pose,
            '/drone/gt_pose',
            self.pose_callback,
            1)

        self.gt_pose = None

        # Publisher for sending velocity commands
        self.command_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)

        # Timer for periodic execution
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Starting point for the drone
        self.starting_point = Point(x=0.0, y=0.0, z=0.0)

        # Time tracking for flight stages
        self.start_time = time.time()

        # Flight stage initialization
        self.flight_stage = 0

    def pose_callback(self, data):
        # Callback for ground truth pose
        self.gt_pose = data

    def timer_callback(self):
        if self.gt_pose is not None:
            # Timer callback for periodic execution

            elapsed_time = time.time() - self.start_time

            if elapsed_time >= 8.0:
                # Move to the next flight stage every 8 seconds
                self.flight_stage += 1
                self.start_time = time.time()

            # Execute the current flight stage
            self.execute_flight_stage()

    def execute_flight_stage(self):
        # Method to define the drone's behavior for each flight stage
        cmd = Twist()

        if self.flight_stage == 0:
            # Ascend by 1 meter (upwards)
            cmd.linear.z = 6.0
            cmd.linear.x = 0.0
        elif self.flight_stage == 1:
            # Move 2 meters to the left
            cmd.linear.z = 6.0
            cmd.linear.x = 0.0
        elif self.flight_stage == 2:
            # Move 2 meters downwards
            cmd.linear.z = 6.0
            cmd.linear.x = 3.0
        elif self.flight_stage == 3:
            # Move 2 meters to the right
            cmd.linear.z = 3.0
            cmd.linear.x = 3.0
        elif self.flight_stage == 4:
            # Return to the starting point
            cmd.linear.z = 3.0
            cmd.linear.x = 0.0
        elif self.flight_stage == 5:
            # Land back on the ground
            cmd.linear.z = 0.0
            cmd.linear.x = 0.0

        # Send the velocity command
        self.command_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)

    try:
        # Initialize the node and spin it
        node = DroneController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Interrupted by the user. Closing.")
    finally:
        # Shutdown the node when done
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
