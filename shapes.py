import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class HeartDrawer(Node):
    def __init__(self):
        super().__init__('heart_drawer')
        
        # --- Parameters ---
        self.path_scale = 0.1  # Scale of the heart shape
        self.path_offset_x = 5.5  # X-offset to center the heart
        self.path_offset_y = 4.5  # Y-offset to center the heart
        self.waypoint_tolerance = 0.1 # How close to get to a waypoint to consider it "reached"
        self.angular_gain = 6.0 # Proportional gain for turning speed
        self.linear_gain = 1.5 # Proportional gain for forward speed

        # --- Publisher for turtle commands ---
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        
        # --- State Variables ---
        self.current_pose = None # To store the turtle's current pose
        self.path = self.generate_heart_path()
        self.path_index = 0 # The index of the current target waypoint in the path

        self.timer = self.create_timer(0.05, self.move)
        
        self.get_logger().info("Heart Drawer node started. Waiting for first pose message...")

    def pose_callback(self, msg):
        """
        Callback function for the pose subscriber.
        This method is called every time a new pose message is received.
        It updates the turtle's current known position and orientation.
        """
        self.current_pose = msg

    def generate_heart_path(self):
        """
        Generates a list of (x, y) waypoints for a heart shape using
        well-known parametric equations.
        """
        points = []
        self.get_logger().info("Generating heart path...")
        # Iterate through angles from 0 to 360 degrees
        for t in range(0, 360, 5): # A step of 5 degrees gives enough resolution
            rad = math.radians(t)
            # Parametric equations for a heart curve
            x = 16 * math.sin(rad) ** 3
            y = 13 * math.cos(rad) - 5 * math.cos(2 * rad) - 2 * math.cos(3 * rad) - math.cos(4 * rad)
            
            # Scale and offset the points to fit nicely in the Turtlesim window
            x = x * self.path_scale + self.path_offset_x
            y = y * self.path_scale + self.path_offset_y
            points.append((x, y))
        self.get_logger().info(f"Path generated with {len(points)} waypoints.")
        return points

    def move(self):
        """
        The core control logic. This method is called by the timer.
        It calculates the required velocity to move towards the current target waypoint.
        """
        # Wait until the first pose message has been received
        if self.current_pose is None:
            return

        # Check if the drawing is complete
        if self.path_index >= len(self.path):
            self.get_logger().info("💖 Finished drawing the heart!")
            self.stop()
            self.timer.cancel() # Stop the timer callback
            # A short delay before shutdown to ensure the stop message is sent
            self.create_timer(1.0, self.shutdown_node)
            return

        # Get the current target waypoint
        target_x, target_y = self.path[self.path_index]

        # Calculate the error (distance and angle) between the turtle's current pose and the target
        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y
        distance_to_target = math.sqrt(dx**2 + dy**2)

        # If we are close enough to the target waypoint, advance to the next one
        if distance_to_target < self.waypoint_tolerance:
            self.path_index += 1
            # If we just advanced past the last point, we are done
            if self.path_index >= len(self.path):
                return
            # Don't command movement on this tick; wait for the next cycle
            # to calculate the error for the new waypoint.
            self.publisher.publish(Twist()) # Publish a zero-velocity Twist
            return

        # --- Proportional Controller ---
        twist = Twist()

        # 1. Calculate the desired angle to the target
        target_angle = math.atan2(dy, dx)
        
        # 2. Calculate the error between the desired angle and the turtle's current angle
        angle_error = target_angle - self.current_pose.theta
        
        # 3. Normalize the angle error to the range [-pi, pi] for efficient turning
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # 4. Set the angular velocity. It's proportional to the angle error.
        twist.angular.z = self.angular_gain * angle_error

        # 5. Set the linear velocity. It's proportional to the distance.
        #    We also reduce speed if the turtle needs to make a sharp turn.
        if abs(angle_error) > math.pi / 2: # If turning more than 90 degrees
            twist.linear.x = 0.0 # Stop and turn
        else:
            twist.linear.x = self.linear_gain * distance_to_target

        # Publish the calculated velocity command
        self.publisher.publish(twist)

    def stop(self):
        """Publishes a zero-velocity Twist message to stop the turtle."""
        self.get_logger().info("Sending stop command.")
        self.publisher.publish(Twist())

    def shutdown_node(self):
        """Cleanly shuts down the node."""
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = HeartDrawer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # This ensures the node is destroyed and ROS is shut down
        # even if the spin is interrupted.
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
