import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import sys

class HolonomicRobotController(Node):
    def __init__(self):
        super().__init__('holonomic_robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_velocity_x = 0.0
        self.linear_velocity_y = 0.0
        self.angular_velocity_z = 0.0
        self.max_linear_speed = 1. # Max linear speed (m/s)
        self.max_angular_speed = 1.5  # Max angular speed (rad/s) 
        self.velocity_increment = 99999  # Speed increase per keypress

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity_x
        msg.linear.y = self.linear_velocity_y
        msg.linear.z = 0.0
        msg.angular.z = self.angular_velocity_z
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published velocity: linear=({self.linear_velocity_x}, {self.linear_velocity_y}), angular={self.angular_velocity_z}')

    def update_velocity(self, keys):
        if keys[pygame.K_a]:
            self.linear_velocity_y = min(self.linear_velocity_y + self.velocity_increment, self.max_linear_speed)
        elif keys[pygame.K_d]:
            self.linear_velocity_y = max(self.linear_velocity_y - self.velocity_increment, -self.max_linear_speed)
        else:
            self.linear_velocity_y = 0.0

        if keys[pygame.K_s]:
            self.linear_velocity_x = max(self.linear_velocity_x - self.velocity_increment, -self.max_linear_speed)
        elif keys[pygame.K_w]:
            self.linear_velocity_x = min(self.linear_velocity_x + self.velocity_increment, self.max_linear_speed)
        else:
            self.linear_velocity_x = 0.0

        if keys[pygame.K_RIGHT]:
            self.angular_velocity_z = max(self.angular_velocity_z - self.velocity_increment, -self.max_angular_speed)
        elif keys[pygame.K_LEFT]:
            self.angular_velocity_z = min(self.angular_velocity_z + self.velocity_increment, self.max_angular_speed)
        else:
            self.angular_velocity_z = 0.0

        # Compute the magnitude of the x-y velocity
        velocity_magnitude = (self.linear_velocity_x ** 2 + self.linear_velocity_y ** 2) ** 0.5

        # If the magnitude exceeds the maximum allowed velocity, scale it down
        if velocity_magnitude > self.max_linear_speed:
            scaling_factor = self.max_linear_speed / velocity_magnitude
            self.linear_velocity_x *= scaling_factor
            self.linear_velocity_y *= scaling_factor


def main(args=None):
    # Initialize pygame and ROS2
    pygame.init()
    rclpy.init(args=args)

    # Create the controller node
    robot_controller = HolonomicRobotController()

    # Set up the display
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("Holonomic Robot Controller")

    # Main loop
    try:
        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    rclpy.shutdown()
                    sys.exit()

            keys = pygame.key.get_pressed()
            robot_controller.update_velocity(keys)
            robot_controller.send_velocity_command()

            # Run ROS2 communication
            rclpy.spin_once(robot_controller, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
