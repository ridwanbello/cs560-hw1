import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.time = 0
        print("Turtle started the Love (Heart) shape...")

    def create_twist(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    def get_twist_msg(self):
        # Turn 69 degrees
        if self.time < 2:
            msg = self.create_twist(0.0, 1.2)
        # Move forward
        elif self.time >= 2 and self.time < 8:
            msg = self.create_twist(1.0, 0.0)
        # Create a half-circle
        elif self.time >= 8 and self.time < 15:
            msg = self.create_twist(1.0, 1.0)
        # Turn 57 degrees
        elif self.time >= 15 and self.time < 21:
            msg = self.create_twist(0.0, -1.0)
        # Create another circle
        elif self.time >= 21 and self.time < 28:
            msg = self.create_twist(1.0, 1.0)
        # Turn 18 degrees to complete the shape
        elif self.time >= 28 and self.time < 30:
            msg = self.create_twist(0.0, 0.3)
        # Move forward
        elif self.time >= 30 and self.time < 38:
            msg = self.create_twist(1.0, 0.0)
        else:
            msg = self.create_twist(0.0, 0.0)
        return msg
    
    def timer_callback(self):
        msg = self.get_twist_msg()       
        self.publisher.publish(msg)
        self.time += 1
        print("time: {}".format(self.time))
        # Stop the timer when the turtle has finished its path
        if self.time > 39:
            self.timer.cancel()
            print("Timer has been stopped")
            print("Turtle ended the Love (Heart) shape.")

def main(args=None):
    rclpy.init(args=args)

    turtle_controller = TurtleController()

    rclpy.spin(turtle_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
