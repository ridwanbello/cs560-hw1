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
        print("Turtle started Diamond shape...")

    def create_twist(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    def get_twist_msg(self):
        # Turn 45 degrees
        if self.time < 2:
            msg = self.create_twist(0.0, 0.785)
        # Move forward
        elif self.time >= 2 and self.time < 7:
            msg = self.create_twist(1.0, 0.0)
        # Turn 171 degrees
        elif self.time > 7 and self.time < 9:
            msg = self.create_twist(0.0, 3.0)
        # Move forward
        elif self.time>=9 and self.time < 14:
            msg = self.create_twist(1.0, 0.0)
        # Turn 97 degrees
        elif self.time >=14 and self.time < 16:
            msg = self.create_twist(0.0, 1.7)
        # Move forward
        elif self.time >= 16 and self.time < 21:
            msg = self.create_twist(1.0, 0.0)
        # Turn 78 degrees
        elif self.time >=21 and self.time <23:
            msg = self.create_twist(0.0, 1.364)
        # Move forward
        elif self.time >= 23 and self.time < 28:
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
        if self.time > 33:
            self.timer.cancel()
            print("Timer has been stopped")
            print("Turtle ended the Diamond shape.")

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
