import rclpy
from rclpy.node import Node
from std_msgs.msg import Float
global acceleration = 0.0
global temperature = 0.0
global pressure = 0.0
global altitude = 0.0

class picotalker(Node):
  def __init__(self):
    super().__init__('picotalker')
    self.publisher_ = self.create_publisher(Float, 'acceleration', 10)
    self.publisher_ = self.create_publisher(Float, 'temperature', 10)
    self.publisher_ = self.create_publisher(Float, 'pressure', 10)
    self.publisher_ = self.create_publisher(Float, 'altitude', 10)
    timer_period = 1
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0
  def timer_callback(self):
    msg.float()
    msg.data(acceleration, temperature, pressure, altitude)
    self.publisher_.publish(msg)
    self.get_logger().info(f'Publishing: "{msg.data}"')
    self.i += 1
def main(args=None):
  rclpy.init(args=args)
  pico_talker = picotalker()
  rclpy.spin(pico_talker)
  pico_talker.destroy_node()
  rclpy.shutdown()
if __name__ == '__main__':
  main()
