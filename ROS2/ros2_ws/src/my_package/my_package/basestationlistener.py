import rclpy
from rclpy.node import Node
from std_msgs.msg import Float

class basestationlistener(Node):
  def __init__(self):
    super().__init__('base_station_listener')
    self.subscription = self.create_subscription(
      Float,
      'temperature',
      self.listener_callback,
      10)
    self.subscription = self.create_subscription(
      Float,
      'pressure'
      self.listener_callback,
      10)
    self.subscription = self.create_subscription(
      Float,
      'altitude',
      self.listener_callback,
      10)
    self.subscription = self.create_subscription(
      Float,
      'acceleration',
      self.listener_callback,
      10)
    self.get_logger().info('base station listener node started and listening for sensor data')
  def listener_callback(self, msg):
    self.get_logger().info(f'Recieved: "{msg.data}"')
def main(args=None):
  rclpy.init(args=args)
  base_station_listener = basestationlistener()
  basestationlistener().destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()

