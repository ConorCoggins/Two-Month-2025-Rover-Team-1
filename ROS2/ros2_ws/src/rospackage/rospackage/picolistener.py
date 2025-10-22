import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class picolistener(Node):
  def __init__(self):
    super().__init__('pico_listener')
    self.subscription = self.create_subscription(
      String,
      'command',
      self.lisener_callback,
      10)
    self.get_logger().info('pico listener node started and listening for commands')
  def listener_callback(self, msg):
    self.get_logger().info(f'Recieved: "{msg.data}"')
def main(args=None):
  rclpy.init(args=args)
  pico_listener = picolistener()
  pico_listener.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
