import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Lock

class picolistener(Node):
  def __init__(self):
    super().__init__('pico_listener')
    self._lock = Lock()
    self.last_command = None

    self.subscription = self.create_subscription(
      String,
      'command',
      self.listener_callback,
      10)
    self.get_logger().info('Pico listener node started and listening for commands')

    self.create_timer(0.1, self._timer_report)

  def listener_callback(self, msg):
    with self._lock:
      self.last_command = msg.data
    self.get_logger().info(f'Received command: "{msg.data}"')

  def _timer_report(self):
    with self._lock:
      cmd = self.last_command
    if cmd is not None:
      self.get_logger().info(f'Last stored command: "{cmd}"')

def main(args=None):
  rclpy.init(args=args)
  pico_listener = picolistener()
  try:
    rclpy.spin(pico_listener)
  finally:
    pico_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
