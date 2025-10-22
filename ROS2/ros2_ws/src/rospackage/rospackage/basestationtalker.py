import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class basestationtalker(Node):
  def __init__(self):
    super().__init__('basestation_talker')
    self.publisher_ = self.create_publisher(String, 'command', 10)
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0

  def timer_callback(self):
    msg = String()
    msg.data = f'CMD_{self.i}'  # replace with any string command or source variable
    self.publisher_.publish(msg)
    self.get_logger().info(f'Publishing: "{msg.data}"')
    self.i += 1

def main(args=None):
  rclpy.init(args=args)
  node = basestationtalker()
  try:
    rclpy.spin(node)
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()

