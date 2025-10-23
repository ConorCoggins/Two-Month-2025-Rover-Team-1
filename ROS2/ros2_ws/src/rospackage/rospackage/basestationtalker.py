import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class basestationtalker(Node):
  def __init__(self):
    super().__init__('basestation_talker')
    self.publisher_ = self.create_publisher(String, 'command', 10)


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

