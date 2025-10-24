import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String

class basestationtalker(Node):
  def __init__(self):
    super().__init__('basestation_talker')
    self.publisher_ = self.create_publisher(String, 'command', 10)

  def update_motor(self, wrist_position=0, claw_position=0, upper_elbow=0, lower_elbow=0, turret_position=0):
    serial.write(f"L{lower_elbow}\n".encode())
    msg = String()
    msg.data = f",L{lower_elbow}"
    self.publisher_.publish(msg)

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

