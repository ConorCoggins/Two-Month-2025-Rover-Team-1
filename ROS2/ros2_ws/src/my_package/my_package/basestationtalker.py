import rclpy
from rclpy.node import Node
from std_msgs.msg import Float
global command

class basestationtalker(Node):
  def __init__(self):
    super().__init__('basestationtalker')
    self.publisher_ = self.create_publisher(Float, 'command', 10)
    timer_period = 0.1
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0
  def timer_callback(self):
    msg.float()
    msg.data(command)
    self.publisher_.publish(msg)
    self.get_logger().info(f'Publishing: "{msg.data}"')
    self.i += 1
def main(args=None):
  rclpy.init(args=args)
  base_station_talker = base_station_talker()
  rclpy.spin(base_station_talker)
  base_station_talker.destroy_node()
  rclpy.shutdown()
if __name__ == '__main__':
  main()

