import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class picotalker(Node):
  def __init__(self):
    super().__init__('picotalker')

    
    self.pub_accel = self.create_publisher(Float32, 'acceleration', 10)
    self.pub_temp = self.create_publisher(Float32, 'temperature', 10)
    self.pub_pressure = self.create_publisher(Float32, 'pressure', 10)
    self.pub_altitude = self.create_publisher(Float32, 'altitude', 10)

    
    self.acceleration = 0.0
    self.temperature = 0.0
    self.pressure = 0.0
    self.altitude = 0.0

    timer_period = 1.0 
    self.create_timer(timer_period, self.timer_callback)
    self.get_logger().info('Pico talker node started publishing sensor data')

  def timer_callback(self):
    
    msg = Float32()
    msg.data = float(self.acceleration)
    self.pub_accel.publish(msg)
    self.get_logger().info(f'Published acceleration: {msg.data}')

    msg = Float32()
    msg.data = float(self.temperature)
    self.pub_temp.publish(msg)
    self.get_logger().info(f'Published temperature: {msg.data}')

    msg = Float32()
    msg.data = float(self.pressure)
    self.pub_pressure.publish(msg)
    self.get_logger().info(f'Published pressure: {msg.data}')

    msg = Float32()
    msg.data = float(self.altitude)
    self.pub_altitude.publish(msg)
    self.get_logger().info(f'Published altitude: {msg.data}')

def main(args=None):
  rclpy.init(args=args)
  pico_talker = picotalker()
  try:
    rclpy.spin(pico_talker)
  finally:
    pico_talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
