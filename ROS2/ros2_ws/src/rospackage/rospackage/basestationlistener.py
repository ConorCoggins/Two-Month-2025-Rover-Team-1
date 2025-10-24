from rclpy.node import Node
from std_msgs.msg import Float32
from threading import Lock

class basestationlistener(Node):
  def __init__(self):
    super().__init__('base_station_listener')

    # stored sensor values (initially None)
    self.temperature = None
    self.pressure = None
    self.altitude = None
    self.acceleration = None


    self.sub_temperature = self.create_subscription(
      Float32,
      'temperature',
      self.temperature_callback,
      10)
    self.sub_pressure = self.create_subscription(
      Float32,
      'pressure',
      self.pressure_callback,
      10)
    self.sub_altitude = self.create_subscription(
      Float32,
      'altitude',
      self.altitude_callback,
      10)
    self.sub_acceleration = self.create_subscription(
      Float32,
      'acceleration',
      self.acceleration_callback,
      10)


    self.create_timer(1.0, self.timer_report)

    self.get_logger().info('Base station listener node started and listening for sensor data')

  def temperature_callback(self, msg):
    with self._lock:
      self.temperature = msg.data
    self.get_logger().info(f'Received temperature: "{msg.data}"')

  def pressure_callback(self, msg):
    with self._lock:
      self.pressure = msg.data
    self.get_logger().info(f'Received pressure: "{msg.data}"')

  def altitude_callback(self, msg):
    with self._lock:
      self.altitude = msg.data
    self.get_logger().info(f'Received altitude: "{msg.data}"')

  def acceleration_callback(self, msg):
    with self._lock:
      self.acceleration = msg.data
    self.get_logger().info(f'Received acceleration: "{msg.data}"')

  def timer_report(self):
    # read stored values safely
    with self._lock:
      t = self.temperature
      p = self.pressure
      a = self.altitude
      acc = self.acceleration

    self.get_logger().info(
      f'Stored values -> temp: {t}, pressure: {p}, altitude: {a}, accel: {acc}'
    )

def main(args=None):
  rclpy.init(args=args)
  node = basestationlistener()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()

