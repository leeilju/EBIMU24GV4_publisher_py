import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import serial
import math

PI = 3.141592

def split(data, delimiter):
    return data.split(delimiter)

class IMUNode(Node):

    def __init__(self):
        super().__init__('imu')
        self.publish_rate = 30
        self.ser = serial.Serial()

        self.imu_pub0 = self.create_publisher(Imu, '/ebimu', self.publish_rate)
        self.create_timer(1/3, self.process_data)

        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        q.w = 1.0

        try:
            self.ser.port = '/dev/ttyUSB0'
            self.ser.baudrate = 115200
            self.ser.timeout = 1.0 / self.publish_rate
            self.ser.open()
        except serial.SerialException as e:
            self.get_logger().error(f"Unable to open port: {e}")
            return

        if self.ser.is_open:
            self.get_logger().info("Serial Port initialized")
        else:
            return

    def write_callback(self, msg):
        self.get_logger().info(f"Writing to serial port: {msg.data}")
        self.ser.write(msg.data.encode())

    def process_data(self):
        if self.ser.in_waiting:
            imu = Imu()
            # imu.header. = self.cnt
            imu.header.stamp = self.get_clock().now().to_msg()
            sensor = self.ser.read(self.ser.in_waiting).decode()
            lines = sensor.strip().split('\n')

            for line in lines:
                tempresult = line.split(',')
                if len(tempresult) == 15:
                    id = int(tempresult[0][4])
                    quaternion_w = float(tempresult[4])
                    quaternion_x = float(tempresult[3])
                    quaternion_y = float(tempresult[2])
                    quaternion_z = float(tempresult[1])
                    angular_velocity_x = float(tempresult[5])
                    angular_velocity_y = float(tempresult[6])
                    angular_velocity_z = float(tempresult[7])
                    linear_acceleration_x = float(tempresult[8])
                    linear_acceleration_y = float(tempresult[9])
                    linear_acceleration_z = float(tempresult[10])

                    q = Quaternion()
                    q.x = quaternion_x
                    q.y = quaternion_y
                    q.z = quaternion_z
                    q.w = quaternion_w
                    q = self.normalize_quaternion(q)

                    imu.orientation = q
                    imu.linear_acceleration = Vector3(x=linear_acceleration_x, y=linear_acceleration_y, z=linear_acceleration_z)
                    imu.angular_velocity = Vector3(x=angular_velocity_x * (PI / 180.0), y=angular_velocity_y * (PI / 180.0), z=angular_velocity_z * (PI / 180.0))

                    if id == 0:
                      self.imu_pub0.publish(imu)

    def normalize_quaternion(self, q):
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        q.x /= norm
        q.y /= norm
        q.z /= norm
        q.w /= norm
        return q

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
