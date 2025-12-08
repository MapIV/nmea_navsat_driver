import socket
import sys
import struct
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from tf_transformations import quaternion_from_euler


class GsofTcpImuNode(Node):
    def __init__(self):
        super().__init__('gsof_tcp_driver')
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.rpy_deg_pub = self.create_publisher(Vector3, 'gsof_rpy_deg', 10)
        self.ip = self.declare_parameter('ip', '192.168.144.1').value
        self.port = self.declare_parameter('port', 5018).value
        self.buffer_size = self.declare_parameter('buffer_size', 4096).value
        self.frame_id = self.get_frame_id()
        self._buffer = b''

    def get_frame_id(self):
        frame_id = self.declare_parameter('frame_id', 'imu').value
        prefix = self.declare_parameter('tf_prefix', '').value
        if len(prefix):
            return f'{prefix}/{frame_id}'
        return frame_id

    def process_chunk(self, chunk: bytes):
        self._buffer += chunk
        i = 0
        while True:
            if i + 2 > len(self._buffer):
                break
            start = self._buffer.find(b"\x31", i)
            if start == -1:
                break
            if start + 2 > len(self._buffer):
                i = start
                break
            rec_len = self._buffer[start + 1]
            total_len = 2 + rec_len
            if start + total_len > len(self._buffer):
                i = start
                break
            record = self._buffer[start:start + total_len]
            self.handle_ins_full_nav(record)
            i = start + total_len
        if i > 0:
            self._buffer = self._buffer[i:]
        else:
            if len(self._buffer) > 4096:
                self._buffer = self._buffer[-4096:]

    def handle_ins_full_nav(self, record: bytes):
        if len(record) < 2:
            return
        rec_type = record[0]
        rec_len = record[1]
        if rec_type != 0x31:
            return
        if rec_len != 0x68:
            return
        data = record[2:]
        if len(data) < rec_len:
            return
        try:
            week, = struct.unpack_from('>H', data, 0)
            gps_ms, = struct.unpack_from('>I', data, 2)
            roll_deg, = struct.unpack_from('>d', data, 48)
            pitch_deg, = struct.unpack_from('>d', data, 56)
            heading_deg, = struct.unpack_from('>d', data, 64)
            rpy_msg = Vector3()
            rpy_msg.x = roll_deg
            rpy_msg.y = pitch_deg
            rpy_msg.z = heading_deg
            ang_rate_x_deg_s, = struct.unpack_from('>f', data, 80)
            ang_rate_y_deg_s, = struct.unpack_from('>f', data, 84)
            ang_rate_z_deg_s, = struct.unpack_from('>f', data, 88)
            acc_x, = struct.unpack_from('>f', data, 92)
            acc_y, = struct.unpack_from('>f', data, 96)
            acc_z, = struct.unpack_from('>f', data, 100)
        except struct.error:
            return
        now = self.get_clock().now().to_msg()
        self.rpy_deg_pub.publish(rpy_msg)
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self.frame_id
        q = quaternion_from_euler(
            math.radians(roll_deg),
            math.radians(pitch_deg),
            math.radians(heading_deg),
        )
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.angular_velocity.x = math.radians(ang_rate_x_deg_s)
        imu.angular_velocity.y = math.radians(ang_rate_y_deg_s)
        imu.angular_velocity.z = math.radians(ang_rate_z_deg_s)
        imu.linear_acceleration.x = acc_x
        imu.linear_acceleration.y = acc_y
        imu.linear_acceleration.z = acc_z
        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = GsofTcpImuNode()
    while rclpy.ok():
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((node.ip, node.port))
        except socket.error as exc:
            node.get_logger().error("Caught exception socket.error when setting up socket: %s" % exc)
            sys.exit(1)
        while rclpy.ok():
            try:
                chunk = sock.recv(node.buffer_size)
                if not chunk:
                    break
                node.process_chunk(chunk)
            except socket.error as exc:
                node.get_logger().error("Caught exception socket.error when receiving: %s" % exc)
                sock.close()
                break
        sock.close()


if __name__ == '__main__':
    main()
