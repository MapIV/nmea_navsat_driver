import socket
import sys

import rclpy
from nmea_msgs.msg import Sentence

from libnmea_navsat_driver.driver import Ros2NMEADriver


def main(args=None):
    rclpy.init(args=args)

    driver = Ros2NMEADriver()

    nmea_pub = driver.create_publisher(Sentence, "nmea_sentence", 10)

    try:
        gnss_ip = driver.declare_parameter('ip', '192.168.131.22').value
        gnss_port = driver.declare_parameter('port', 9001).value
        buffer_size = driver.declare_parameter('buffer_size', 4096).value
    except KeyError as e:
        driver.get_logger().error("Parameter %s not found" % e)
        sys.exit(1)

    frame_id = driver.get_frame_id()

    driver.get_logger().info(
        "Using gnss sensor with ip {} and port {}".format(gnss_ip, gnss_port)
    )

    while rclpy.ok():
        try:
            gnss_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            gnss_socket.connect((gnss_ip, gnss_port))
        except socket.error as exc:
            driver.get_logger().error(
                "Caught exception socket.error when setting up socket: %s" % exc
            )
            sys.exit(1)

        partial = ""
        while rclpy.ok():
            try:
                recv_data = gnss_socket.recv(buffer_size)
                if not recv_data:
                    driver.get_logger().warn("Socket closed by remote host")
                    break

                partial += recv_data.decode("ascii")

                lines = partial.splitlines()
                if partial.endswith('\n'):
                    full_lines = lines
                    partial = ""
                else:
                    full_lines = lines[:-1]
                    partial = lines[-1] if lines else ""

                for data in full_lines:
                    data = data.strip()
                    if not data:
                        continue

                    sentence = Sentence()
                    sentence.header.stamp = driver.get_clock().now().to_msg()
                    sentence.header.frame_id = frame_id
                    sentence.sentence = data
                    nmea_pub.publish(sentence)

            except socket.error as exc:
                driver.get_logger().error(
                    "Caught exception socket.error when receiving: %s" % exc
                )
                gnss_socket.close()
                break

        gnss_socket.close()


if __name__ == "__main__":
    main()
