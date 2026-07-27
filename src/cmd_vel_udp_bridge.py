#!/usr/bin/env python3
"""
Node ROS2: subscribe /cmd_vel (geometry_msgs/Twist)
lalu kirim ke ESP32 (SoftAP mode) via UDP dalam format JSON.

Cara jalanin:
  1. Connect PC ke WiFi SSID "ESP32_Robot" (password "robot1234")
  2. Pastikan PC dapat IP di range 192.168.4.x
  3. ros2 run <package> cmd_vel_udp_bridge
     atau langsung: python3 cmd_vel_udp_bridge.py

Requirement:
  pip install --break-system-packages (tidak perlu, cukup pakai socket bawaan python)
  ROS2 sudah ter-install & di-source
"""

import json
import socket

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ==== Konfigurasi jaringan ESP32 SoftAP ====
ESP32_IP = "192.168.4.1"   # IP default ESP32 saat SoftAP
ESP32_PORT = 4210          # harus sama dengan UDP_PORT di sketch ESP32


class CmdVelUdpBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_udp_bridge')

        # Buat UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Subscribe ke /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info(
            f"Bridge aktif. Meneruskan /cmd_vel -> UDP {ESP32_IP}:{ESP32_PORT}"
        )

    def cmd_vel_callback(self, msg: Twist):
        payload = {
            "linear_x": round(msg.linear.x, 4),
            "angular_z": round(msg.angular.z, 4),
        }
        data = json.dumps(payload).encode('utf-8')

        try:
            self.sock.sendto(data, (ESP32_IP, ESP32_PORT))
            self.get_logger().debug(f"Sent: {payload}")
        except Exception as e:
            self.get_logger().warn(f"Gagal kirim UDP: {e}")

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelUdpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
