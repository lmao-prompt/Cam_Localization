#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import firebase_admin
from firebase_admin import credentials, db

SERVICE_ACCOUNT_PATH = "klod-a02d7-firebase-adminsdk-fbsvc-2af2f722df.json"
DATABASE_URL = "https://klod-a02d7-default-rtdb.asia-southeast1.firebasedatabase.app/"
ODOM_TOPIC = "/robot_pose"
RTDB_PATH = "robot_status/position"   
PUSH_INTERVAL_SEC = 0.5               


def quaternion_to_yaw(q):
    """Convert quaternion (geometry_msgs/Quaternion) ke yaw (radian)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
 
 
class OdomToRTDB(Node):
    def __init__(self):
        super().__init__("odom_to_rtdb")
 
        cred = credentials.Certificate(SERVICE_ACCOUNT_PATH)
        firebase_admin.initialize_app(cred, {"databaseURL": DATABASE_URL})
        self.ref = db.reference(RTDB_PATH)
        self.get_logger().info(f"Firebase RTDB connected -> {RTDB_PATH}")
 
        self.last_push_time = 0.0
        self.latest = None  # (x, y, theta)
 
        self.sub = self.create_subscription(
            PoseStamped, ODOM_TOPIC, self.pose_callback, 10
        )
 
        self.timer = self.create_timer(PUSH_INTERVAL_SEC, self.push_to_rtdb)
 
        self.get_logger().info(
            f"Subscribing {ODOM_TOPIC}, push tiap {PUSH_INTERVAL_SEC}s ke RTDB"
        )
 
    def pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = quaternion_to_yaw(msg.pose.orientation)
        self.latest = (x, y, theta)
 
    def push_to_rtdb(self):
        if self.latest is None:
            return  # belum ada data odom masuk
 
        x, y, theta = self.latest
        try:
            self.ref.set(
                {
                    "x": round(x, 4),
                    "y": round(y, 4),
                    "theta": round(theta, 4),
                    "theta_deg": round(math.degrees(theta), 2),
                    "timestamp": time.time(),
                }
            )
            self.get_logger().info(
                f"Pushed -> x={x:.2f} y={y:.2f} theta={math.degrees(theta):.1f}°"
            )
        except Exception as e:
            self.get_logger().error(f"Gagal push ke RTDB: {e}")
 
 
def main():
    rclpy.init()
    node = OdomToRTDB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
 
