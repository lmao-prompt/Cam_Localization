#!/usr/bin/env python3
"""
update_robot_pose.py (fixed)

Update posisi nexus_robot di Gazebo Sim berdasarkan /robot_pose dari ArUco.

FIX: subprocess call dipindah ke worker thread terpisah dengan
"drop-stale" pattern -- kalau masih ada call yang lagi jalan, data
lama di-skip dan cuma data TERBARU yang diproses. Ini mencegah
backlog menumpuk di callback subscription (yang bikin update
kelihatan lag/patah-patah).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import subprocess
import threading


class UpdateRobotPose(Node):
    def __init__(self):
        super().__init__('update_robot_pose')
        self.sub = self.create_subscription(
            PoseStamped, '/robot_pose', self.on_robot_pose, 10)
        self.world_name = 'car_world'

        # --- shared state buat worker thread ---
        self._lock = threading.Lock()
        self._latest_pose = None      # (x, y, z, qx, qy, qz, qw) paling baru
        self._pending = False
        self._busy = False
        self._stop = False

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info('Update Robot Pose - READY (threaded, non-blocking)')

    def on_robot_pose(self, msg: PoseStamped):
        # Callback cuma nyimpen data, gak nunggu apa-apa -> instan return
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)  # offset z robot
        qx = float(msg.pose.orientation.x)
        qy = float(msg.pose.orientation.y)
        qz = float(msg.pose.orientation.z)
        qw = float(msg.pose.orientation.w)

        with self._lock:
            self._latest_pose = (x, y, z, qx, qy, qz, qw)
            self._pending = True

    def _worker_loop(self):
        while not self._stop:
            pose = None
            with self._lock:
                if self._pending and not self._busy:
                    pose = self._latest_pose
                    self._pending = False
                    self._busy = True

            if pose is not None:
                self.set_pose('nexus_robot', *pose)
                with self._lock:
                    self._busy = False
            else:
                threading.Event().wait(0.005)

    def set_pose(self, name, x, y, z, qx, qy, qz, qw):
        req = f'name: "{name}" position {{ x: {x} y: {y} z: {z} }} orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }}'
        cmd = [
            "gz", "service", "-s", f"/world/{self.world_name}/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "30",
            "--req", req
        ]
        try:
            result = subprocess.run(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                text=True, timeout=1.0)
            if result.returncode != 0:
                self.get_logger().warn(f'Failed: {result.stderr}')
            return result.returncode == 0
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            return False

    def destroy_node(self):
        self._stop = True
        self._worker.join(timeout=1.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = UpdateRobotPose()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()