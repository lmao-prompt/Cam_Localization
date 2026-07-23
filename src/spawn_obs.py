#!/usr/bin/env python3
"""
spawn_obs.py (FIXED for PoseStamped)

Update posisi yellow_obstacle_0 di Gazebo Sim via /world/car_world/set_pose.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped  # ⬅️ GANTI: PoseArray → PoseStamped
import subprocess
import threading


class UpdateYellowObstacle(Node):
    def __init__(self):
        super().__init__('update_yellow_obstacle')
        
        # ⬇️ GANTI: PoseArray → PoseStamped
        self.sub = self.create_subscription(
            PoseStamped, '/target_yellow', self.on_obstacles, 10)
        
        self.world_name = 'car_world'

        self._lock = threading.Lock()
        self._latest_pose = None
        self._pending = False
        self._busy = False
        self._stop = False

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info('Update Yellow Obstacle - READY (PoseStamped)')

    # ⬇️ GANTI: parameter type PoseStamped, gak perlu msg.poses[0]
    def on_obstacles(self, msg: PoseStamped):
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z) + 0.25

        with self._lock:
            self._latest_pose = (x, y, z)
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
                self.set_pose('yellow_obstacle_0', *pose)
                with self._lock:
                    self._busy = False
            else:
                threading.Event().wait(0.005)

    def set_pose(self, name, x, y, z):
        req = f'name: "{name}" position {{ x: {x} y: {y} z: {z} }} orientation {{ x: 0 y: 0 z: 0 w: 1 }}'
        cmd = [
            "gz", "service", "-s", f"/world/{self.world_name}/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "5000",
            "--req", req
        ]
        try:
            result = subprocess.run(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                text=True, timeout=5.0)
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
    node = UpdateYellowObstacle()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()