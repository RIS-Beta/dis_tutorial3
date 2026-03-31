#!/usr/bin/env python3

import json
import os

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


amcl_pose_qos = QoSProfile(
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class SaveRobotPose(Node):
    def __init__(self):
        super().__init__("save_robot_pose")

        self.declare_parameter("pose_topic", "amcl_pose")
        self.declare_parameter("output_file", "robot_poses.json")
        self.declare_parameter("wait_timeout_sec", 10.0)

        self.pose_topic = self.get_parameter("pose_topic").value
        output_file_param = self.get_parameter("output_file").value
        self.wait_timeout_sec = float(self.get_parameter("wait_timeout_sec").value)

        if os.path.isabs(output_file_param):
            self.output_file = output_file_param
        else:
            self.output_file = os.path.join(os.getcwd(), output_file_param)

        self.pose_saved = False
        self.timeout_timer = self.create_timer(self.wait_timeout_sec, self._on_timeout)

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.pose_callback,
            amcl_pose_qos,
        )

        self.get_logger().info(
            f"Waiting for pose on '{self.pose_topic}', output file: '{self.output_file}'"
        )

    def _load_existing_poses(self):
        if not os.path.exists(self.output_file):
            return []

        try:
            with open(self.output_file, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (json.JSONDecodeError, OSError) as exc:
            self.get_logger().warn(
                f"Could not parse existing JSON, creating new list. Reason: {exc}"
            )
            return []

        if isinstance(data, list):
            return data

        if isinstance(data, dict) and isinstance(data.get("poses"), list):
            return data["poses"]

        self.get_logger().warn("Existing JSON has unexpected format. Starting new pose list.")
        return []

    def _append_pose_to_file(self, pose_entry):
        poses = self._load_existing_poses()
        poses.append(pose_entry)

        parent_dir = os.path.dirname(self.output_file)
        if parent_dir:
            os.makedirs(parent_dir, exist_ok=True)

        with open(self.output_file, "w", encoding="utf-8") as f:
            json.dump(poses, f, indent=2)

    def _on_timeout(self):
        if self.pose_saved:
            return

        self.get_logger().error(
            f"No pose received on '{self.pose_topic}' after {self.wait_timeout_sec:.1f}s. Exiting."
        )
        rclpy.shutdown()

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        if self.pose_saved:
            return

        pose = msg.pose.pose
        stamp = msg.header.stamp
        pose_entry = {
            "frame_id": msg.header.frame_id if msg.header.frame_id else "map",
            "timestamp": {
                "sec": int(stamp.sec),
                "nanosec": int(stamp.nanosec),
            },
            "position": {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
            },
            "orientation": {
                "x": float(pose.orientation.x),
                "y": float(pose.orientation.y),
                "z": float(pose.orientation.z),
                "w": float(pose.orientation.w),
            },
        }

        try:
            self._append_pose_to_file(pose_entry)
        except OSError as exc:
            self.get_logger().error(f"Failed to write pose to JSON: {exc}")
            rclpy.shutdown()
            return

        self.pose_saved = True
        self.timeout_timer.cancel()
        self.get_logger().info("Robot pose saved and appended successfully.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SaveRobotPose()

    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
