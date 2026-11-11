import csv
import math
import os
import re
import shutil
import threading
from pathlib import Path

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GazeboNode


def next_telemetry_path(directory):
    pattern = re.compile(r"^telemetria_(\d{3})\.csv$")
    numbers = []
    for path in directory.glob("telemetria_*.csv"):
        match = pattern.match(path.name)
        if match:
            numbers.append(int(match.group(1)))
    number = max(numbers, default=0) + 1
    return directory / f"telemetria_{number:03d}.csv"


class ColetorValidacao(Node):
    """Registra poses e forças da simulação em um arquivo CSV."""

    POSE_TOPIC = "/world/default/pose/info"
    TENSION_ANCHOR_TOPIC = "/cabo/tensao_ancora"
    TENSION_TIP_TOPIC = "/cabo/tensao_ponta"
    ROVER_CONNECTION_OFFSET = (0.0, 0.0, 0.24)

    def __init__(self):
        super().__init__("coletor_validacao")

        self.lock = threading.Lock()
        self.poses = {}
        self.orientations = {}
        self.tension_anchor = None
        self.tension_tip = None
        self.last_stamp = None

        configured_output = os.environ.get("DRONE_VALIDATION_OUTPUT")
        if configured_output:
            self.output_path = Path(configured_output).expanduser()
            self.latest_path = None
        else:
            directory = Path("results/validation")
            directory.mkdir(parents=True, exist_ok=True)
            self.output_path = next_telemetry_path(directory)
            self.latest_path = directory / "telemetria.csv"

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.output_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.csv_header())
        self.file.flush()

        self.gz_node = GazeboNode()
        if not self.gz_node.subscribe(
            Pose_V,
            self.POSE_TOPIC,
            self.pose_callback,
        ):
            raise RuntimeError(
                f"Não foi possível assinar {self.POSE_TOPIC}."
            )

        self.anchor_subscription = self.create_subscription(
            WrenchStamped,
            self.TENSION_ANCHOR_TOPIC,
            self.anchor_callback,
            10,
        )
        self.tip_subscription = self.create_subscription(
            WrenchStamped,
            self.TENSION_TIP_TOPIC,
            self.tip_callback,
            10,
        )
        self.timer = self.create_timer(0.02, self.write_sample)

        self.get_logger().info(
            f"Gravando dados de validação em {self.output_path}"
        )

    @staticmethod
    def csv_header():
        columns = [
            "time_s",
            "tether_length_m",
            "alpha_d_rad",
            "beta_d_rad",
            "alpha_p_rad",
            "beta_p_rad",
        ]
        for prefix in ("drone_pose", "rover_pose"):
            columns.extend(
                f"{prefix}_{axis}" for axis in ("x", "y", "z")
            )
        for prefix in ("cable_root", "cable_tip", "rover_connection"):
            columns.extend(
                f"{prefix}_{axis}" for axis in ("x", "y", "z")
            )
        for prefix in ("anchor_force", "tip_force"):
            columns.extend(
                f"{prefix}_{axis}" for axis in ("x", "y", "z")
            )
            columns.append(f"{prefix}_norm")
        return columns

    @staticmethod
    def pose_values(pose):
        return (
            pose.position.x,
            pose.position.y,
            pose.position.z,
        )

    @staticmethod
    def orientation_values(pose):
        return (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

    @staticmethod
    def rotate_vector(quaternion, vector):
        x, y, z, w = quaternion
        vx, vy, vz = vector
        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)
        return (
            vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx,
        )

    @staticmethod
    def force_values(message):
        force = message.wrench.force
        norm = (force.x**2 + force.y**2 + force.z**2) ** 0.5
        return force.x, force.y, force.z, norm

    @staticmethod
    def tangent_angles(origin, target):
        dx = target[0] - origin[0]
        dy = target[1] - origin[1]
        dz = target[2] - origin[2]
        horizontal_distance = math.hypot(dx, dy)

        return (
            math.atan2(dy, dx),
            math.atan2(dz, horizontal_distance),
        )

    @staticmethod
    def message_time(message):
        stamp = message.header.stamp
        return stamp.sec + stamp.nanosec * 1e-9

    def pose_callback(self, message):
        with self.lock:
            for pose in message.pose:
                if pose.name == "base_link":
                    self.poses["rover_base_link"] = self.pose_values(pose)
                    self.orientations["rover_base_link"] = (
                        self.orientation_values(pose)
                    )
                else:
                    self.poses[pose.name] = self.pose_values(pose)

            rover = self.poses.get("rover_mecanum_1")
            rover_base = self.poses.get("rover_base_link")
            if rover is not None and rover_base is not None:
                base_orientation = self.orientations["rover_base_link"]
                offset = self.rotate_vector(
                    base_orientation,
                    self.ROVER_CONNECTION_OFFSET,
                )
                self.poses["rover_connection"] = tuple(
                    model_axis + link_axis + offset_axis
                    for model_axis, link_axis, offset_axis in zip(
                        rover,
                        rover_base,
                        offset,
                    )
                )

    def anchor_callback(self, message):
        with self.lock:
            self.tension_anchor = self.force_values(message)
            self.last_stamp = self.message_time(message)

    def tip_callback(self, message):
        with self.lock:
            self.tension_tip = self.force_values(message)
            self.last_stamp = self.message_time(message)

    def find_pose(self, suffix):
        if suffix == "rover_mecanum_1::base_link":
            return self.poses.get("rover_connection")

        for name, values in self.poses.items():
            if name == suffix or name.endswith(f"::{suffix}"):
                return values
        return None

    def cable_points(self):
        points = [self.poses.get("raiz_cabo")]
        segment_number = 1

        while True:
            point = self.poses.get(f"segment_{segment_number}")
            if point is None:
                break
            points.append(point)
            segment_number += 1

        points.append(self.poses.get("ponta_cabo"))
        if any(point is None for point in points):
            return None
        return points

    def write_sample(self):
        with self.lock:
            root = self.find_pose("raiz_cabo")
            tip = self.find_pose("ponta_cabo")
            drone = self.find_pose("x500_cabo")
            rover_model = self.find_pose("rover_mecanum_1")
            rover = self.find_pose("rover_mecanum_1::base_link")
            anchor = self.tension_anchor
            tension_tip = self.tension_tip
            stamp = self.last_stamp
            points = self.cable_points()

        if None in (
            root,
            tip,
            drone,
            rover_model,
            rover,
            anchor,
            tension_tip,
            stamp,
            points,
        ):
            return

        tether_length = sum(
            math.dist(first, second)
            for first, second in zip(points, points[1:])
        )
        alpha_d, beta_d = self.tangent_angles(points[0], points[1])
        alpha_p, beta_p = self.tangent_angles(points[-1], points[-2])

        self.writer.writerow(
            [
                stamp,
                tether_length,
                alpha_d,
                beta_d,
                alpha_p,
                beta_p,
                *drone,
                *rover_model,
                *root,
                *tip,
                *rover,
                *anchor,
                *tension_tip,
            ]
        )
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        if self.latest_path is not None:
            shutil.copyfile(self.output_path, self.latest_path)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ColetorValidacao()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
