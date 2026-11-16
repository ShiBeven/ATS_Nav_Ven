#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import Pose
from pb_rm_interfaces.msg import GameRobotHP, RfidStatus, RobotStatus
from rclpy.node import Node
from rmoss_interfaces.msg import RfidStatusArray, RobotStatus as SimRobotStatus


class GazeboRefereeAdapter(Node):
    def __init__(self):
        super().__init__("gazebo_referee_adapter")
        self.declare_parameter("robot_name", "red_standard_robot1")
        self.robot_name = str(self.get_parameter("robot_name").value)
        self.robot_statuses = {}
        self.supplier_area_active = False
        self.supplier_area_last_seen = 0.0

        self.robot_status_pub = self.create_publisher(
            RobotStatus, "referee/robot_status", 10
        )
        self.robot_hp_pub = self.create_publisher(
            GameRobotHP, "referee/all_robot_hp", 10
        )
        self.rfid_status_pub = self.create_publisher(
            RfidStatus, "referee/rfid_status", 10
        )

        self.create_subscription(
            SimRobotStatus,
            "/referee_system/red_standard_robot1/robot_status",
            lambda msg: self.robot_status_callback("red_standard_robot1", msg),
            10,
        )
        self.create_subscription(
            SimRobotStatus,
            "/referee_system/blue_standard_robot1/robot_status",
            lambda msg: self.robot_status_callback("blue_standard_robot1", msg),
            10,
        )
        self.create_subscription(
            RfidStatusArray,
            "/referee_system/rfid_info",
            self.rfid_status_callback,
            10,
        )
        self.create_timer(0.2, self.publish_rfid_status)

    def robot_status_callback(self, name: str, source: SimRobotStatus):
        self.robot_statuses[name] = source
        if name == self.robot_name:
            target = RobotStatus()
            target.robot_id = 107 if name.startswith("blue") else 7
            target.robot_level = max(1, source.level)
            target.current_hp = max(0, source.remain_hp)
            target.maximum_hp = max(0, source.max_hp)
            target.projectile_allowance_17mm = max(
                0, source.total_projectiles - source.used_projectiles
            )
            target.robot_pos = Pose()
            target.robot_pos.position.x = source.gt_tf.translation.x
            target.robot_pos.position.y = source.gt_tf.translation.y
            target.robot_pos.position.z = source.gt_tf.translation.z
            target.robot_pos.orientation = source.gt_tf.rotation
            q = target.robot_pos.orientation
            if q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w < 1e-9:
                q.w = 1.0
            self.robot_status_pub.publish(target)

        hp = GameRobotHP()
        red = self.robot_statuses.get("red_standard_robot1")
        blue = self.robot_statuses.get("blue_standard_robot1")
        if red:
            hp.red_7_robot_hp = max(0, red.remain_hp)
        if blue:
            hp.blue_7_robot_hp = max(0, blue.remain_hp)
        # The current Gazebo referee does not model outpost/base HP. Keep these
        # alive in the behavior input until the simulator exposes those values.
        hp.red_outpost_hp = 600
        hp.blue_outpost_hp = 600
        hp.red_base_hp = 1500
        hp.blue_base_hp = 1500
        self.robot_hp_pub.publish(hp)

    def rfid_status_callback(self, source: RfidStatusArray):
        self.supplier_area_active = False
        for status in source.robot_rfid_status:
            if status.robot_name != self.robot_name:
                continue
            if status.supplier_area_is_triggered:
                self.supplier_area_active = True
                self.supplier_area_last_seen = time.monotonic()
                break

    def publish_rfid_status(self):
        active = self.supplier_area_active and (
            time.monotonic() - self.supplier_area_last_seen < 2.0
        )
        target = RfidStatus()
        target.friendly_supply_zone_non_exchange = active
        self.rfid_status_pub.publish(target)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboRefereeAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
