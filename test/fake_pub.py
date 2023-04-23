#!/usr/bin/env python3

from functools import partial
import numpy as np
import rclpy
from rclpy.node import Node, Publisher
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


def pub(nh: Node, pub_d: Publisher, pub_s: Publisher):
    now = nh.get_clock().now().to_msg()
    msg_d = LaserScan(
        header=Header(stamp=now, frame_id="scandx"),
        angle_min=-1.57,
        angle_max=0.,
        angle_increment=0.03,
        time_increment=0.022,
        scan_time=1.,
        range_min=0.3,
        range_max=3.,
        ranges=np.random.uniform(low=0.3, high=3., size=45)
    )

    msg_s = LaserScan(
        header=Header(stamp=now, frame_id="scansx"),
        angle_min=0.,
        angle_max=1.57,
        angle_increment=0.03,
        time_increment=0.022,
        scan_time=1.,
        range_min=0.3,
        range_max=3.,
        ranges=np.random.uniform(low=0.3, high=3., size=45)
    )

    pub_d.publish(msg_d)
    pub_s.publish(msg_s)


def main():
    """
    Creates a node that publishes fake data to the default laser scan topics.
    It can be used to verify the Multi-merger and visualizer functions.
    """
    rclpy.init()
    print("Node is ready to publish")
    nh = rclpy.create_node("fake_publisher")
    quality = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
    pub_d = nh.create_publisher(LaserScan, "/scandx", quality)
    pub_s = nh.create_publisher(LaserScan, "/scansx", quality)
    while rclpy.ok():
        timer = nh.create_timer(0.5, partial(pub, nh=nh, pub_d=pub_d, pub_s=pub_s))  # NOQA:F841
        rclpy.spin(nh)


if __name__ == "__main__":
    main()
