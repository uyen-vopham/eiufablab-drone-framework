#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import time
import csv
import os

class PathPublish (Node):
    def __init__(self):
        super().__init__("PathPublishNode")
        self.get_logger().info("Starting path publish node...")

        self.publish_path = self.create_publisher()

(PoseStamped, '/mavros/setpoint_position/local', self.qos_profile_pub)