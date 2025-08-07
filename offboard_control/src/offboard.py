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

class OffboardControlNode(Node):
    def __init__(self):
        super().__init__("drone_control_node")
        self.get_logger().info("Starting drone control node...")
        self.state = State()

        # Flight states
        self.takeoff_flag = False
        self.land_flag = False
        self.flight_mission = False
        self.flight_start_time = None  # To track the flight time

        #Initial value for x, y, and z in local position
        self.local_x = 0
        self.local_y = 0
        self.local_z = 0

        # QoS Profile for communication
        self.qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10)
        local_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10)
        self.csv_path = os.path.expanduser("drone_position_log.csv")
        with open(self.csv_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time(s)","X", "Y", "Z"])
            self.start_time = self.get_clock().now()

        # Subscribe to drone state to monitor mode and armed status
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, self.qos_profile_pub)
        
        # Publisher for position setpoint to maintain Offboard mode
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', self.qos_profile_pub)

        # Clients for arming and mode setting
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming', qos_profile=self.qos_profile_pub)
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode', qos_profile=self.qos_profile_pub)

        # Sub local, global location
        self.current_pose = PoseStamped()
        self.current_global = NavSatFix()
        self.global_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.global_cb, qos_profile=local_qos)
        self.local_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_cb, qos_profile=local_qos)

        # Timer to update drone position periodically
        self.log_timer = self.create_timer(0.1, self.log_position)

    def state_cb(self, msg):
        self.state = msg
        self.get_logger().info(f"Mode: {self.state.mode}, Armed: {self.state.armed}, Connected: {self.state.connected}")
        
        # Automatically arm and takeoff if Offboard mode is active
        if self.state.mode == "OFFBOARD" and not self.flight_mission:
            if not self.state.armed:
                self.arm_drone()  # Arm the drone if not already armed
                self.takeoff_flag = True # Trigger takeoff if armed

        # Trigger RTL mode and landing after 20 seconds
        if self.flight_start_time is not None:
            flight_duration = (self.get_clock().now() - self.flight_start_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
            if flight_duration >= 120.0:
                self.land_flag = True
                self.get_logger().info("20 seconds of flight passed. Switching to RTL mode and landing...")

    def arm_drone(self):
        self.get_logger().info("Arming drone...")
        arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        if not arm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for arming service...')
            return
        
        req = CommandBool.Request()
        req.value = True
        future = arm_client.call_async(req)
        future.add_done_callback(self.arm_callback)

    def arm_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Drone Armed!")
            else:
                self.get_logger().warn("Failed to Arm Drone!")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

    def takeoff(self):
        # self.get_logger().info("Taking off...")
        target_pose = PoseStamped()
        self.flight_start_time = self.get_clock().now()  # Track flight start time
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = 0.0
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 7.0  # Desired takeoff height
        self.local_pos_pub.publish(target_pose)

    def publish_target_position(self):
        if self.flight_mission:
            self.get_logger().info("Mission completed. Stopping setpoint publishing.")
            return
        
        # if self.land_flag: 
        #     self.set_mode_to_rtl_and_land()
        #     return 

        if self.takeoff_flag and not self.flight_mission:
            self.takeoff()
            

    def mode_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("Mode switched to AUTO.RTL successfully!")
                self.flight_mission = True
                self.land_flag = False
                self.takeoff_flag = False 

                req = SetMode.Request()
                req.custom_mode = "STABILIZED"  # Switch to a manual mode after landing
                future = self.set_mode_client.call_async(req)

                self.get_logger().info("Landing process completed, shutting down node.")
                rclpy.shutdown()    
            else:
                self.get_logger().warn("Failed to switch to AUTO.RTL mode!")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
    
    def local_cb(self,msg):
        self.local_x = msg.pose.position.x
        self.local_y = msg.pose.position.y
        self.local_z = msg.pose.position.z
        # print("z position: ", self.current_pose.pose.position.z)
        self.get_logger().info(f"Local pose: {self.local_x:.2f}, y: {self.local_y:.2f}, z: {self.local_z:.2f}")
        
    def global_cb(self, msg):
        self.current_global = msg

    def log_position(self):
        
        # print("local z", self.local_z)
        elapsed_time = (self.get_clock().now()- self.start_time).nanoseconds/1e9
        try:
            with open(self.csv_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([f"{elapsed_time:.2f}", f"{self.local_x:.2f}", f"{self.local_y:.2f}", f"{self.local_z:.2f}"])
        except Exception as e:
            self.get_logger().error(f"Failed to write to CSV: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
    ####