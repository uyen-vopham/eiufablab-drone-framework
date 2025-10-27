#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from mavros_msgs.srv import SetMode
from std_msgs.msg import Bool
from nav_msgs.msg import Path 
from mavros_msgs.msg import WaypointList, Waypoint
import math 



class PubWpToPath(Node):
    def __init__(self):
        super().__init__('PubWpToPath')

        #Initial  
        self.pose = None
        self.lat = None
        self.lon = None
        self.alt = None 
        # self.last_percentage = None
        # self.last_voltage = None 
        # self.serial_lock = Lock()
        self.vel = None 

        # QoS 
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        
        qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=50
        )

        #Subcription and Publisher 
        self.publisher_ = self.create_publisher(WaypointList, "/offboard_control/waypoints", qos_pub)
        timer_period = 2.0
        self.count = 0
        self.publish_waypoint()
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def publish_waypoint(self):
        if (self.count > 1):
            return
        waypoints = []
        for i in range(50):
            wp = Waypoint()
            wp.frame = 3 #MAV_FRAME_GLOBAL_RELATIVE_ALT lat, long, alt
            wp.command = 16 #MAV_CMD_NAV_WAYPOINT
            wp.is_current = (i == 0)
            wp.autocontinue = True
            wp.x_lat = 11.052940 + 0.0001 * i
            wp.y_long = 106.666112 + 0.0001 * i
            wp.z_alt = 7.0
            waypoints.append(wp)
        msg = WaypointList()
        msg.current_seq = 0
        msg.waypoints = waypoints

        self.publisher_.publish(msg)
        self.get_logger().info(f'✅ Published {len(waypoints)} waypoints')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = PubWpToPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 