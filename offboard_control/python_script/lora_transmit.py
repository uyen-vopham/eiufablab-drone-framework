#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
import serial, json, threading, time, csv, os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from threading import Lock
from datetime import datetime
from mavros_msgs.srv import SetMode
from std_msgs.msg import Bool
from nav_msgs.msg import Path 
from sensor_msgs.msg import NavSatFix, BatteryState
from custom_msgs.srv import ModeSignal 
from mavros_msgs.msg import WaypointList, Waypoint
import math 



class LoraDrone(Node):
    def __init__(self):
        super().__init__('LoraDrone')

        #Initial  
        self.pose = None
        self.lat = None
        self.lon = None
        self.alt = None 
        self.last_percentage = None
        self.last_voltage = None 
        self.serial_lock = Lock()
        self.vel = None 

        # Serial
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info("✅ Serial đã kết nối tại /dev/ttyUSB0")
        except Exception as e:
            self.get_logger().error(f"❌ Không mở được serial: {e}")
            return

        # QoS 
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        
        qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        #Subcription and Publisher 
        self.subscription = self.create_subscription(PoseStamped, '/mavros/local_position/pose',self.pose_callback, qos_profile)
        self.subscription_global_position = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.global_position_callback, qos_profile)
        self.subscription_battery = self.create_subscription(BatteryState, '/mavros/battery', self.battery_cb, qos_profile)
        self.subscription_speed = self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self.velocity_cb, qos_profile)  
        self.mission_pub = self.create_publisher(WaypointList, '/offboard_waypoint_list', qos_pub) 
        
       
        self.mode_client = self.create_client(ModeSignal, '/offboard_service') 


        # Threads
        threading.Thread(target=self.read_serial, daemon=True).start()
        threading.Thread(target=self.send_pose_loop, daemon=True).start()
       
    #Helpers 
    def _send_json(self, obj: dict):
        try:
            with self.serial_lock:
                self.ser.write((json.dumps(obj, separators=(',', ':')) + "\n").encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"❌ Lỗi gửi JSON: {e}")
            

    # ROS2 callbacks / loops 
    def pose_callback(self, msg):
        self.pose = msg.pose
        # self.get_logger().info(f"Read {self.pose}")
    def global_position_callback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.alt = msg.altitude 
    def battery_cb(self, msg: BatteryState):
        # voltage (lọc NaN/inf)
        v = float(msg.voltage) if (isinstance(msg.voltage, (int, float)) and math.isfinite(msg.voltage)) else None

        # percentage: có thể là [0..1], [0..100], hoặc -1 (unknown)
        p = None
        if isinstance(msg.percentage, (int, float)) and math.isfinite(msg.percentage):
            p = float(msg.percentage)
            if 0.0 <= p <= 1.0:
                p *= 100.0
            elif p < 0:
                p = None  # unknown

        # nếu thiếu % mà có Volt -> ước lượng theo số cell bạn đang dùng (4S)
        if p is None and v is not None:
            p = self._estimate_percent_from_voltage(v, cells=4)
        if p is not None:
            p = max(0.0, min(100.0, p)) 

        self.last_percentage = p
        self.last_voltage = v
    
    def velocity_cb(self, msg: TwistStamped):
        try:
            linear = msg.twist.linear 
            self.vel = (float(linear.x), float(linear.y), float(linear.z))
        except Exception as e:
            self.get_logger().warn(f"velocity_cb error: {e}")
            self.vel = None 


    def _estimate_percent_from_voltage(self, voltage, cells=4, v_full=4.2, v_empty=3.3):
        if not (isinstance(voltage, (int, float)) and math.isfinite(voltage)) or cells <=0:
            return None
        v_cell = float(voltage) /float(cells) 
        if v_cell >=v_full: return 100.0
        if v_cell <=v_empty: return 0.0 
        return (v_cell-v_empty)/(v_full-v_empty)*100.0 
    
    def call_mode_service(self, mode_const, mode_name):
        if not self.mode_client.wait_for_service(timeout_sec=3.0):
            self._send_json({"event": "mode_push", "status": False, "mode": mode_name, "error": "service not ready"})
            self.get_logger().warn("Mode service not ready")
            return

        req = ModeSignal.Request()
        req.mode = mode_const
        future = self.mode_client.call_async(req)
        self.get_logger().info("SEND request to OFFBOARD SERVICE")

        def response_cb(fut):
            try:
                resp = fut.result()
                ok = bool(resp.accepted)
                self._send_json({"event": "mode_push", "status": ok, "mode": mode_name, "msg": resp.msg})
                self.get_logger().info(f"Mode {mode_name} -> accepted={ok}, msg={resp.msg}")
            except Exception as e:
                self._send_json({"event": "mode_push", "status": False, "mode": mode_name, "error": str(e)})
                self.get_logger().error(f"Mode service error: {e}")

        future.add_done_callback(response_cb)

    
    #Write/Read Data 
    def send_pose_loop(self):
        while rclpy.ok():
            try: 
                if self.pose and self.lat is not None and self.lon is not None and self.alt is not None:
                    
                    data = {
                        "x": round(self.pose.position.x, 3),
                        "y": round(self.pose.position.y, 3),
                        "z": round(self.pose.position.z, 3),
                        "lat": round(self.lat,6),
                        "lon": round(self.lon,6),
                        "alt": round(self.alt, 3)
                    }
                    if self.last_percentage is not None and self.last_voltage is not None:
                       
                        data["battery"] = {
                            "percent": round(self.last_percentage, 1),
                            "voltage": round(self.last_voltage, 2),
                        }
                    if self.vel is not None:
                        vx, vy, vz = self.vel 
                        spd = math.sqrt(vx*vx+vy*vy+vz*vz)
                        data["speed"]= round(spd,3) 
                        
                    data["hb"] = 1
                    with self.serial_lock:
                        json_str = json.dumps(data)
                        self.ser.write((json_str + "\n").encode())
                    self.get_logger().info(f"Sent: {json_str}")
            except Exception as e:
                self.get_logger().error(f"❌ Lỗi khi gửi dữ liệu serial: {e}")
            time.sleep(0.4)
    def read_serial(self):
        while True:
            try:
                with self.serial_lock:
                    line = self.ser.readline().decode('utf-8', errors='replace').strip()
                if not line:
                    continue
                self.handle_command(line)
            except Exception as e:
                self.get_logger().error(f"❌ Lỗi đọc serial: {e}")

    #Command handler
    def handle_command(self, line: str):
        """Nhận JSON từ Ground qua LoRa:
           - {"waypoints":[...]}  -> lưu vào path và publish thông qua topic /offboard_waypoint_list
        """
        try:
            cmd = json.loads(line)
        except json.JSONDecodeError:
            self.get_logger().warn(f"⚠️ Không thể decode JSON: {line}")
            return

        self.get_logger().info(f"📥 Lệnh nhận được: {cmd}")

        items = cmd.get("waypoints")
        if isinstance(items, list):
            saved = 0
            wp_msgs = WaypointList()
            # wp_msgs.header.stamp = self.get_clock().now().to_msg()
            # wp_msgs.header.frame_id = "GPS"
            wp_msgs.current_seq = 0
            try:
                    for wp in items:
                        self.get_logger().info(f"ITEM IS {wp}")
                        if all(k in wp for k in ("lat", "lon", "alt")):
                           
                            saved += 1
                            wp_msg = Waypoint()
                            wp_msg.frame = 3 #MAV_FRAME_GLOBAL 
                            wp_msg.command = 16 #MAV_CMD_NAV_WAYPOINT
                            wp_msg.is_current = (saved == 1)
                            wp_msg.autocontinue = True
                            wp_msg.x_lat = float(wp["lat"])
                            wp_msg.y_long = float(wp["lon"])
                            wp_msg.z_alt = float(wp["alt"])

                            wp_msgs.waypoints.append(wp_msg)
                            print("path", wp) 
                
                    self.mission_pub.publish(wp_msgs)
                    self.get_logger().info(f"📤 Đã publish {saved} waypoint lên /offboard_waypoint_list")
                    self._send_json({"event": "uploaded", "status": True, "count": saved})
            except Exception as e:
                self.get_logger().error(f"❌ Lỗi publish: {e}")
                self._send_json({"event": "uploaded", "status": False, "error": str(e)})
            return
        

       
        c = str(cmd.get("cmd","")).strip().lower()
        if c == "offboard":
            self.call_mode_service(ModeSignal.Request.OFFBOARD, "OFFBOARD")
            return
        if c == "land":
            self.call_mode_service(ModeSignal.Request.LAND, "LAND")
            return

        self.get_logger().info("ℹ️ Bỏ qua cmd không hợp lệ")

def main(args=None):
    rclpy.init(args=args)
    node = LoraDrone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()