#!/usr/bin/env python
# -*- coding: utf-8 -*-

# # Python 2.7

# import rospy
# import struct
# import base64
# import websocket
# from sensor_msgs.msg import LaserScan
# import time

# JETSON_IP = "192.168.18.110"
# WS_PORT = 8765
# TOPIC_NAME = "/scan"

# def on_message(ws, message):
#     try:
#         data = base64.b64decode(message)
#         n = struct.unpack("I", data[:4])[0]
#         ranges = struct.unpack("%sf" % n, data[4:4 + 4 * n])

#         scan_msg = LaserScan()
#         scan_msg.header.stamp = rospy.Time.now()
#         scan_msg.header.frame_id = "laser"
#         scan_msg.angle_min = 0.0
#         scan_msg.angle_max = 2 * 3.1415926
#         scan_msg.angle_increment = 2 * 3.1415926 / n
#         scan_msg.time_increment = 0.0
#         scan_msg.scan_time = 0.1
#         scan_msg.range_min = 0.0
#         scan_msg.range_max = 10.0
#         scan_msg.ranges = ranges

#         pub.publish(scan_msg)
#     except Exception as e:
#         rospy.logerr("Error parsing LIDAR data: %s" % str(e))

# def on_error(ws, error):
#     rospy.logerr("WebSocket error: %s" % str(error))

# def on_close(ws, close_status_code, close_msg):
#     rospy.logwarn("WebSocket connection closed")

# def on_open(ws):
#     rospy.loginfo("Connected to Jetson LIDAR WebSocket server")

# if __name__ == "__main__":
#     rospy.init_node("lidar_ws_receiver", anonymous=True)
#     pub = rospy.Publisher(TOPIC_NAME, LaserScan, queue_size=1)

#     ws_url = "ws://{}:{}".format(JETSON_IP, WS_PORT)

#     while not rospy.is_shutdown():
#         ws = websocket.WebSocketApp(
#             ws_url,
#             on_message=on_message,
#             on_error=on_error,
#             on_close=on_close
#         )
#         ws.on_open = on_open

#         try:
#             ws.run_forever(ping_interval=10, ping_timeout=5)
#         except KeyboardInterrupt:
#             rospy.loginfo("Shutting down LIDAR WebSocket receiver...")
#             ws.close()
#             break
#         except Exception as e:
#             rospy.logerr("Unexpected error: %s" % str(e))

#         if not rospy.is_shutdown():
#             rospy.logwarn("Reconnecting in 1s...")
#             time.sleep(1)











#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Python 2.7 (Requires websocket-client: pip install websocket-client)

import rospy
import struct
import base64
import websocket # Make sure you use 'websocket-client', not 'websocket-server'
import time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# JETSON_IP = "192.168.18.110"  # <-- **UPDATE THIS IP IF NEEDED**
JETSON_IP = "10.75.1.177"  # <-- **UPDATE THIS IP IF NEEDED**
WS_PORT = 8765

SCAN_TOPIC = "/scan"
CMD_TOPIC = "/cmd_vel"

ws_global = None   # global WS handle
scan_pub = None # global publisher handle

# ------------------ WebSocket Receive ------------------

def on_message(ws, message):
    global scan_pub
    try:
        data = base64.b64decode(message)

        # Get the first byte as a slice
        msg_type = data[0:1]

        # ---------- LIDAR ----------
        if msg_type == b'L':
            # Ensure enough data is available for length and at least one point
            if len(data) < 9: # 1 byte tag + 4 bytes length + 4 bytes float
                rospy.logwarn("Received incomplete LIDAR message.")
                return

            # Unpack number of points (I = uint32, 4 bytes)
            n = struct.unpack("I", data[1:5])[0]
            
            # Unpack ranges (n * float, 4 * n bytes)
            # Use Little-Endian '<' for consistency in unpacking the float array
            ranges = struct.unpack("<%sf" % n, data[5:5 + 4 * n])

            if scan_pub is None:
                rospy.logwarn("Scan publisher not ready.")
                return

            scan = LaserScan()
            scan.header.stamp = rospy.Time.now()
            scan.header.frame_id = "laser"
            
            # These values need to be correct for your RPLIDAR model (e.g., A1 is 360 degrees)
            scan.angle_min = 0.0
            scan.angle_max = 2 * 3.1415926
            scan.angle_increment = (2 * 3.1415926) / n
            scan.time_increment = 0.0 # Placeholder
            scan.scan_time = 0.1      # Placeholder
            scan.range_min = 0.05
            scan.range_max = 10.0
            scan.ranges = ranges

            scan_pub.publish(scan)

        else:
            rospy.logwarn("Unknown message type received: %s" % msg_type)

    except Exception as e:
        rospy.logerr("WS parse error: %s", str(e))


def on_open(ws):
    rospy.loginfo("WebSocket connected to Jetson")


def on_close(ws, *args):
    rospy.logwarn("WebSocket closed")


def on_error(ws, error):
    rospy.logerr("WebSocket error: %s", str(error))


# ------------------ CMD_VEL Sender ------------------

def cmd_vel_callback(msg):
    global ws_global

    if ws_global is None or not ws_global.sock or not ws_global.sock.connected:
        return

    try:
        # FIX: Use '<' (Little-Endian) prefix to guarantee 9 bytes (1 byte 'c' + 8 bytes 'ff')
        payload = struct.pack(
            "<cff",
            b'C',                 # message tag (1 byte)
            msg.linear.x,         # linear x (4 bytes)
            msg.angular.z         # angular z (4 bytes)
        )

        encoded = base64.b64encode(payload)
        ws_global.send(encoded)
        rospy.logdebug("Sent cmd_vel (9 bytes): Lin: %.2f, Ang: %.2f" % (msg.linear.x, msg.angular.z))


    except Exception as e:
        rospy.logerr("Failed to send cmd_vel: %s", str(e))


# ------------------ Main ------------------

if __name__ == "__main__":

    rospy.init_node("lidar_ws_receiver", anonymous=True)

    scan_pub = rospy.Publisher(SCAN_TOPIC, LaserScan, queue_size=1)
    rospy.Subscriber(CMD_TOPIC, Twist, cmd_vel_callback)

    # Required to prevent errors if the remote machine's /cmd_vel topic is constantly publishing
    # Set this to a high number if your controller is publishing very fast
    rospy.Rate(10) # 10 Hz loop rate for the subscriber processing

    ws_url = "ws://{}:{}".format(JETSON_IP, WS_PORT)

    while not rospy.is_shutdown():
        ws = websocket.WebSocketApp(
            ws_url,
            on_message=on_message,
            on_open=on_open,
            on_close=on_close,
            on_error=on_error
        )

        ws_global = ws

        try:
            rospy.loginfo("Attempting to connect to %s" % ws_url)
            # run_forever blocks until the connection closes.
            ws.run_forever(ping_interval=10, ping_timeout=5)
        except Exception as e:
            rospy.logerr("WS exception: %s", str(e))

        ws_global = None

        if not rospy.is_shutdown():
            rospy.logwarn("WebSocket closed. Reconnecting in 1 second...")
            time.sleep(1)