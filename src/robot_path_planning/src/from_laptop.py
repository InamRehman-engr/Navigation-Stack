#!/usr/bin/env python
# -*- coding: utf-8 -*-
# # Python 2.7

# import rospy
# import math
# import base64
# import struct
# import threading
# from sensor_msgs.msg import LaserScan
# from websocket_server import WebsocketServer

# clients_lock = threading.Lock()
# clients = []

# def new_client(client, server):
#     rospy.loginfo("Client connected: %d" % client['id'])
#     with clients_lock:
#         clients.append(client)

# def client_left(client, server):
#     rospy.loginfo("Client disconnected: %d" % client['id'])
#     with clients_lock:
#         if client in clients:
#             clients.remove(client)

# def clean_ranges(ranges, max_range):
#     """Replace NaN or Infinity with max_range"""
#     cleaned = []
#     for r in ranges:
#         if r is None or math.isnan(r) or math.isinf(r):
#             cleaned.append(max_range)
#         else:
#             cleaned.append(r)
#     return cleaned


# def scan_callback(msg):
#     with clients_lock:
#         if not clients:
#             return

#         # Clean the ranges
#         ranges_clean = clean_ranges(msg.ranges, msg.range_max)
#         n = len(ranges_clean)

#         # Pack data: number of points (uint32) + ranges (float32)
#         packet = struct.pack("I", n) + struct.pack("%sf" % n, *ranges_clean)

#         # Encode the binary packet as Base64 string
#         packet_b64 = base64.b64encode(packet).decode('ascii')

#         # Send Base64 string to all clients
#         for c in clients[:]:  # copy to avoid modification during iteration
#             try:
#                 server.send_message(c, packet_b64)
#             except Exception as e:
#                 rospy.logerr("WebSocket send error to client %d: %s" % (c['id'], str(e)))
#                 clients.remove(c)

# def main():
#     rospy.init_node("scan_websocket_sender", anonymous=False)

#     global server
#     server = WebsocketServer(host='0.0.0.0', port=8765)  # Removed loglevel
#     server.set_fn_new_client(new_client)
#     server.set_fn_client_left(client_left)

#     # Run WebSocket server in a separate thread
#     server_thread = threading.Thread(target=server.run_forever)
#     server_thread.daemon = True
#     server_thread.start()

#     rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=1)

#     rospy.loginfo("WebSocket Server started at ws://<Jetson_IP>:8765")
#     rospy.spin()  # Keep ROS running

# if __name__ == "__main__":
#     main()













#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Python 2.7 (Requires websocket-server: pip install websocket-server)

import rospy
import math
import base64
import struct
import threading
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from websocket_server import WebsocketServer

# --- ROS Topics ---
SCAN_TOPIC = "/scan"
CMD_TOPIC = "/cmd_vel"
# --- WebSocket Port ---
WS_PORT = 8765

# --- Globals for WebSocket and Clients ---
clients_lock = threading.Lock()
clients = []
server = None  # Global server instance

# --- ROS Publisher ---
cmd_vel_pub = None

# ------------------ WebSocket Client Handlers ------------------

def new_client(client, server):
    rospy.loginfo("Client connected: %d" % client['id'])
    with clients_lock:
        clients.append(client)

def client_left(client, server):
    rospy.loginfo("Client disconnected: %d" % client['id'])
    with clients_lock:
        if client in clients:
            clients.remove(client)

# ------------------ CMD_VEL Receiver (New Functionality) ------------------

def message_received(client, server, message):
    """Handles incoming WebSocket messages (e.g., cmd_vel)."""
    try:
        data = base64.b64decode(message)

        if not data:
            return

        # Get the first byte as a slice (important for robust Python 2 struct/byte handling)
        msg_type = data[0:1]

        # ---------- CMD_VEL ----------
        # Use b'C' for byte string comparison and check length is 9 (1 byte tag + 8 bytes floats)
        if msg_type == b'C':
            if len(data) != 9:
                 rospy.logwarn("Received 'C' message of incorrect length: %d. Expected 9." % len(data))
                 return
                 
            # Unpack the linear (float) and angular (float) velocity
            # data[1:] contains the 8 bytes of two floats
            linear_x, angular_z = struct.unpack("ff", data[1:])

            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z

            if cmd_vel_pub:
                cmd_vel_pub.publish(twist)
                rospy.logdebug("Published cmd_vel: Lin: %.2f, Ang: %.2f" % (linear_x, angular_z))
            else:
                rospy.logwarn("cmd_vel publisher not initialized.")

        elif msg_type == b'L':
            # Ignore incoming LIDAR data if the client accidentally sends it back
            rospy.logdebug("Ignoring incoming LIDAR data.")
            pass
            
        else:
            rospy.logwarn("Unknown message type received from client %d: %s" % (client['id'], msg_type))

    except Exception as e:
        rospy.logerr("WS parse error (cmd_vel receiver): %s", str(e))


# ------------------ LIDAR Sender ------------------

def clean_ranges(ranges, max_range):
    """Replace NaN or Infinity with max_range"""
    cleaned = []
    for r in ranges:
        # Check for None, NaN, or Infinity
        if r is None or math.isnan(r) or math.isinf(r):
            cleaned.append(max_range)
        else:
            cleaned.append(r)
    return cleaned

def scan_callback(msg):
    """ROS callback to send LaserScan data over WebSocket."""
    global server

    with clients_lock:
        if not clients or server is None:
            return

        # Clean the ranges
        ranges_clean = clean_ranges(msg.ranges, msg.range_max)
        n = len(ranges_clean)

        # Prepend message tag 'L' (LIDAR)
        # Pack data: number of points (uint32 'I') + ranges (float32 'f')
        try:
            packet_data = struct.pack("I", n) + struct.pack("%sf" % n, *ranges_clean)
            packet = 'L' + packet_data

            # Encode the binary packet as Base64 string
            packet_b64 = base64.b64encode(packet)

            # Send Base64 string to all clients
            for c in clients[:]:  # copy to avoid modification during iteration
                server.send_message(c, packet_b64)
        except Exception as e:
            rospy.logerr("Error packing or sending LIDAR data: %s" % str(e))


# ------------------ Main ------------------

def main():
    rospy.init_node("ws_bridge_server", anonymous=False)

    global server
    global cmd_vel_pub

    # 1. Initialize CMD_VEL Publisher
    cmd_vel_pub = rospy.Publisher(CMD_TOPIC, Twist, queue_size=1)
    
    # 2. Setup WebSocket Server
    server = WebsocketServer(host='0.0.0.0', port=WS_PORT)
    server.set_fn_new_client(new_client)
    server.set_fn_client_left(client_left)
    server.set_fn_message_received(message_received)

    # Run WebSocket server in a separate thread
    rospy.loginfo("Starting WebSocket Server...")
    server_thread = threading.Thread(target=server.run_forever)
    server_thread.daemon = True
    server_thread.start()

    # 3. Subscribe to LIDAR for Sending
    rospy.Subscriber(SCAN_TOPIC, LaserScan, scan_callback, queue_size=1)

    rospy.loginfo("WebSocket Server running at ws://0.0.0.0:%d. Waiting for ROS and WS data." % WS_PORT)
    rospy.spin()  # Keep ROS running

if __name__ == "__main__":
    main()