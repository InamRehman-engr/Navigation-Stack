#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Python 2.7

"""
This script sends LIDAR scan data over a WebSocket server.
Clients can connect to receive real-time LIDAR data.
"""

import rospy
import math
import struct
import base64
from sensor_msgs.msg import LaserScan
from websocket_server import WebsocketServer

clients = []

def new_client(client, server):
    rospy.loginfo("Client connected: %d" % client['id'])
    clients.append(client)

def client_left(client, server):
    rospy.loginfo("Client disconnected: %d" % client['id'])
    if client in clients:
        clients.remove(client)

def clean_ranges(ranges, max_range):
    """Replace NaN or Infinity with max_range"""
    cleaned = []
    for r in ranges:
        if r is None or math.isnan(r) or math.isinf(r):
            cleaned.append(max_range)
        else:
            cleaned.append(r)
    return cleaned

def scan_callback(msg):
    if not clients:
        return

    ranges_clean = clean_ranges(msg.ranges, msg.range_max)
    n = len(ranges_clean)

    # Pack data: number of points (uint32) + ranges (float32)
    packet = struct.pack("I", n) + struct.pack("%sf" % n, *ranges_clean)

    # Encode as base64 string for UTF-8 safe transfer
    packet_b64 = base64.b64encode(packet)

    # Send to all clients
    for c in clients:
        try:
            server.send_message(c, packet_b64)
        except Exception as e:
            rospy.logerr("WebSocket send error: %s" % str(e))

def main():
    rospy.init_node("scan_websocket_sender", anonymous=False)

    global server
    # Bind to all interfaces
    server = WebsocketServer(host='0.0.0.0', port=8765)
    server.set_fn_new_client(new_client)
    server.set_fn_client_left(client_left)

    rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=1)

    rospy.loginfo("WebSocket Server started at ws://<Jetson_IP>:8765")
    server.run_forever()

if __name__ == "__main__":
    main()
