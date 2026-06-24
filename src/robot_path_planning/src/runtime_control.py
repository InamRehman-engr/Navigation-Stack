#!/usr/bin/env python

"""

import rospy
import json
import threading
from geometry_msgs.msg import Twist
from websocket_server import WebsocketServer

# Store connected clients (full client objects)
connected_clients = []
rosip = '192.168.18.32'  # Use 0.0.0.0 to allow external devices like Flutter apps to connect
# Latest cmd_vel
latest_cmd_vel = {}

def cmd_vel_callback(msg):
    global latest_cmd_vel
    latest_cmd_vel = {
        'linear': {
            'x': round(msg.linear.x, 2),
            'y': round(msg.linear.y, 2),
            'z': round(msg.linear.z, 2)
        },
        'angular': {
            'x': round(msg.angular.x, 2),
            'y': round(msg.angular.y, 2),
            'z': round(msg.angular.z, 2)
        }
    }

def send_cmd_vel_to_clients(server):
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        if latest_cmd_vel:
            message = json.dumps(latest_cmd_vel)
            for client in connected_clients:
                try:
                    server.send_message(client, message)
                except Exception as e:
                    rospy.logwarn("Failed to send message to client: %s", e)
        rate.sleep()

def on_message(client, server, message):
    rospy.loginfo("Received message from client: %s", message)
    # You can add handling if Flutter app sends any control messages

def on_error(client, server, error):
    rospy.logerr("WebSocket Error: %s", error)

def on_close(client, server):
    rospy.loginfo("WebSocket closed: %s", client)
    if client in connected_clients:
        connected_clients.remove(client)

def on_open(client, server):
    rospy.loginfo("WebSocket connection established: %s", client)
    connected_clients.append(client)

def websocket_server():
    server = WebsocketServer(host=rosip, port=9090)
    server.set_fn_new_client(on_open)
    server.set_fn_client_left(on_close)
    server.set_fn_message_received(on_message)

    thread = threading.Thread(target=send_cmd_vel_to_clients, args=(server,))
    thread.daemon = True
    thread.start()

    server.run_forever()

def main():
    rospy.init_node('cmd_vel_websocket_server')
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    rospy.loginfo("Subscribed to /cmd_vel")

    websocket_thread = threading.Thread(target=websocket_server)
    websocket_thread.daemon = True
    websocket_thread.start()

    rospy.loginfo("WebSocket server started at ws://{}:9090".format(rosip))

    rospy.spin()

if __name__ == "__main__":
    main()

"""









#!/usr/bin/env python

import rospy
import json
import threading
from geometry_msgs.msg import Twist
from websocket_server import WebsocketServer
import time


connected_clients = []
rosip = '172.17.69.104'
latest_cmd_vel = {}

# Velocity scale factors (tweak as needed)
LINEAR_SPEED = 0.8
ANGULAR_SPEED = 0.8

# Publisher
cmd_vel_pub = None

def cmd_vel_callback(msg):
    global latest_cmd_vel
    latest_cmd_vel = {
        'linear': {
            'x': round(msg.linear.x, 2),
            'y': round(msg.linear.y, 2),
            'z': round(msg.linear.z, 2)
        },
        'angular': {
            'x': round(msg.angular.x, 2),
            'y': round(msg.angular.y, 2),
            'z': round(msg.angular.z, 2)
        }
    }

def send_cmd_vel_to_clients(server):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if latest_cmd_vel:
            message = json.dumps(latest_cmd_vel)
            for client in connected_clients:
                try:
                    server.send_message(client, message)
                except Exception as e:
                    rospy.logwarn("Failed to send message to client: %s", e)
        rate.sleep()

def on_message(client, server, message):
    global last_key_time, current_key
    rospy.loginfo("Received message: %s", message)
    try:
        data = json.loads(message)
        topic = data.get("topic")
        msg = data.get("message")

        if topic == "/key_command" and 'key' in msg:
            key = msg['key']
            twist = Twist()

            if key == 'W' or current_key == 'W':
                twist.linear.x = LINEAR_SPEED
                cmd_vel_pub.publish(twist)
                rospy.sleep(0.5)
                current_key = key
            elif key == 'S' or current_key == 'S':
                twist.linear.x = -LINEAR_SPEED
                cmd_vel_pub.publish(twist)
                rospy.sleep(0.5)
                current_key = key
            elif key == 'A' or current_key == 'A':
                twist.angular.z = ANGULAR_SPEED
                cmd_vel_pub.publish(twist)
                rospy.sleep(0.5)
                current_key = key
            elif key == 'D' or current_key == 'D':
                twist.angular.z = -ANGULAR_SPEED
                cmd_vel_pub.publish(twist)
                rospy.sleep(0.5)
                current_key = key
            elif key == 'X' or current_key == 'X':  # stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                rospy.sleep(0.5)
                current_key = key

            cmd_vel_pub.publish(twist)
            last_key_time = time.time()
            rospy.loginfo("Published key command: %s", key)

    except Exception as e:
        rospy.logwarn("Failed to parse incoming message: %s", e)

def on_error(client, server, error):
    rospy.logerr("WebSocket Error: %s", error)

def on_close(client, server):
    rospy.loginfo("WebSocket closed: %s", client)
    if client in connected_clients:
        connected_clients.remove(client)

def on_open(client, server):
    rospy.loginfo("WebSocket connection established: %s", client)
    connected_clients.append(client)

def websocket_server():
    server = WebsocketServer(host=rosip, port=9090)
    server.set_fn_new_client(on_open)
    server.set_fn_client_left(on_close)
    server.set_fn_message_received(on_message)

    thread = threading.Thread(target=send_cmd_vel_to_clients, args=(server,))
    thread.daemon = True
    thread.start()

    server.run_forever()

def main():
    global cmd_vel_pub
    rospy.init_node('cmd_vel_websocket_bridge')
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.loginfo("Subscribed to /cmd_vel and ready to publish commands")

    websocket_thread = threading.Thread(target=websocket_server)
    websocket_thread.daemon = True
    websocket_thread.start()

    rospy.loginfo("WebSocket server started at ws://{}:9090".format(rosip))
    rospy.spin()

if __name__ == "__main__":
    main()
