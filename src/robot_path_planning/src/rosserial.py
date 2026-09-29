#!/usr/bin/env python

"""
This script provides a bridge between ROS and an Arduino using a custom serial protocol.
It allows sending and receiving messages between ROS topics and the Arduino.
"""


import serial
import rospy
from std_msgs.msg import String

HEADER_BYTE = 0xFF
PROTOCOL_VER = 0xFE

class RosSerialBridge:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        rospy.init_node('rosserial_bridge', anonymous=True)
        self.subscribers = {}
        self.loop()

    def compute_checksum(self, data):
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def send_to_arduino(self, msg_type, topic, message):
        topic_bytes = topic.encode('utf-8')
        message_bytes = bytearray(message)
        topic_len = len(topic_bytes)
        msg_len = len(message_bytes)
        total_len = topic_len + msg_len + 2
        
        frame = bytearray([HEADER_BYTE, PROTOCOL_VER, total_len, topic_len])
        frame.extend(topic_bytes)
        frame.append(ord(msg_type))
        frame.extend(message_bytes)
        
        checksum = self.compute_checksum(frame[2:])
        frame.append(checksum)
        
        self.ser.write(frame)
        rospy.loginfo("Sent to Arduino: {}".format(' '.join(format(x, '02X') for x in frame)))

    def callback(self, data, topic):
        message_bytes = bytearray(data.data, 'utf-8')
        self.send_to_arduino('S', topic, message_bytes)

    def read_byte(self):
        byte = self.ser.read()
        return ord(byte) if byte else None

    def loop(self):
        while not rospy.is_shutdown():
            if self.ser.in_waiting > 0:
                header1 = self.read_byte()
                header2 = self.read_byte()
                
                if header1 is None or header2 is None:
                    continue  # Skip iteration if we didn't get enough data

                if header1 == HEADER_BYTE and header2 == PROTOCOL_VER:
                    total_len = self.read_byte()
                    topic_len = self.read_byte()
                    
                    if total_len is None or topic_len is None or topic_len >= 64:
                        continue  # Ignore invalid data

                    topic = self.ser.read(topic_len)
                    if len(topic) != topic_len:
                        continue  # Ignore if topic read fails

                    msg_type = self.ser.read(1)
                    if not msg_type:
                        continue  # Ignore if message type is missing

                    message_len = total_len - topic_len - 2
                    if message_len < 0 or message_len >= 64:
                        continue  # Ignore invalid message length

                    message = self.ser.read(message_len)
                    if len(message) != message_len:
                        continue  # Ignore if message read fails

                    received_checksum = self.read_byte()
                    if received_checksum is None:
                        continue  # Ignore if checksum is missing

                    computed_checksum = self.compute_checksum([total_len, topic_len, ord(msg_type)] + list(bytearray(topic)) + list(bytearray(message)))
                    
                    rospy.loginfo("Received: {}".format(' '.join(format(x, '02X') for x in ([header1, header2, total_len, topic_len] + list(bytearray(topic)) + [ord(msg_type)] + list(bytearray(message)) + [received_checksum]))))

                    if received_checksum == computed_checksum:
                        topic_str = topic.decode('utf-8')
                        message_str = message.decode('utf-8')

                        if msg_type == 'P':
                            pub = rospy.Publisher(topic_str, String, queue_size=10)
                            pub.publish(String(message_str))
                        elif msg_type == 'S':
                            if topic_str not in self.subscribers:
                                self.subscribers[topic_str] = rospy.Subscriber(topic_str, String, self.callback, callback_args=topic_str)

                    else:
                        rospy.logwarn("Checksum mismatch! Data may be corrupted.")
        
if __name__ == '__main__':
    try:
        RosSerialBridge()
        
    except rospy.ROSInterruptException:
        pass
