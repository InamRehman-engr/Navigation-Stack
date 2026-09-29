#!/usr/bin/env python

"""
This script automates the detection of serial ports for the Arduino Nano and RPLIDAR devices.
It updates the corresponding ROS launch files with the detected port names.
"""


import serial.tools.list_ports
import os

# Define identifiers for the devices
ARDUINO_NANO_IDENTIFIER = "USB Serial"             # Adjust based on `description` or `hwid`
#RPLIDAR_IDENTIFIER = "CP2102 USB to UART Bridge Controller"  # Adjust based on `description` or `hwid`
RPLIDAR_IDENTIFIER = "PSR-TRISAFE"  # Adjust based on `description` or `hwid`


# Paths to the launch files
ARDUINO_LAUNCH_FILE = "/home/jetson/new_ws/src/robot_path_planning/launch/rosserial_control.launch"
RPLIDAR_LAUNCH_FILE = "/home/jetson/new_ws/src/rplidar_ros/launch/rplidar_a1.launch"

def find_device(identifier):
    """Find the serial port of a device based on its identifier."""
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if identifier in port.description or identifier in port.hwid:
            return port.device  # e.g., '/dev/ttyUSB0'
    return None

def update_launch_file(file_path, port_name):
    """Update the port name in the launch file."""
    if not os.path.exists(file_path):
        print("Launch file {} not found!".format(file_path))
        return False

    with open(file_path, "r") as file:
        lines = file.readlines()

    updated_lines = []
    for line in lines:
        if "port" in line and "value" in line:  # Match the port parameter line
            updated_line = '  <param name="port" value="{}" />\n'.format(port_name)
            updated_lines.append(updated_line)
        else:
            updated_lines.append(line)

    with open(file_path, "w") as file:
        file.writelines(updated_lines)

    print("Updated {} with port {}.".format(file_path, port_name))
    return True

def main():
    # Find the ports for Arduino Mega and RPLIDAR
    arduino_port = find_device(ARDUINO_NANO_IDENTIFIER)
    rplidar_port = find_device(RPLIDAR_IDENTIFIER)

    if arduino_port:
        print("Arduino Nano found on {}".format(arduino_port))
        update_launch_file(ARDUINO_LAUNCH_FILE, arduino_port)
        print("Port in Launch file adjusted")
    else:
        print("Arduino Nano not found!")

    if rplidar_port:
        print("RPLIDAR A1M8 found on {}".format(rplidar_port))
        update_launch_file(RPLIDAR_LAUNCH_FILE, rplidar_port)
        print("Port in Launch file adjusted")
    else:
        print("RPLIDAR A1M8 not found!")

if __name__ == "__main__":
    main()