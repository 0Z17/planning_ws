#!/usr/bin/env python3

import struct
import rospy
import serial
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, WrenchStamped

hex_data = bytes([0x7B, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6A, 0x7D])
# hex_data = bytes([0x7B, 0x00, 0x00, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6B, 0x7D])

def find_frame(buffer):
    """get a frame from buffer"""
    while len(buffer) >= 2:  # the buffer should have at least two bytes
        # get the start byte
        if buffer[0] == 0x7B:
            # get the end byte
            try:
                end_index = buffer.index(0x7D, 1)   # find the end byte from the second byte
                frame = buffer[:end_index + 1]      # extract the frame from the buffer
                buffer = buffer[end_index + 1:]     # leave the remaining part in the buffer
                return frame, buffer
            except ValueError:
                # if no end byte found, wait for more data
                break
        else:
            # discard invalid bytes
            buffer.pop(0)
    return None, buffer

def xor_checksum(hex_list):
    """
    Calculate the XOR checksum for a list of hexadecimal numbers
    :param hex_list: Input list containing integers or hexadecimal strings (e.g. 0xA1 or 'A1')
    :return: Checksum as hexadecimal integer (e.g. 0x12), returns None for invalid input
    """
    checksum = 0x00  # XOR initialization value
    
    for item in hex_list:
        try:
            # Convert to integer
            num = int(item)
            
            # Validate single-byte range
            if not (0x00 <= num <= 0xFF):
                raise ValueError(f"Value {hex(num)} exceeds single-byte range")
            
            checksum ^= num  # Perform XOR operation
        
        except (ValueError, TypeError):
            print(f"Error: Invalid hex value - {item}")
            return None
    
    return checksum  # Returns checksum as hexadecimal integer

def end_effector_velocity_callback(data):
    global hex_data
    """callback function for end_effector_velocity topic"""
    x_vel = data.linear.x*1000.0
    y_vel = data.linear.y*1000.0
    z_vel = data.linear.z*1000.0
    # convert the velocity to hex data
    bytes_list = [0x7B, 0x00, 0x00, 0x11]
    bytes_list.extend(float_to_hex(x_vel))
    bytes_list.extend(float_to_hex(y_vel))
    bytes_list.extend(float_to_hex(z_vel))
    bytes_list.append(xor_checksum(bytes_list))
    bytes_list.append(0x7D)
    print([hex(byte) for byte in bytes_list])
    hex_data = bytes(bytes_list)
    # print(hex_data)

def float_to_hex(value):
    """convert a float to hex data"""
    high_val = (int(value) >> 8) & 0xFF
    low_val = int(value) & 0xFF
    return [high_val, low_val]

if __name__ == '__main__':
    rospy.init_node('serial_client')
    print("Serial client started")
    # get the serial port from the parameter server
    port = rospy.get_param('~port', '/dev/ttyUSB0')
    baud_rate = rospy.get_param('~baud_rate', 115200)
    # open the serial port
    ser = serial.Serial(port, baud_rate, timeout=1)
    print(f"Serial port {port} opened with baud rate {baud_rate}")
    force_pub = rospy.Publisher('/ft_sensor_topic', WrenchStamped, queue_size=10)
    rospy.Subscriber('/end_effector_velocity', Twist, end_effector_velocity_callback)

    rosrate = rospy.Rate(100)

    ser.write(hex_data)
    buffer = bytearray()
    
    while not rospy.is_shutdown():
        # rospy.spin()

        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            buffer.extend(data)
        
        frame, buffer = find_frame(buffer)
        if frame and len(frame) == 12:
            if frame[3] == 0x40:
                ser.write(hex_data)
            force_value = struct.unpack('<f', frame[4:8])[0]
            if force_value == 0:
                continue
            force_msg = WrenchStamped()
            force_msg.header.stamp = rospy.Time.now()
            force_msg.wrench.force.z = force_value
            force_pub.publish(force_msg)
            

        rosrate.sleep()