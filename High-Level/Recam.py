# Recam Project
# This script streams camera data from an ESP32-CAM to a Python application over a UDP network in real-time.
# It can be integrated with image detection or other processing tasks for further applications.
# Author: Rezier [https://www.youtube.com/@REZIER_0]
# Date: (07/01/2025)
# License: MIT (see accompanying LICENSE file)

import socket
import struct
import numpy as np
import cv2
from collections import defaultdict
import time

DEST_PORT = 7445
MAX_UDP_PACKET_SIZE = 1400

# Packet Header Format: Frame ID (4 bytes), Seq Num (2 bytes), Total Packets (2 bytes), 10 bytes robot data
HEADER_FORMAT = "!IHH5H"  # Updated for the extra 2 bytes for robot data (5 shorts total, 10 bytes)
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

RESPONSE_FORMAT = "!5h"

send_time_previous = 0.00

def send_response(sock, addr, integers):
    global send_time_previous
    """
    Sends a response with 5 integers to the client.

    Args:
        sock: The UDP socket used for communication.
        addr: The client address to send the response to.
        integers: A list of 5 integers to send.
    """
    if len(integers) != 5:
        raise ValueError("Exactly 5 integers are required.")
    
    if any(i < -32768 or i > 32767 for i in integers):
        raise ValueError("over integer 16 bit")
    
    if((time.time() / 1000.0) - send_time_previous) :
        response_data = struct.pack(RESPONSE_FORMAT, *integers)
        sock.sendto(response_data, addr)
        send_time_previous = (time.time() / 1000.0)
    # print(f"Sent response to {addr}: {integers}")


trackBarData = [0,0,0,0,0]
def trackbarCallback_a(value) :
    trackBarData[0] = value
def trackbarCallback_b(value) :
    trackBarData[1] = value
def trackbarCallback_c(value) :
    trackBarData[2] = value
def trackbarCallback_d(value) :
    trackBarData[3] = value
def trackbarCallback_e(value) :
    trackBarData[4] = value


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", DEST_PORT))
    print(f"Listening for UDP packets on port {DEST_PORT}...")

    frame_buffers = defaultdict(lambda: defaultdict(bytes))
    received_packet_counts = {}

    cv2.namedWindow('Data')
    cv2.createTrackbar('Data_1' , 'Data' ,  -32767 , 32767 , trackbarCallback_a)
    cv2.createTrackbar('Data_2' , 'Data' ,  -32767 , 32767, trackbarCallback_b)
    cv2.createTrackbar('Data_3' , 'Data' ,  -32767 , 32767, trackbarCallback_c)
    cv2.createTrackbar('Data_4' , 'Data' ,  -32767 , 32767, trackbarCallback_d)
    cv2.createTrackbar('Data_5' , 'Data' ,  -32767 , 32767, trackbarCallback_e)
    
    
    try:
        while True:
            # Receive data
            data, addr = sock.recvfrom(MAX_UDP_PACKET_SIZE + HEADER_SIZE)

            # Parse the header
            if len(data) < HEADER_SIZE:
                print("Received packet too small, ignoring.")
                continue

            # Unpack the updated header with the extra byte
            header = struct.unpack(HEADER_FORMAT, data[:HEADER_SIZE])
            frame_id, packet_seq_num, total_packets, *robot_data = header
            robot_data_array = np.array(robot_data, dtype=np.int16)
            
            print(robot_data_array)

            # Log header details
            # print(f"Header - Frame ID: {frame_id}, Seq Num: {packet_seq_num}, Total Packets: {total_packets}, Robot Data: {robot_data_array}")

            if frame_id not in frame_buffers:
                frame_buffers[frame_id] = [None] * total_packets
                received_packet_counts[frame_id] = 0

            if packet_seq_num >= total_packets:
                print(f"Warning: Invalid packet sequence number {packet_seq_num} for frame {frame_id}, skipping.")
                continue

            if frame_buffers[frame_id][packet_seq_num] is None:
                frame_buffers[frame_id][packet_seq_num] = data[HEADER_SIZE:]
                received_packet_counts[frame_id] += 1

            if received_packet_counts[frame_id] == total_packets:
                # print(f"Frame {frame_id} fully received. Reconstructing...")

                frame_data = b''.join(packet for packet in frame_buffers[frame_id] if packet)

                # Decode the image
                frame_array = np.frombuffer(frame_data, dtype=np.uint8)
                img = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)

                if img is not None:
                    # print(f"Displaying Frame ID: {frame_id}")
                    cv2.imshow("Data", img)
                    cv2.waitKey(1)  # Allow OpenCV to update the window

                    send_response(sock, addr, trackBarData)
                else:
                    print(f"Failed to decode frame {frame_id}")

                # Clean up buffers for the completed frame
                del frame_buffers[frame_id]
                del received_packet_counts[frame_id]

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        sock.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
