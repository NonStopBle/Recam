# Recam Project
# This script streams camera data from an ESP32-CAM to a Python application over a UDP network in real-time.
# It can be integrated with image detection or other processing tasks for further applications.
# Author: Rezier [https://www.youtube.com/@REZIER_0]
# Date: (07/01/2025)
# License: MIT (see accompanying LICENSE file)

import threading
import socket
import struct
import numpy as np
from collections import defaultdict
import time
import cv2
import select

class Recam:
    def __init__(self, _port):
        self._port = _port
        # Match ESP32 constants exactly
        self._max_udp_packet_size = 1400  # Matches ESP32's MAX_UDP_PACKET_SIZE
        # Header: 4 bytes frameId, 2 bytes packetSeqNum, 2 bytes totalPackets, 10 bytes robot data
        self._header_format = "!IHH5H"  # Matches ESP32's packet structure
        self._header_size = struct.calcsize(self._header_format)
        self._response_format = "!5h"  # For sending 5 int16_t values back
        self._send_time_previous = 0.00

        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.bind(("0.0.0.0", self._port))
            self._sock.setblocking(False)
            print(f"Listening for UDP packets on port {self._port}...")
        except socket.error as e:
            print(f"Error creating or binding socket: {e}")
            self._sock = None
            return

        self._frame_buffers = defaultdict(lambda: defaultdict(bytes))
        self._received_packet_counts = {}
        self._image_recv = np.array([])
        self._send_data = [0] * 5  # Initialize with 5 zeros like ESP32's robotDataSend
        self._lock = threading.Lock()
        self._running = True
        self.receiver_thread = threading.Thread(target=self._receive_data, daemon=True)
        self._robot_data = [0] * 5  # Store received robot data

    def _send_response(self, sock, addr, integers):
        """Send response matching ESP32's expected format"""
        if len(integers) != 5:
            return
        
        if any(i < -32768 or i > 32767 for i in integers):  # Check int16_t range
            return

        current_time = time.time()
        if (current_time - self._send_time_previous) >= 0.001:  # Rate limiting
            try:
                response_data = struct.pack(self._response_format, *integers)
                sock.sendto(response_data, addr)
                self._send_time_previous = current_time
            except Exception as e:
                print(f"Error sending response: {e}")

    def _receive_data(self):
        while self._running:
            try:
                if self._sock is None:
                    break

                ready_to_read, _, _ = select.select([self._sock], [], [], 0.001)
                if not ready_to_read:
                    continue

                # Receive packet
                data, addr = self._sock.recvfrom(self._max_udp_packet_size + self._header_size)
                
                if len(data) < self._header_size:
                    continue

                # Parse header exactly as ESP32 formats it
                header = struct.unpack(self._header_format, data[:self._header_size])
                frame_id, packet_seq_num, total_packets, *robot_data = header
                
                # Store robot data
                with self._lock:
                    self._robot_data = [val for val in robot_data]

                # Process frame data
                if frame_id not in self._frame_buffers:
                    self._frame_buffers[frame_id] = [None] * total_packets
                    self._received_packet_counts[frame_id] = 0

                if packet_seq_num >= total_packets:
                    continue

                if self._frame_buffers[frame_id][packet_seq_num] is None:
                    # Store image data (after header)
                    self._frame_buffers[frame_id][packet_seq_num] = data[self._header_size:]
                    self._received_packet_counts[frame_id] += 1

                    if self._received_packet_counts[frame_id] == total_packets:
                        # Process complete frame
                        frame_data = b''.join(packet for packet in self._frame_buffers[frame_id] if packet)
                        frame_array = np.frombuffer(frame_data, dtype=np.uint8)
                        img = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)

                        if img is not None:
                            with self._lock:
                                self._image_recv = img
                                self._send_response(self._sock, addr, self._send_data)

                        # Cleanup
                        del self._frame_buffers[frame_id]
                        del self._received_packet_counts[frame_id]

            except Exception as e:
                if not isinstance(e, socket.error):
                    print(f"Receive error: {e}")
                time.sleep(0.01)

    def getRobotData(self):
        """Get the latest robot data received from ESP32"""
        with self._lock:
            return self._robot_data.copy()

    def imageData(self):
        """Get the latest image received"""
        with self._lock:
            return self._image_recv.copy() if self._image_recv.size > 0 else None

    def setSendData(self, data):
        """Set data to send back to ESP32"""
        if len(data) == 5 and all(-32768 <= x <= 32767 for x in data):
            with self._lock:
                self._send_data = data.copy()

    def begin(self):
        """Start the receiver thread"""
        if self._sock is None:
            print("Socket initialization failed, unable to start receiver.")
            return False
        self.receiver_thread.start()
        return True

    def close(self):
        """Clean shutdown"""
        self._running = False
        if self._sock:
            self._sock.close()