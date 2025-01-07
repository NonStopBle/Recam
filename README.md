# Recam Project

Recam is a real-time video streaming solution that enables high-performance camera streaming from an ESP32-CAM to a Python application over UDP. The project consists of two main components: an ESP32-CAM firmware and a Python library for receiving and processing the video stream.

## Features

- Real-time video streaming over UDP
- Bidirectional data communication (5 int16 values in each direction) / need to update can send multiple variable
- Low latency transmission
- Simple API for integration with other Python applications
- Configurable video quality and resolution
- Support for AI-THINKER ESP32-CAM module

## Requirements

### Hardware
- AI-THINKER ESP32-CAM module
- FTDI programmer or similar for uploading firmware

### Software
- Arduino IDE with ESP32 board support
- Python 3.6 or later
- Required Python packages:
  ```
  numpy
  opencv-python
  ```

## Installation

1. **ESP32-CAM Setup**
   - Install the Arduino IDE
   - Add ESP32 board support to Arduino IDE
   - Install required libraries:
     - `esp32` (board package)
     - `WiFi`
     - `WiFiUdp`
   - Upload the ESP32-CAM firmware from the `esp32cam` folder

2. **Python Library Setup**
   ```bash
   pip install numpy opencv-python
   ```

## Usage

### ESP32-CAM Configuration

1. Open the ESP32-CAM sketch in Arduino IDE
2. Configure your network settings:
   ```cpp
   const char *ssid = "your-network-ap";
   const char *password = "your-network-password";
   const char *udpAddress = "192.168.10.2";  // Your computer's IP address
   const int udpPort = 7445;
   ```
3. Upload the firmware to your ESP32-CAM

### Python Implementation

Basic example of using the Recam library:

```python
from recam import Recam
import cv2

# Initialize Recam
recam = Recam(7445)  # Use same port as configured in ESP32

# Start the receiver
if recam.begin():
    try:
        while True:
            # Get the latest image
            img = recam.imageData()
            if img is not None:
                # Display the image
                cv2.imshow("Camera Feed", img)
                
            # Get robot data sent from ESP32
            robot_data = recam.getRobotData()
            print(f"Received data: {robot_data}")
            
            # Send data back to ESP32 (5 int16 values)
            recam.setSendData([0, 0, 0, 0, 0])
            
            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        recam.close()
        cv2.destroyAllWindows()
```

## API Reference

### Class: Recam

#### Constructor
```python
recam = Recam(port)
```
- `port`: UDP port number (must match ESP32 configuration)

#### Methods

##### begin()
Starts the receiver thread.
- Returns: `bool` - True if successfully started

##### imageData()
Gets the latest received image.
- Returns: `numpy.ndarray` or `None` - OpenCV image format (BGR)

##### getRobotData()
Gets the latest robot data received from ESP32.
- Returns: `list` - 5 integers received from ESP32

##### setSendData(data)
Sets data to send back to ESP32.
- `data`: list of 5 integers (-32768 to 32767)

##### close()
Closes the connection and cleans up resources.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Author

- Author: Rezier
- YouTube: [@REZIER_0](https://www.youtube.com/@REZIER_0)
- GitHub: [NonStopBle](https://github.com/NonStopBle/Recam)

## Acknowledgments
- Special thanks to all contributors who helped improve this project ninzaza007
