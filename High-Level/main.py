import asyncio
from Recam import Recam
import cv2

recam = Recam(7445)  # Use same port as ESP32 (7445)
if recam.begin():
    try:
        while True:
            # Get image
            img = recam.imageData()
            if img is not None:
                cv2.imshow("Camera Feed", img)
                
            # Get robot data sent from ESP32
            robot_data = recam.getRobotData()
            print(f"Robot data: {robot_data}")
            
            # Send data back to ESP32 (5 int16 values)
            recam.setSendData([0, 0, 0, 0, 0])
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        recam.close()
        cv2.destroyAllWindows()