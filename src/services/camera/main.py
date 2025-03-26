# import numpy as np
# from picamera2 import Picamera2
# import time
# import cv2

# # Create an instance of Picamera2
# picam2 = Picamera2()

# # Start the camera
# picam2.start()

# # Sleep to give the camera time to adjust settings (optional)
# time.sleep(2)

# # Capture the image to a NumPy array
# image_array = picam2.capture_array()

# # Stop the camera after capturing the image
# picam2.stop()

# # Display the captured image using OpenCV
# cv2.imshow("Captured Image", image_array)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# # Optionally save the image as a file
# cv2.imwrite("captured_image.jpg", image_array)

# print("Image captured and displayed.")


import cv2
import base64
import numpy as np
from paho.mqtt import client as mqtt
from paho.mqtt import enums as mqttEnums
from picamera2 import Picamera2
from time import sleep

class Camera:
    """
    Temporary class for testing the publishing of camera feed to other services.
    """
    def __init__(
            self,
            # broker = "mqtt-broker",
            # broker = "localhost",
            broker = "192.168.12.102",
            port = 1883,
            topic = "device/camera/frames"
    ) -> None:

        self.client = mqtt.Client(mqttEnums.CallbackAPIVersion(2))
        self.camera = None # set through initialization below
        self.state = True
        self.topic = topic

        # Connect to Broker and start camera
        try:
            self.client.on_connect = self._on_connect
            self.client.connect(broker, port, 60)
            self.client.loop_start()

        except Exception as e:
            self.stop()
            raise e

    def init(self) -> None:
        """Initializes camera before use"""
        print("Initializing camera...")
        # Open the camera
        self.camera = Picamera2()
        self.camera.configure(self.camera.create_preview_configuration(main={"size": (620,480), "format": "BGR888"}))
        self.camera.start()
        sleep(2)


    def start(self) -> None:
        """Starts capturing images and publishing to the specified topic."""
        count = 0
        while self.state:
            frame = self.camera.capture_array()
            count+= 1
            print(count, np.shape(frame))
            _, img_encoded = cv2.imencode('.jpg', frame)
            frame = img_encoded.tobytes()
            frame = base64.b64encode(frame).decode('utf-8')
            self.client.publish(self.topic, frame, qos=0)

    def stop(self) -> None:
        """Stop the camera: disconnect from client and close camera"""
        self.state = False
        if self.camera: self.camera.stop()
        if self.client.is_connected(): self.client.loop_stop()


    def _on_connect(self, client, userdata, flags, rc, properties) -> None:
        """On connect callback"""
        print(f"Connected with result code {rc}")


if __name__ == "__main__":
    camera = Camera()
    try:
        camera.init()
        camera.start()
    except KeyboardInterrupt:
        print("User manually stopped camera with CTRL+C...")
    except Exception as e:
        print(e)
    finally:
        print("Cleaning up camera connections...")
        camera.stop()
