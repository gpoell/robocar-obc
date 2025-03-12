import cv2
import base64
import numpy as np
from paho.mqtt import client as mqtt
from paho.mqtt import enums as mqttEnums

class Camera:
    """
    Temporary class for testing the publishing of camera feed to other services.
    """
    def __init__(
            self,
            broker = "mqtt-broker",
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
            # Open the camera
            self.camera = cv2.VideoCapture(0)

            # Check if the camera opened successfully
            if not self.camera.isOpened():
                raise ValueError("Error: Could not open webcam.")

        except Exception as e:
            self.stop()
            raise e

    def start(self) -> None:
        """Starts capturing images and publishing to the specified topic."""
        while self.state:
            try:
                ret, frame = self.camera.read()
            except Exception as e:
                print(e)
                raise ValueError("Error: Failed to capture image.")
            # if not ret:
                # raise ValueError("Error: Failed to capture image.")
            print(np.size(frame))
            _, img_encoded = cv2.imencode('.jpg', frame)
            frame = img_encoded.tobytes()
            frame = base64.b64encode(frame).decode('utf-8')
            self.client.publish(self.topic, frame, qos=0)

    def stop(self) -> None:
        """Stop the camera: disconnect from client and close camera"""
        self.state = False
        if self.camera: self.camera.release()
        if self.client.is_connected(): self.client.loop_stop()
        cv2.destroyAllWindows()

    def _on_connect(self, client, userdata, flags, rc, properties) -> None:
        """On connect callback"""
        print(f"Connected with result code {rc}")


if __name__ == "__main__":
    camera = Camera()
    try:
        camera.start()
    except KeyboardInterrupt:
        print("User manually stopped camera with CTRL+C...")
    except Exception as e:
        print(e)
    finally:
        print("Cleaning up camera connections...")
        camera.stop()
