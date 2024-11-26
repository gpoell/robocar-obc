@dataclass
class ClientConfig:
    host: str = "mqtt-broker"
    port: int = 1883

class MqttClient:
    def __init__(self, host, port, topics):
        self.host = host
        self.port = port
        self.topics = topics
        self.connected = False

        def connect_mqtt(self, host: str, port: int) -> bool:
            # Connect to MQTT Client
            mqtt_conn_err = self.client.connect(host, port)
            if mqtt_conn_err:
                raise ValueError(f"Connection to MQTT broker failed with error code: {mqtt_conn_err}") # Expand this
                return False
            
            # Subscribe to topics
            subscriptions = self.topics.get_subscribers()
            self.client.subscribe(subscriptions)

            # Start Client
            mqtt_loop_error = self.client.loop_start()
            if mqtt_loop_error:
                raise ValueError(f"MQTT Client Loop failed with error code: {mqtt_loop_error}") # Expand this
                return False

            return True


    def validate_msg_publish(self, mqttMessage, timeout=1.0) -> bool:
        mqttMessage.wait_for_publish(timeout=timeout)
        if not mqttMessage.is_published():
            print("Failed to send message to topic " + self.topics.publishers.distance.topic)

            if not self.client.is_connected():
                print("Client not connected, exiting...")
                return False
        return True
