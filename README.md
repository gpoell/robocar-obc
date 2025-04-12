# robocar-obc
Source code for a self driving RC car with a Raspberry Pi 4. Some parts of the vehicle run as containerized microservices,
distrubting data over the MQTT message/broker service to inform the vehicle's driving logic. Below is an overview of how
the repository is organized.

### Vehicle
The vehicle.py is the primary component for operating the RC car. It actively listens for data collected by the ultrasonic sensor,
infrared sensors, and the camera. This data is used to create driving pipelines that enable the vehicle's obstacle and lane
detection, driving algorithms, and determining the overall health of the system.

### Mosquitto - MQTT Message/Broker Protocol
The vehicle uses the MQTT message/broker protocol for distributing data throughout the vehicle. It decouples how components
publish and consume data with low network bandwidth, enabling new components to scale and distribute data consistently. The
mosquitto/ contains the configuration for starting the MQTT broker.

### Services
Docker containers running as a service for each part or component. Containers copy in their respective part and client
configuration, and run a main() process that sets the parts topics and executes its primary process run().
* camera: publishes frames from the RPI camera to the telemetry dashboard and lane detection models
* infrared: publishes infrared data to the vehicle and telemetry dashboard
* ultrasonic: publishes ultrasonic data to the vehicle and telemetry dashboard
* vehicle: primary service subscribing to sensor data and system health metrics
* listener: simple service for quickly testing if data is being published
* unittests: service for executing unit tests


### Parts
All of the parts and subcomponents that make up the vehicle.
* Motor: the motor supplies power to the 4 wheels of the vehicle
* ADC: useful for reading the battery power
* PCA9685: PWM servo driver primarily used for controlling the Motor, Servos, and LEDs.
* Ultrasonic: ultrasonic sensor mounted at the front of the vehicle and used to determine the proximity of the vehicle to other objects
* Infrared: infrared sensors mounted at the front of the vehicle and used for lane detection
* Camera: used for streaming images for telemetry and machine learning algorithms for lane detection

### Client
Topics for various components are configured here, along with a MQTT Device wrapper class used by various parts
to publish and subscribe to data using the MQTT message/broker protocol.

### Pipelines
Contains scripts for driving and lane detection pipelines.