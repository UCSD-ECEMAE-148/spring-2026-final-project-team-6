#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Float32

import json
import threading
import numpy as np
import paho.mqtt.client as mqtt

#Configurations

MQTT_BROKER    = 'broker.emqx.io'
MQTT_PORT      = 1883
MQTT_CLIENT_ID = 'robocar-ros2-bridge'
MQTT_TOPIC     = 'robocar/scan'    # change  this to your different command/topics you will use later
COMMAND_TOPIC  = 'robocar/command'

#global variables for this implementation


thermal_data    = np.zeros((8, 8))
thermistor_temp = 0.0
prediction      = 'empty'
confidence      = 0.0
mac_address     = 'unknown'
new_data        = False

#MQTT callbacks

def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code {rc}")
    client.subscribe(MQTT_TOPIC)
    print(f"Subscribed to topic: {MQTT_TOPIC}")

    # Comunicating with esp32
    client.publish(COMMAND_TOPIC, 'auto')
    print(f"[CMD] Sent 'auto' to ESP32 on {COMMAND_TOPIC}")

def on_disconnect(client, userdata, rc):
    print(f"Disconnected from MQTT broker (rc={rc})")
    if rc != 0:
        print("[MQTT] Unexpected disconnect — will attempt reconnect")

def on_message(client, userdata, msg):
    global thermal_data, thermistor_temp, prediction, confidence, mac_address, new_data

    # Parse JSON: depending on what you expect this will look different
    data = json.loads(msg.payload.decode())

    # Extract fields from ESP32 payload
    prediction      = data.get('prediction',  'empty')
    confidence      = float(data.get('confidence',  0.0))
    mac_address     = data.get('mac_address', 'unknown')
    thermistor_temp = float(data['thermistor'])
    pixels          = data['pixels']

    # Sanity check
    if len(pixels) != 64:
        print(f"[Warn] Expected 64 pixels, got {len(pixels)}")
        return

    # Reshape - for a nice to have that may or may not be implemented
    thermal_data = np.array(pixels, dtype=float).reshape(8, 8)

    # Compute stats
    max_temp = float(np.max(thermal_data))
    min_temp = float(np.min(thermal_data))

    # Print status
    print(f"[Thermal] Ambient={thermistor_temp:.1f}C  "
          f"Max={max_temp:.1f}C  Min={min_temp:.1f}C  "
          f"Prediction={prediction} ({confidence:.2%})")

    # Signal ROS2 node that new data is ready to publish
    new_data = True

class ThermalMqttBridgeNode(Node):

    def __init__(self, mqtt_client):
        super().__init__('thermal_mqtt_bridge_node')
        self.get_logger().info('Thermal MQTT Bridge Node started!')
        self.mqtt_client = mqtt_client

        # ── Publishers ──
        self.detection_pub  = self.create_publisher(String,'/thermal/detection',10)
        self.pixels_pub     = self.create_publisher(Float32MultiArray,'/thermal/pixels',10)
        self.thermistor_pub = self.create_publisher(Float32,'/thermal/thermistor',10)

        # Poll for new MQTT data at 20Hz(half a second, rate of current implentations esp32 publish rate)
        self.timer = self.create_timer(0.05, self.publish_if_new)


    def publish_if_new(self):
        """Publish to ROS2 whenever new MQTT data has arrived."""
        global new_data

        if not new_data:
            return
        new_data = False

        # ── /thermal/detection ──
        detection_msg      = String()
        detection_msg.data = f'{prediction}:{confidence:.4f}'
        self.detection_pub.publish(detection_msg)

        # ── /thermal/pixels ──
        pixels_msg      = Float32MultiArray()
        pixels_msg.data = thermal_data.flatten().tolist()
        self.pixels_pub.publish(pixels_msg)

        # ── /thermal/thermistor ──
        therm_msg      = Float32()
        therm_msg.data = thermistor_temp
        self.thermistor_pub.publish(therm_msg)

        # Warn if person detected, will be it's own node
        if prediction == 'present':
            self.get_logger().warn(
                f'PERSON DETECTED | Confidence: {confidence:.2%} | '
                f'Ambient: {thermistor_temp:.1f}C | MAC: {mac_address}'
            )

def main(args=None):
    # Setup MQTT client
    client = mqtt.Client(client_id=MQTT_CLIENT_ID)
    client.on_connect    = on_connect
    client.on_disconnect = on_disconnect
    client.on_message    = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)

    # Run MQTT loop in background thread
    mqtt_thread = threading.Thread(target=client.loop_forever, daemon=True)
    mqtt_thread.start()

    # Start ROS2 node
    rclpy.init(args=args)
    node = ThermalMqttBridgeNode(client)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Tell ESP32 to stop before disconnecting
        print("[CMD] Sending 'stop' to ESP32...")
        client.publish(COMMAND_TOPIC, 'stop')
        client.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
