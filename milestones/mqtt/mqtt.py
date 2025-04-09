import paho.mqtt.client as mqtt
import json
from datetime import datetime

BROKER_ADDRESS = "192.168.137.1"
PORT = 1883
USERNAME = "mqtt"
PASSWORD = "MosquitoBroker"

TOPIC_WILDCARD = "/BlackBox_User01/+/+/sensor_data"
#BlackBox/User01/MAC/TEMP
#BlackBox/User01/MAC/TEMP
#BlackBox/User01/MAC/TEMP
#BlackBox/User01/MAC/TEMP...

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe(TOPIC_WILDCARD)
    print(f"Subscribed to topic: {TOPIC_WILDCARD}")
    print("Listening for sensor data...")

def on_message(client, userdata, msg):
    try:
        topic_parts = msg.topic.split('/')
        if len(topic_parts) >= 5:
            user_id = topic_parts[1]
            device_id = topic_parts[2]
            sensor_name = topic_parts[3]
        else:
            print(f"Invalid topic structure: {msg.topic}")
            return

        data = json.loads(msg.payload.decode())
        
        timestamp = datetime.fromtimestamp(data.get('timestamp', 0)).strftime("%Y-%m-%d %H:%M:%S")
        
        print("\n" + "="*60)
        print(f"Topic: {msg.topic}")
        print(f"Timestamp: {timestamp}")
        print(f"Device ID: {data.get('device_id', 'unknown')}")
        print(f"User ID: {data.get('user_id', 'unknown')}")
        print(f"Sensor: {sensor_name}")
        print("-"*60)
        print(f"Temperature: {data.get('temperature', 0):.2f}°C")
        print(f"Humidity: {data.get('humidity', 0):.2f}%")
        print(f"Light: {data.get('light', 0):.2f} lux")
        print("="*60 + "\n")
        
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON: {e}")
    except Exception as e:
        print(f"Error processing message: {e}")

def main():
    client = mqtt.Client()
    
    client.username_pw_set(USERNAME, PASSWORD)
    
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        print(f"Connecting to MQTT broker ({BROKER_ADDRESS})...")
        client.connect(BROKER_ADDRESS, PORT, 60)
        client.loop_forever()
        
    except Exception as e:
        print(f"Connection error: {e}")

if __name__ == "__main__":
    main()