import paho.mqtt.client as mqtt
from datetime import datetime

BROKER_ADDRESS = "192.168.137.1"
PORT = 1883
USERNAME = "mqtt"
PASSWORD = "MosquitoBroker"
TOPIC_WILDCARD = "BlackBox/+/+/#"  

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe(TOPIC_WILDCARD)
    print(f"Subscribed to topic: {TOPIC_WILDCARD}")
    print("Listening for sensor data...")

def on_message(client, userdata, msg):
    try:
        topic_parts = msg.topic.split('/')
        if len(topic_parts) != 4:
            print(f"Invalid topic structure: {msg.topic}")
            return

        sensor_type = topic_parts[3] 

        value = float(msg.payload.decode())

        
        print("\n" + "="*60)
        print(f"Topic: {msg.topic}")
        print("-"*60)
        
        if sensor_type == "temperature":
            print(f"Temperature: {value:.2f}°C")
        elif sensor_type == "pressure":
            print(f"Pressure: {value:.2f} hPa")
            
        print("="*60 + "\n")
        
    except ValueError as e:
        print(f"Error parsing value: {e}")
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