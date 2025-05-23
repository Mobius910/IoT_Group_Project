import time
import board
import busio
import digitalio
import wifi
import socketpool
import ssl
import adafruit_requests
import adafruit_pcd8544
import adafruit_minimqtt.adafruit_minimqtt as MQTT
from secrets import secrets

# Default Gate Status
gate_status = None

#########################
# ---- Wi-Fi Setup ---- #
#########################

def wifi_connection():
    # Wi-Fi credentials
    ssid = secrets["ssid"]
    password = secrets["password"]

    # Connect to Wi-Fi
    print("Connecting to Wi-Fi...")
    while True:
        try:
            wifi.radio.connect(ssid, password)
            print("Connection Successful!")
            print(f"Connected to {ssid}")
            print(f"IP address : {wifi.radio.ipv4_address}")
            break
        except Exception as e:
            print(f"Connection failed : {e}")

#####################################
# ---- MQTT & ThingSpeak Setup ---- #
#####################################

def setup_mqtt():
    # MQTT client configuration
    MQTT_BROKER = secrets["mqtt_broker"]
    MQTT_PORT = secrets["mqtt_port"]
    MQTT_KEEPALIVE_INTERVAL = 60
    
    MQTT_CLIENT_ID = secrets["mqtt_client_id"]
    MQTT_USER = secrets["mqtt_username"]
    MQTT_PWD = secrets["mqtt_password"]
    MQTT_TOPIC = secrets["mqtt_topic"]

    # Creates a socket pool
    pool = socketpool.SocketPool(wifi.radio)

    # Setup MiniMQTT client
    mqtt_client = MQTT.MQTT(
        broker=MQTT_BROKER,
        port=MQTT_PORT,
        username=MQTT_USER,
        client_id=MQTT_CLIENT_ID,
        password=MQTT_PWD,
        socket_pool=pool,
        ssl_context=None
    )

    # Message callback for received MQTT messages
    def message_callback(client, topic, message):
        global gate_status
        gate_status = message
        print(f"Received: {message}")

    mqtt_client.on_message = message_callback

    # Connect & Subscribe to ThingSpeak dashboard
    print("Connecting to MQTT broker...")
    mqtt_client.connect()
    mqtt_client.subscribe(MQTT_TOPIC)
    print(f"Subscribed to {MQTT_TOPIC}")
    return mqtt_client


######################################################
# ---- LCD Display (PCD8544 - Nokia 5110) Setup ---- #
######################################################

def main():
    # Initiate a connection with local WiFi
    wifi_connection()

    # Initiate MQTT connection with ThingSpeak
    mqtt_client = setup_mqtt()

    # Create SPI bus (CLK|SCLK=GP6, DIN|MOSI=GP7)
    spi = busio.SPI(clock=board.GP6, MOSI=board.GP7)

    # Control pins (DC=GP4, CE|CS=GP2, RST=GP3)
    dc = digitalio.DigitalInOut(board.GP4)  # Data/Command select
    cs = digitalio.DigitalInOut(board.GP2)  # Chip Select (CE)
    reset = digitalio.DigitalInOut(board.GP3)  # Reset

    # Initialize LCD Display
    display = adafruit_pcd8544.PCD8544(spi, dc, cs, reset)

    # Set optional display parameters
    display.bias = 5
    display.contrast = 46

    # Maintains ThingSpeak MQTT connection to constantly receive
    # gate status updates and continuously refresh the display
    try:
        while True:
            # Receive the latest gate status from ThingSpeak
            if not mqtt_client.is_connected():
                mqtt_client.reconnect()
            mqtt_client.loop()
            # Update the display based on the received value
            if gate_status is not None:
                display.fill(0)
                display.text("The gate is :", 0, 16, 1)
                if gate_status == "1":
                    display.text(f"     OPEN", 0, 24, 1)
                    display.show()
                else:
                    display.text(f"    CLOSED", 0, 24, 1)
                    display.show()
            # Wait for 15 seconds before the next request
            time.sleep(15)
    except KeyboardInterrupt:
        display.fill(0)
        display.show()

if __name__ == "__main__":
    main()
