# IoT Essentials - Group Project - Automated Chicken Coop Entry System
This repository contains scripts and files used for the IoT Essentials group project, part of the Electronics-ICT curriculum. 

---
## Prerequisite

## ThingSpeak Dashboard

For this project, reading & writing to a ThingSpeak dashboard is important.
**Make sure you have a account & channel created for this project!**
**Make sure to have your channel ID and MQTT Credentials on hand!**

## Raspberry Pi Pico

1. For our project, the Pico is programmed with **CircuitPy**. Install **CircuitPy** on your Pico using the link below.
- [CircuitPy](https://learn.adafruit.com/getting-started-with-raspberry-pi-pico-circuitpython/circuitpython)

2. To communicate with the Pico, we recommend using the Python code editor **Mu Editor**. This is a ligthweight editor
that can directly communicate with the Pico over serial.
Install **Mu Editor** via the link below.
- [Mu Editor](https://codewith.mu/)

3. To install additional libraries needed for this project on the Pico, the **Adafruit-CircuitPython-bundle** has to be installed.
Install the bundle using the following link.
- [Adafruit-CircuitPython-bundle](https://circuitpython.org/libraries)

4. When the **Adafruit-CircuitPython-bundle** is opened, go into the **lib** directory.
   Copy & paste the following libraries into your **Pico lib folder**.
   
   ```bash
   adafruit_bitmap_font
   adafruit_bus_device
   adafruit_display_text
   adafruit_displayio_layout
   adafruit_minimqtt
   adafruit_rgb_display
   adafruit_bh1750.mpy
   adafruit_connection_manager.mpy
   adafruit_framebuf.mpy
   adafruit_pcd8544.mpy
   adafruit_requests.mpy
   adafruit_ticks.mpy
   ```

6. After **cloning this repo**, copy the content of the **Pico folder** of this project into your **root directory of your Pico**.

   ```bash
   code.py
   ```

7. Inside your **root directory of your Pico**, create your own **secrets.py**. Put the following content inside.

   ```bash
   secrets = {
    # WiFi Credentials
    "ssid": 'your-wifi-name',
    "password": "your-wifi-password",
    # MQTT & ThingSpeak Credentials
    "mqtt_broker": "mqtt3.thingspeak.com",
    "mqtt_port": 1883,
    "mqtt_client_id": "your-mqtt-client-id",
    "mqtt_username": "your-mqtt-username",
    "mqtt_password": "your-mqtt-password",
    "mqtt_topic": "channels/your-channel-id/subscribe/fields/field1"
    }
   ```
8. After rebooting the Pico, it will automatically run code.py.

## Orange Pi

1. The following Python libraries are essential for this project’s functionality. 

   To install the required packages, use the following commands.

```bash
sudo apt install python3-pip
sudo pip3 install smbus2 paho.mqtt numpy tflite-runtime
or
sudo pip install smbus2 paho.mqtt numpy tflite-runtime
```

2. Additionally, install **WiringOP** and **WiringOP-Python** on your Orange Pi using the links below.

- [WiringOP](https://github.com/orangepi-xunlong/wiringOP)
- [WiringOP-Python](https://github.com/orangepi-xunlong/wiringOP-Python/blob/master/README.rst)

3. After cloning the repo on your Orange Pi, create your own **secrets.py**.

```bash
chicken_detector
pico
main.py
secrets.py <--- Create this
```
   Put the following content inside the file.

   ```bash
   secrets = {
       # MQTT & ThingSpeak Credentials
       "mqtt_broker": "mqtt3.thingspeak.com",
       "mqtt_port": 1883,
       "mqtt_client_id": "your-mqtt-client-id",
       "mqtt_username": "your-mqtt-username",
       "mqtt_password": "your-mqtt-password",
       "mqtt_topic": "channels/your-channel-id/publish"
   }
   ```
5. After cloning the repo on your Orange Pi, run the script with the following command.

```bash
sudo python3 main.py
```
