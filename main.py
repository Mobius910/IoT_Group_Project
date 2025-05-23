import wiringpi as wp
from smbus2 import SMBus, i2c_msg
import paho.mqtt.client as mqtt
import numpy as np
import tflite_runtime.interpreter as tflite
import time
import threading
import cv2
import subprocess
import os
from secrets import secrets

###################################################
# ==== Global Configuration & Initialization ==== #
###################################################

# === MQTT Client Configuration === #

MQTT_BROKER = secrets["mqtt_broker"]
MQTT_PORT = secrets["mqtt_port"]
MQTT_KEEPALIVE = 60

MQTT_CLIENT_ID = secrets["mqtt_client_id"]
MQTT_USER = secrets["mqtt_username"]
MQTT_PASSWORD = secrets["mqtt_password"]
MQTT_TOPIC = secrets["mqtt_topic"]

# === Gate Motor, PWM & Button Initializing === #
motor_pin = [3, 4, 5, 6] # Physical pins : [8, 10, 11, 12]
button_pin = 16 # Physical pin : 26
pwm_pin = 2 # Physical pin : 7
gate_status = 0
gate_status_lock = threading.Lock()
step_seq = [
    [1, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 1],
    [1, 0, 0, 1]
]

# === AI Detection Initializing === #
access_detection_lock = threading.Lock()
access_detection = "2" # 0 = chicken; 1 = fox; 2 = background
latest_animal = "none"

CONFIDENCE_THRESHOLD = 0.5
IMAGE_PATH = "./capture.jpg"

# === Day/Night Cycle Initializing === #
daytime = False
day_night_lock = threading.Lock()

####################################################
# ==== Chicken Detector (AI Detection System) ==== #
####################################################

def chicken_detector():
    def verify_webcam_functionality():
        # Command captures a frame with the webcam
        # and if was taken and webcam exists, return True
        result = subprocess.run([
            "fswebcam",
            "-d", "/dev/video1",
            "--no-banner",
            "-r", "1280x720",
            "-S", "5",
            IMAGE_PATH
        ], capture_output=True)
        return result.returncode == 0

    
    print("Starting Chicken Detector Thread...")
    global gate_status, access_detection, latest_animal

    # Loads labels file, a mapping file between number IDs (used by AI & it's TFLite model) and names
    labels = {}
    with open("/home/orangepi/iot/group_project/chicken_detector/labels.txt", "r") as f:
    # Puts content of file in labels dictionary
        for line in f:
            pair = line.strip().split(maxsplit=1)
            if len(pair) == 2:
                labels[int(pair[0])] = pair[1]

    # Loads TFLite model file, training model used to train AI to detect chickens & foxes
    interpreter = tflite.Interpreter(model_path="/home/orangepi/iot/group_project/chicken_detector/vww_96_grayscale_quantized.tflite")
    # Create Interpreter object to run the AI model on the OrangePi
    # and give it the needed resources
    interpreter.allocate_tensors()
    # Give AI the needed I/O information
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    # AI continuously checks via webcam if a chicken or fox is at gate.
    while True:
        # Checks if webcam exists
        if not verify_webcam_functionality():
            print("Failed to capture image from /dev/video1.")
            time.sleep(1)
            continue

        # Check if captured frame was taken & webcam works
        frame = cv2.imread(IMAGE_PATH)
        if frame is None:
            print("Error reading captured image.")
            time.sleep(1)
            continue

        # converts captured frame to grayscale for model to work
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # resize captured frame to 96x96 for model to work
        resized = cv2.resize(gray, (96, 96))
        # prepares captured frame input data shape & type
        input_data = np.expand_dims(resized, axis=(0, -1)).astype(np.float32)

        # Normalize input for float32 model to work better & faster
        input_data = input_data / 255.0

        # Loads captured frame into AI
        interpreter.set_tensor(input_details[0]['index'], input_data)
        # Executes model on captured frame
        interpreter.invoke()
        # retrieve AI output
        output = interpreter.get_tensor(output_details[0]['index'])[0]

        # AI gives index score if captured frame is closest to a background, chicken or fox
        predicted_index = int(np.argmax(output))
        # compares to label file and gives the most confidently accurate result
        predicted_label = labels.get(predicted_index, "Unknown").lower()
        confidence = float(output[predicted_index])

        if confidence < CONFIDENCE_THRESHOLD:
            # Low confidence, ignore detection, set to Background
            print(f"Low confidence ({confidence:.2f}) detection ignored, treated as background.")
            time.sleep(1)
            continue

        # will open/close gate based on confidently accurate detection result
        with gate_status_lock:
            if predicted_label == "chicken":
                # Chicken detected: open gate for 10 seconds then close
                if not gate_status:
                    print("Chicken detected - Opening gate for 10 seconds...")
                    control_gate(step_seq, 120, motor_pin, reverse=True)
                    gate_status = 1
                    with access_detection_lock:
                        access_detection = "0"
                        latest_animal = "chicken"
                    time.sleep(10)
                    print("Closing gate after chicken passed...")
                    control_gate(step_seq, 120, motor_pin, reverse=False)
                    gate_status = 0
                else:
                    print("Gate already open for chicken.")
                    with access_detection_lock:
                        access_detection = "0"
                        latest_animal = "chicken"
            elif predicted_label == "foxes":
                # Fox detected: close gate immediately
                if gate_status:
                    print("Fox detected - Closing gate immediately!")
                    control_gate(step_seq, 120, motor_pin, reverse=False)
                    gate_status = 0
                    with access_detection_lock:
                        access_detection = "1"
                        latest_animal = "fox"
                else:
                    print("Fox detected - Gate already closed.")
                    with access_detection_lock:
                        access_detection = "1"
                        latest_animal = "fox"
            elif predicted_label == "background":
                # Background detected: do nothing
                continue
            else:
                # Unknown label: treat as background (no action)
                print(f"Unknown label '{predicted_label}' detected - No action taken.")
        time.sleep(1)

####################################
# ==== Gate Status & Control  ==== #
####################################

def control_gate(step_seq, steps, pins, reverse=True, delay=0.00195):
    # Communicates to steppe motor to open/close the gate
    sequence = list(reversed(step_seq)) if reverse else step_seq
    direction = "Opening" if reverse else "Closing"
    print(f"{direction} Gate...")
    for _ in range(steps):
        for step in sequence:
            for pin, state in zip(pins, step):
                wp.digitalWrite(pin, state)
            time.sleep(delay)

def status_gate(button_pin, motor_pin):
    global gate_status
    prev_button_state = 1
    print("Status Gate thread started...")

    while True:
        # When button pressed, it opens/closes the gate
        button_state = wp.digitalRead(button_pin)
        if button_state == 0 and prev_button_state == 1:
            print("-----------------------")
            print("Button pressed")
            with gate_status_lock, day_night_lock:
                if daytime:
                # When it's daytime, gate is opened/closed until button pressed again
                    if gate_status:
                        control_gate(step_seq, 120, motor_pin, reverse=False)
                        gate_status = 0
                        print("Gate closed...")
                    else:
                        control_gate(step_seq, 120, motor_pin, reverse=True)
                        gate_status = 1
                        print("Gate open...")
                else:
                # When it's nighttime, gate is open for 10s before closing
                    if not gate_status:
                        control_gate(step_seq, 120, motor_pin, reverse=True)
                        gate_status = 1
                        print("Gate open for 10 seconds...")
                        time.sleep(10)
                        control_gate(step_seq, 120, motor_pin, reverse=False)
                        gate_status = 0
            time.sleep(0.3)
        prev_button_state = button_state
        time.sleep(0.01)

##################################
# ==== BH1750 Light Sensor  ==== #
##################################

def bh1750_values(bus, addr):
    # Reading BH1750 Lux values
    # 1 lux high resolution measurement & 120ms measure time (see datasheet)
    write = i2c_msg.write(addr, [0x10]) 
    # 2 bytes size read request (see datasheet)
    read = i2c_msg.read(addr, 2) 
    bus.i2c_rdwr(write, read)
    bytes_read = list(read)
    # Byte to lux conversion (see datasheet)
    return (((bytes_read[0] & 3) << 8) + bytes_read[1]) / 1.2

#############################
# ==== Day/Night Cycle ==== #
#############################

def day_night_cycle(bh1750_bus, bh1750_addr, pwm, threshold_lux):
    print("Day/Night cycle thread started...")
    global daytime, gate_status
    
    # Control how fast Sunrise/Sunset happens
    wait = 5 # Delay (in seconds) between each PWM adjustment step
    pwm_step = 5 # How much brightness increases/decreases per step
    
    # Control how long a single day/night is (in seconds)
    duration = 180
    
    print("--- Initial Night ---")
    # Initial Night : PWM off, gate closed
    wp.softPwmWrite(pwm, 0)
    with day_night_lock:
        daytime = False
    with gate_status_lock:
        if gate_status:
            control_gate(step_seq, 120, motor_pin, reverse=False)
            gate_status = 0
    time.sleep(duration)

    # The day/night cycle loops continuously until the program is manually stopped or exits.
    while True:
        print("--- Sunrise Starting ---")
        # Sunrise : PWM brightness increases to match threshold lux
        brightness = 0
        while brightness <= 100:
            lux = bh1750_values(bh1750_bus, bh1750_addr)
            control_pwm(pwm, brightness, wait)
            if lux >= threshold_lux:
                break
            brightness += pwm_step
        # When threshold lux is reached, gate opens and day starts
        with day_night_lock:
            daytime = True
        with gate_status_lock:
            if not gate_status:
                control_gate(step_seq, 120, motor_pin, reverse=True)
                gate_status = 1
        print("--- Day Started ---")
        # Day: PWM on, gate open - dynamically adjust brightness during the day
        start_time = time.time()
        while time.time() - start_time < duration:
            lux = bh1750_values(bh1750_bus, bh1750_addr)
            with day_night_lock:
                if daytime:
                    brightness = adjust_pwm(lux, threshold_lux, brightness)
                    control_pwm(pwm, brightness, wait)
            time.sleep(wait)  # Wait between adjustments

        print("--- Sunset Starting ---")
        # Sunset : PWM brightness decreases until PWM is off
        with day_night_lock:
            daytime = False
        while brightness >= 0:
            lux = bh1750_values(bh1750_bus, bh1750_addr)
            control_pwm(pwm, brightness, wait)
            if brightness == 0:
                break
            brightness -= pwm_step
        # When PWM is off, gate closes and night starts
        with gate_status_lock:
            if gate_status:
                control_gate(step_seq, 120, motor_pin, reverse=False)
                gate_status = 0
        print("--- Night Started ---")
        # Night : PWM off, gate closed
        time.sleep(duration)
        
#######################################
# ==== PWM Control & Adjustment  ==== #
#######################################

def adjust_pwm(lux, threshold_lux, brightness):
    # Adjust PWM brightness to match threshold lux
    
    # lux difference between current and threshold lux
    lux_difference = threshold_lux - lux

    # if current lux is within 2 lux of baseline, stay same
    if abs(lux_difference) < 2:
        print("Lux is within the range...\n")
    else:
        # if current lux is not within range, automatically control brightness
        if threshold_lux > lux:
            brightness += 5  # Increase brightness
            print("Increasing light brightness...\n")
        else:
            brightness -= 5  # Decrease brightness
            print("Decreasing light brightness or turning off light...\n")
    # Ensure brightness is within 0-100% and convert to int
    brightness = int(max(0, min(100, brightness)))                
    print(f"PWM brightness : {brightness}%\n")
    return brightness

def control_pwm(sig, cnt, wait):
    # Control LED with SoftPWM
    wp.softPwmWrite(sig, cnt)
    time.sleep(wait)

#############################
# ==== MQTT Callbacks  ==== #
#############################

def on_connect(client, userdata, flags, rc, properties):
    # Called on connection with MQTT broker
    print("Connected OK" if rc == 0 else f"Connection failed: {rc}")

def on_disconnect(client, userdata, rc, properties):
    # Called on disconnection with MQTT broker
    print(f"Disconnected with code {rc}")

def on_message(client, userdata, msg):
    # Called when a message is received from a subscribed topic 
    print(f"Received message on {msg.topic}: {msg.payload}")

##################
# ==== Main ==== #
##################

def main():
    # I2C Setup BH1750
    bh1750_bus = SMBus(0)
    bh1750_addr = 0x23
    
    # WiringPi Initializing
    wp.wiringPiSetup()

    # Global Variables
    global button_pin, motor_pin, pwm_pin, step_seq, gate_status

    # Input Threshold Lux (Lux amount when daytime starts)
    threshold_lux = float(input("What is the threshold lux?\n(When is it bright enough for start of daytime...)\n"))

    # Set SoftPWM pin & Start PWM
    wp.softPwmCreate(pwm_pin, 0, 100)
    wp.softPwmWrite(pwm_pin, 0)

    # Set Button pin & pull-down resistor configuration on Button pin
    wp.pinMode(button_pin, 0)
    wp.pullUpDnControl(button_pin, 2)

    # Set Steppe Motor pins
    for pin in motor_pin:
        wp.pinMode(pin, 1)

    # Setup MQTT Client
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, MQTT_CLIENT_ID)
    client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message

    # Connection initiation to MQTT Broker
    print(f"Connecting to MQTT broker at {MQTT_BROKER}...")
    client.connect(MQTT_BROKER, MQTT_PORT)
    client.loop_start() # Holds MQTT connection until manually disconnected

    # Start background threads for Gate Status, Day/Night Cycle and Chicken Detector
    # All key functions NEED to happen at the same time for program to work! Don't delete threads!
    threading.Thread(target=status_gate, args=(button_pin, motor_pin), daemon=True).start()
    threading.Thread(target=day_night_cycle, args=(bh1750_bus, bh1750_addr, pwm_pin, threshold_lux), daemon=True).start()
    threading.Thread(target=chicken_detector, daemon=True).start()

    try:
        while True:
            print("------------------------\n")
            
            # Convert gate status into JSON format
            with gate_status_lock:
                mqtt_gate_status = f"field1={str(gate_status)}&status=MQTTPUBLISH"

            # Measure lux from BH1750
            lux = bh1750_values(bh1750_bus, bh1750_addr)
            print(f"Current Lux : {lux:.2f}\nThreshold Lux : {threshold_lux}\n")

            # Convert lux data into JSON format
            mqtt_bh1750_data = f"field2={str(lux)}&status=MQTTPUBLISH"
            mqtt_bh1750_threshold = f"field3={str(threshold_lux)}&status=MQTTPUBLISH"
            
            # Convert predicted label & access detection from AI into JSON format
            with access_detection_lock:
                mqtt_access_detection = f"field4={str(access_detection)}&status=MQTTPUBLISH - Latest Animal Detected : {str(access_detection)} - {str(latest_animal)}"

            # Publish data to ThingsSpeak
            try:
                client.publish(topic=MQTT_TOPIC,
                               payload=mqtt_gate_status + "&" + mqtt_bh1750_data
                               + "&" + mqtt_bh1750_threshold + "&" + mqtt_access_detection, qos=0, retain=False)
                time.sleep(15) # 15 second interval per data upload
            except OSError:
                client.reconnect()
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
    # After program ends : PWM off, gate closed, MQTT disconnected & most recent capture frame deleted
        print("Shutting down: closing gate and turning off PWM.")
        with gate_status_lock:
            if gate_status:
                control_gate(step_seq, 120, motor_pin, reverse=False)
                gate_status = 0     
        capture_path = './capture.jpg'
        if os.path.exists(capture_path):
            os.remove(capture_path)
            print(f"Deleted {capture_path}")
        wp.softPwmWrite(pwm_pin, 0)
        client.on_disconnect
        client.loop_stop()
        print("MQTT disconnected.")

if __name__ == "__main__":
    main()
