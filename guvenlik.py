import RPi.GPIO as GPIO
import time

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pin definitions
pir_pin = 18        # PIR sensor
led_pin = 17        # LED
TRIG = 23           # HC-SR04 TRIG
ECHO = 24           # HC-SR04 ECHO

# GPIO setup
GPIO.setup(pir_pin, GPIO.IN)
GPIO.setup(led_pin, GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

print("Sistem baslatildi...")

while True:
    # --- PIR Motion Detection ---
    if GPIO.input(pir_pin):
        print("HAREKET ALARMI!")
        GPIO.output(led_pin, True)
        time.sleep(1)
        GPIO.output(led_pin, False)

    # --- Distance Measurement ---
    GPIO.output(TRIG, False)
    time.sleep(0.1)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    pulse_start = time.time()
    pulse_end = time.time()

    # Wait for echo to go high
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    
    # Wait for echo to go low
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    if 2 < distance < 400:
        print("Mesafe:", distance - 0.5, "cm")
    else:
        print("Menzil asildi")

    time.sleep(0.5)
