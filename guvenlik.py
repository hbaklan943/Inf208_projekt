import time
import RPi.GPIO as io

io.setmode(io.BCM)

pir_pin = 18       # PIR sensor
door_pin = 23      # Door sensor
led_pin = 17       # GPIO 17 (pin 11)

io.setup(pir_pin, io.IN)
io.setup(door_pin, io.IN, pull_up_down=io.PUD_UP)
io.setup(led_pin, io.OUT)

while True:
    if io.input(pir_pin):
        print("HAREKET ALARMI!")
        io.output(led_pin, True)   # Turn LED on
        time.sleep(1)              # Keep it on for 1 second
        io.output(led_pin, False)  # Turn LED off
    if io.input(door_pin):
        print("KAPI ALARMI!")
    time.sleep(0.5)
