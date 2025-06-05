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
servo_pin = 27      # Servo motor (GPIO 27)

# GPIO setup
GPIO.setup(pir_pin, GPIO.IN)
GPIO.setup(led_pin, GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(servo_pin, GPIO.OUT)

# Servo setup
servo = GPIO.PWM(servo_pin, 50)  # 50Hz frequency
servo.start(0)

print("Sistem baslatildi...")

# Lock state tracker
lock_open = False

def set_servo_angle(angle):
    duty = 2.5 + (angle / 180.0) * 10  # Map angle to duty cycle
    servo.ChangeDutyCycle(duty)
    time.sleep(0.3)
    servo.ChangeDutyCycle(0)

def unlock():
    print("Kilit ACILDI")
    set_servo_angle(90)  # open position

def lock():
    print("Kilit KAPANDI")
    set_servo_angle(0)   # locked position

try:
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

        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()

        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        distance = round(distance, 2)

        if 2 < distance < 400:
            print("Mesafe:", distance - 0.5, "cm")
            if distance < 30 and not lock_open:
                unlock()
                lock_open = True
            elif distance >= 30 and lock_open:
                lock()
                lock_open = False
        else:
            print("Menzil asildi")

        time.sleep(0.5)

except KeyboardInterrupt:
    print("Temizleniyor...")
    servo.stop()
    GPIO.cleanup()
