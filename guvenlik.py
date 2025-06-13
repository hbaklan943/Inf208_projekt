import RPi.GPIO as GPIO
import time

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pin definitions
PIR_PIN = 18        # PIR sensor
LED_PIN = 17        # LED
TRIG_PIN = 23       # HC-SR04 TRIG
ECHO_PIN = 24       # HC-SR04 ECHO
SERVO_PIN = 27      # Servo motor
BUZZER_PIN = 4      # Buzzer

# Security and distance thresholds
DISTANCE_THRESHOLD_CM = 30  # Distance threshold (cm) for door lock to open/close
MIN_DISTANCE_CM = 2         # Minimum distance (cm) the ultrasonic sensor can detect
MAX_DISTANCE_CM = 400       # Maximum distance (cm) the ultrasonic sensor can detect
ULTRASONIC_OFFSET = 0.5     # Calibration offset for the ultrasonic sensor
SOUND_SPEED = 17150         # Speed of sound in air (cm/s, 34300 / 2 for one way)

# Timing constants
PIR_DEBOUNCE_TIME = 1.0     # Wait time (s) after PIR detection
SERVO_MOVE_DELAY = 0.3      # Wait time (s) after servo movement
BUZZER_DURATION = 0.5       # Buzzer sound duration (s)
ULTRASONIC_PULSE_DELAY = 0.00001 # Ultrasonic trigger pulse duration (s)
ULTRASONIC_READ_DELAY = 0.1 # Wait time (s) between ultrasonic sensor readings

# GPIO setup
GPIO.setup(PIR_PIN, GPIO.IN)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

# Servo setup
servo = GPIO.PWM(SERVO_PIN, 50)  # 50Hz frequency
servo.start(0)

print("System wurde gestartet...")

# Lock state tracker
lock_open = False

def set_servo_angle(angle):
    duty = 2.5 + (angle / 180.0) * 10  # Map angle to duty cycle
    servo.ChangeDutyCycle(duty)
    time.sleep(SERVO_MOVE_DELAY)
    servo.ChangeDutyCycle(0)

def unlock():
    global lock_open
    print("Das Schloss ist geöffnet.")
    set_servo_angle(90)  # Open position
    GPIO.output(BUZZER_PIN, True)
    time.sleep(BUZZER_DURATION)
    GPIO.output(BUZZER_PIN, False)
    lock_open = True

def lock():
    global lock_open
    print("Das Schloss ist geschlossen.")
    set_servo_angle(0)  # Locked position
    lock_open = False

try:
    while True:
        # --- PIR Motion Detection ---
        if GPIO.input(PIR_PIN):
            print("Bewegung erkannt!")
            GPIO.output(LED_PIN, True)
            time.sleep(PIR_DEBOUNCE_TIME)
            GPIO.output(LED_PIN, False)

        # --- Distance Measurement ---
        GPIO.output(TRIG_PIN, False)
        time.sleep(ULTRASONIC_READ_DELAY)

        GPIO.output(TRIG_PIN, True)
        time.sleep(ULTRASONIC_PULSE_DELAY)
        GPIO.output(TRIG_PIN, False)

        pulse_start = time.time()
        pulse_end = time.time()

        while GPIO.input(ECHO_PIN) == 0:
            pulse_start = time.time()

        while GPIO.input(ECHO_PIN) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * SOUND_SPEED
        distance = round(distance, MIN_DISTANCE_CM)

        if MIN_DISTANCE_CM < distance < MAX_DISTANCE_CM:
            print("Distanz:", distance - ULTRASONIC_OFFSET, "cm")
            if distance < DISTANCE_THRESHOLD_CM and not lock_open:
                unlock()
            elif distance >= DISTANCE_THRESHOLD_CM and lock_open:
                lock()
        else:
            print("Außerhalb des Bereichs")
            if lock_open:
                lock()

        time.sleep(0.5)

except KeyboardInterrupt:
    print("Speicher wird bereinigt...")
    servo.stop()
    GPIO.cleanup()
