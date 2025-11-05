import RPi.GPIO as GPIO
import time
from time import sleep

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# ==========================
# Motor Pin Mapping (4 independent)
# ==========================
# Right Driver
RF_IN1, RF_IN2, RF_EN = 17, 27, 18   # Right Front
RR_IN1, RR_IN2, RR_EN = 22, 23, 24   # Right Rear

# Left Driver
LF_IN1, LF_IN2, LF_EN = 5, 6, 12     # Left Front
LR_IN1, LR_IN2, LR_EN = 13, 19, 26   # Left Rear

motor_pins = [RF_IN1, RF_IN2, RR_IN1, RR_IN2, LF_IN1, LF_IN2, LR_IN1, LR_IN2,
              RF_EN, RR_EN, LF_EN, LR_EN]

for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)

# Setup PWM (independent enable pins)
FREQ = 100
pwm_rf = GPIO.PWM(RF_EN, FREQ)
pwm_rr = GPIO.PWM(RR_EN, FREQ)
pwm_lf = GPIO.PWM(LF_EN, FREQ)
pwm_lr = GPIO.PWM(LR_EN, FREQ)

# Start all motors at 40% duty cycle
for pwm in [pwm_rf, pwm_rr, pwm_lf, pwm_lr]:
    pwm.start(40)

# Motor dictionary for easier handling
MOTORS = {
    'rf': (RF_IN1, RF_IN2, pwm_rf),
    'rr': (RR_IN1, RR_IN2, pwm_rr),
    'lf': (LF_IN1, LF_IN2, pwm_lf),
    'lr': (LR_IN1, LR_IN2, pwm_lr)
}

# ==========================
# Ultrasonic Pins
# ==========================
TRIG_FRONT, ECHO_FRONT = 23, 24
TRIG_LEFT,  ECHO_LEFT  = 20, 21
TRIG_RIGHT, ECHO_RIGHT = 16, 25  # (adjusted right sensor pins)

for trig in [TRIG_FRONT, TRIG_LEFT, TRIG_RIGHT]:
    GPIO.setup(trig, GPIO.OUT)
    GPIO.output(trig, False)

for echo in [ECHO_FRONT, ECHO_LEFT, ECHO_RIGHT]:
    GPIO.setup(echo, GPIO.IN)

# ==========================
# Motion Functions
# ==========================
TURN_TIME = 0.7
LANE_STEP = 1.0   # seconds to move one lane (≈30cm)
SAFE_DIST = 25

def motor_forward(motor_id):
    in1, in2, _ = MOTORS[motor_id]
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)

def motor_backward(motor_id):
    in1, in2, _ = MOTORS[motor_id]
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)

def motor_stop(motor_id):
    in1, in2, _ = MOTORS[motor_id]
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)

def all_forward():
    for m in MOTORS:
        motor_forward(m)

def all_backward():
    for m in MOTORS:
        motor_backward(m)

def stop_all():
    for m in MOTORS:
        motor_stop(m)

def turn_left_90():
    # left wheels backward, right wheels forward
    motor_backward('lf')
    motor_backward('lr')
    motor_forward('rf')
    motor_forward('rr')
    sleep(TURN_TIME)
    stop_all()

def turn_right_90():
    # right wheels backward, left wheels forward
    motor_forward('lf')
    motor_forward('lr')
    motor_backward('rf')
    motor_backward('rr')
    sleep(TURN_TIME)
    stop_all()

# ==========================
# Ultrasonic Function
# ==========================
def get_distance(TRIG, ECHO):
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(ECHO) == 0:
        start_time = time.time()

    while GPIO.input(ECHO) == 1:
        stop_time = time.time()

    elapsed = stop_time - start_time
    distance = (elapsed * 34300) / 2
    return distance

# ==========================
# Lawn Mowing Logic
# ==========================
direction = "right"

try:
    print("Lawn mower pattern started... Press CTRL+C to stop")

    while True:
        front = get_distance(TRIG_FRONT, ECHO_FRONT)
        print(f"Front Distance: {front:.1f} cm | Direction: {direction}")

        if front > SAFE_DIST:
            all_forward()
        else:
            stop_all()
            sleep(1)
            print("Obstacle detected. Changing lane...")

            if direction == "right":
                # turn right, move one lane, turn right again
                turn_right_90()
                sleep(1)
                all_forward()
                sleep(LANE_STEP)
                stop_all()
                sleep(1)
                turn_right_90()
                sleep(1)
                direction = "left"
            else:
                # turn left, move one lane, turn left again
                turn_left_90()
                sleep(1)
                all_forward()
                sleep(LANE_STEP)
                stop_all()
                sleep(1)
                turn_left_90()
                sleep(1)
                direction = "right"

        sleep(0.1)

except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    stop_all()
    for _, _, pwm in MOTORS.values():
        pwm.stop()
    GPIO.cleanup()
    print("GPIO cleaned up. Program ended.")
