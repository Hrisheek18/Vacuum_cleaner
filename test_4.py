import RPi.GPIO as GPIO
from time import sleep

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# === Pin Mapping ===
# Right Driver
RF_IN1, RF_IN2, RF_EN = 17, 27, 18   # Right Front motor
RR_IN1, RR_IN2, RR_EN = 22, 23, 24   # Right Rear motor

# Left Driver
LF_IN1, LF_IN2, LF_EN = 5, 6, 12     # Left Front motor
LR_IN1, LR_IN2, LR_EN = 13, 19, 26   # Left Rear motor

# List all pins
pins = [RF_IN1, RF_IN2, RR_IN1, RR_IN2, LF_IN1, LF_IN2, LR_IN1, LR_IN2,
        RF_EN, RR_EN, LF_EN, LR_EN]

# Setup all GPIO pins
for p in pins:
    GPIO.setup(p, GPIO.OUT)

# Setup PWM for each motor (independent speed control)
FREQ = 100  # Hz
pwm_rf = GPIO.PWM(RF_EN, FREQ)
pwm_rr = GPIO.PWM(RR_EN, FREQ)
pwm_lf = GPIO.PWM(LF_EN, FREQ)
pwm_lr = GPIO.PWM(LR_EN, FREQ)

# Start all PWM with 75% speed
pwm_rf.start(75)
pwm_rr.start(75)
pwm_lf.start(75)
pwm_lr.start(75)

# Motor dictionary for easy control
MOTORS = {
    'rf': (RF_IN1, RF_IN2, pwm_rf),
    'rr': (RR_IN1, RR_IN2, pwm_rr),
    'lf': (LF_IN1, LF_IN2, pwm_lf),
    'lr': (LR_IN1, LR_IN2, pwm_lr)
}

# === Motor control functions ===
def motor_forward(motor_id):
    a, b, _ = MOTORS[motor_id]
    GPIO.output(a, GPIO.HIGH)
    GPIO.output(b, GPIO.LOW)

def motor_backward(motor_id):
    a, b, _ = MOTORS[motor_id]
    GPIO.output(a, GPIO.LOW)
    GPIO.output(b, GPIO.HIGH)

def motor_stop(motor_id):
    a, b, _ = MOTORS[motor_id]
    GPIO.output(a, GPIO.LOW)
    GPIO.output(b, GPIO.LOW)

def motor_speed(motor_id, speed):
    _, _, pwm = MOTORS[motor_id]
    pwm.ChangeDutyCycle(max(0, min(100, speed)))

def stop_all():
    for m in MOTORS:
        motor_stop(m)

# === Basic bot control ===
def forward():
    for m in MOTORS:
        motor_forward(m)
    print("Moving Forward")

def backward():
    for m in MOTORS:
        motor_backward(m)
    print("Moving Backward")

def left_turn():
    motor_backward('lf')
    motor_backward('lr')
    motor_forward('rf')
    motor_forward('rr')
    print("Turning Left")

def right_turn():
    motor_forward('lf')
    motor_forward('lr')
    motor_backward('rf')
    motor_backward('rr')
    print("Turning Right")

# === Main loop ===
print("""
Control Bot with:
  w = Forward
  s = Backward
  a = Left
  d = Right
  c = Stop
  q = Quit

Individual wheel test:
  rf, rr, lf, lr = forward
  rfb, rrb, lfb, lrb = backward
  rf0, rr0, lf0, lr0 = stop

Speed control:
  rf+ / rf- , rr+ / rr-, lf+ / lf-, lr+ / lr-
""")

# Current speeds
speed = {'rf': 75, 'rr': 75, 'lf': 75, 'lr': 75}

try:
    while True:
        cmd = input("Enter command: ").lower().strip()

        if cmd == 'w':
            forward()
        elif cmd == 's':
            backward()
        elif cmd == 'a':
            left_turn()
        elif cmd == 'd':
            right_turn()
        elif cmd == 'c':
            stop_all()
            print("Stop All")
        elif cmd == 'q':
            print("Exiting...")
            break

        # --- Individual motor commands ---
        elif cmd in ['rf', 'rr', 'lf', 'lr']:
            motor_forward(cmd)
            print(f"{cmd} forward")
        elif cmd in ['rfb', 'rrb', 'lfb', 'lrb']:
            motor_backward(cmd[:2])
            print(f"{cmd[:2]} backward")
        elif cmd in ['rf0', 'rr0', 'lf0', 'lr0']:
            motor_stop(cmd[:2])
            print(f"{cmd[:2]} stopped")

        # --- Individual motor speed adjustments ---
        elif len(cmd) == 3 and cmd[:2] in speed and cmd[2] in ['+', '-']:
            m = cmd[:2]
            if cmd[2] == '+':
                speed[m] = min(100, speed[m] + 5)
            else:
                speed[m] = max(0, speed[m] - 5)
            motor_speed(m, speed[m])
            print(f"{m} speed set to {speed[m]}%")

        else:
            print("Invalid command!")

except KeyboardInterrupt:
    print("\nKeyboard Interrupt. Exiting...")

finally:
    stop_all()
    for _, _, pwm in MOTORS.values():
        pwm.stop()
    GPIO.cleanup()
    print("GPIO Cleaned up. Program Ended.")
