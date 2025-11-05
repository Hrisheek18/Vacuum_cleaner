import RPi.GPIO as GPIO
from time import sleep

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# === Pin mapping ===
# Right driver (controls right-front and right-rear motors)
RF_IN1 = 17  # Right Front motor input1
RF_IN2 = 27  # Right Front motor input2
RR_IN1 = 22  # Right Rear  motor input1 (new)
RR_IN2 = 23  # Right Rear  motor input2 (new)
EN_A   = 18  # PWM enable for right driver (controls both right motors)

# Left driver (controls left-front and left-rear motors)
LF_IN1 = 5   # Left Front motor input1
LF_IN2 = 6   # Left Front motor input2
LR_IN1 = 24  # Left Rear  motor input1 (new)
LR_IN2 = 25  # Left Rear  motor input2 (new)
EN_B   = 13  # PWM enable for left driver (controls both left motors)

# Setup motor pins as outputs
pins = [RF_IN1, RF_IN2, RR_IN1, RR_IN2, LF_IN1, LF_IN2, LR_IN1, LR_IN2, EN_A, EN_B]
for p in pins:
    GPIO.setup(p, GPIO.OUT)

# Setup PWM on enable pins (controls driver speed for both motors on that driver)
FREQ = 100  # Hz
pwm_right = GPIO.PWM(EN_A, FREQ)
pwm_left  = GPIO.PWM(EN_B, FREQ)
INITIAL_DUTY = 75
pwm_right.start(INITIAL_DUTY)
pwm_left.start(INITIAL_DUTY)

# Convenience: map motor ids to their pins
MOTORS = {
    'rf': (RF_IN1, RF_IN2),  # right front
    'rr': (RR_IN1, RR_IN2),  # right rear
    'lf': (LF_IN1, LF_IN2),  # left front
    'lr': (LR_IN1, LR_IN2),  # left rear
}

# Helper functions
def motor_forward(motor_id):
    a, b = MOTORS[motor_id]
    GPIO.output(a, GPIO.HIGH)
    GPIO.output(b, GPIO.LOW)

def motor_backward(motor_id):
    a, b = MOTORS[motor_id]
    GPIO.output(a, GPIO.LOW)
    GPIO.output(b, GPIO.HIGH)

def motor_stop(motor_id):
    a, b = MOTORS[motor_id]
    GPIO.output(a, GPIO.LOW)
    GPIO.output(b, GPIO.LOW)

def all_forward():
    for m in MOTORS:
        motor_forward(m)

def all_backward():
    for m in MOTORS:
        motor_backward(m)

def stop_all():
    for m in MOTORS:
        motor_stop(m)

def set_speed_right(duty):
    # clamp 0..100
    duty = max(0, min(100, duty))
    pwm_right.ChangeDutyCycle(duty)
    return duty

def set_speed_left(duty):
    duty = max(0, min(100, duty))
    pwm_left.ChangeDutyCycle(duty)
    return duty

# Start with all motors stopped
stop_all()

# current speeds
speed_right = INITIAL_DUTY
speed_left  = INITIAL_DUTY

help_text = """
Control Bot with keys:
  w : forward (all)
  s : backward (all)
  a : turn left  (right motors forward, left motors backward)
  d : turn right (left motors forward, right motors backward)
  c : stop (all)
  q : quit

Single-wheel commands (two-letter):
  rf  : right-front forward
  rfb : right-front backward
  rr  : right-rear forward
  rrb : right-rear backward
  lf  : left-front forward
  lfb : left-front backward
  lr  : left-rear forward
  lrb : left-rear backward
  <id>0 : stop that wheel, e.g. rf0 stops right-front

Speed control (affects entire driver side):
  +r  : increase RIGHT driver speed by 5
  -r  : decrease RIGHT driver speed by 5
  +l  : increase LEFT  driver speed by 5
  -l  : decrease LEFT  driver speed by 5

Examples:
  rf   -> right-front forward
  rrb  -> right-rear backward
  +r   -> bump right side speed up
"""

print(help_text)

try:
    while True:
        cmd = input("Enter command: ").strip().lower()

        # Basic motions
        if cmd == 'w':
            all_forward()
            print("Forward (all)")
        elif cmd == 's':
            all_backward()
            print("Backward (all)")
        elif cmd == 'c':
            stop_all()
            print("Stop (all)")
        elif cmd == 'q':
            print("Exiting...")
            break
        elif cmd == 'd':
            # Turn right: left motors forward, right motors backward
            motor_forward('lf')
            motor_forward('lr')
            motor_backward('rf')
            motor_backward('rr')
            print("Turn right")
        elif cmd == 'a':
            # Turn left: right motors forward, left motors backward
            motor_forward('rf')
            motor_forward('rr')
            motor_backward('lf')
            motor_backward('lr')
            print("Turn left")

        # Single wheel controls:
        elif cmd in ('rf', 'rfb', 'rr', 'rrb', 'lf', 'lfb', 'lr', 'lrb'):
            mid = cmd[:2]  # 'rf','rr','lf','lr'
            if cmd.endswith('b'):
                motor_backward(mid)
                print(f"{mid} backward")
            else:
                motor_forward(mid)
                print(f"{mid} forward")

        elif len(cmd) == 3 and cmd.endswith('0') and cmd[:2] in MOTORS:
            motor_stop(cmd[:2])
            print(f"{cmd[:2]} stopped")

        # Speed adjustments for driver sides
        elif cmd == '+r':
            speed_right = set_speed_right(speed_right + 5)
            print(f"Right driver speed: {speed_right}%")
        elif cmd == '-r':
            speed_right = set_speed_right(speed_right - 5)
            print(f"Right driver speed: {speed_right}%")
        elif cmd == '+l':
            speed_left = set_speed_left(speed_left + 5)
            print(f"Left driver speed: {speed_left}%")
        elif cmd == '-l':
            speed_left = set_speed_left(speed_left - 5)
            print(f"Left driver speed: {speed_left}%")

        else:
            print("Invalid command! Type w/s/a/d/c/q or use two-letter wheel commands. See help above.")

except KeyboardInterrupt:
    print("\nKeyboard interrupt, exiting...")

finally:
    stop_all()
    pwm_right.stop()
    pwm_left.stop()
    GPIO.cleanup()
    print("GPIO cleaned up, program ended.")
