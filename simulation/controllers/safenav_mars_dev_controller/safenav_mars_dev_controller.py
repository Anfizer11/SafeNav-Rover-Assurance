from controller import Robot, Keyboard


# ============================================================
# CONFIGURATION
# ============================================================

TIME_STEP = 64

# Sojourner's wheel motors have maxVelocity = 0.6 rad/s.
DRIVE_SPEED = 0.45
TURN_SPEED = 0.30


# ============================================================
# ROBOT
# ============================================================

robot = Robot()

# enable camera
camera = robot.getDevice("nav_camera")
camera.enable(TIME_STEP)

# ============================================================
# KEYBOARD
# ============================================================

keyboard = robot.getKeyboard()
keyboard.enable(TIME_STEP)


# ============================================================
# WHEEL MOTORS
# ============================================================

left_wheel_names = [
    "FrontLeftWheel",
    "MiddleLeftWheel",
    "BackLeftWheel",
]

right_wheel_names = [
    "FrontRightWheel",
    "MiddleRightWheel",
    "BackRightWheel",
]


left_wheels = []
right_wheels = []


for name in left_wheel_names:
    motor = robot.getDevice(name)

    if motor is None:
        raise RuntimeError(f"Could not find motor: {name}")

    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)

    left_wheels.append(motor)


for name in right_wheel_names:
    motor = robot.getDevice(name)

    if motor is None:
        raise RuntimeError(f"Could not find motor: {name}")

    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)

    right_wheels.append(motor)


# ============================================================
# HELPER
# ============================================================

def set_wheel_speeds(left_speed, right_speed):
    for motor in left_wheels:
        motor.setVelocity(left_speed)

    for motor in right_wheels:
        motor.setVelocity(right_speed)


# ============================================================
# MAIN LOOP
# ============================================================

print("SafeNav manual rover control started.")
print("W = forward")
print("S = reverse")
print("A = turn left")
print("D = turn right")
print("SPACE = stop")


while robot.step(TIME_STEP) != -1:

    key = keyboard.getKey()

    left_speed = 0.0
    right_speed = 0.0


    # Forward
    if key == ord("W"):
        left_speed = DRIVE_SPEED
        right_speed = DRIVE_SPEED


    # Reverse
    elif key == ord("S"):
        left_speed = -DRIVE_SPEED
        right_speed = -DRIVE_SPEED


    # Turn left
    elif key == ord("A"):
        left_speed = -TURN_SPEED
        right_speed = TURN_SPEED


    # Turn right
    elif key == ord("D"):
        left_speed = TURN_SPEED
        right_speed = -TURN_SPEED


    # Stop
    elif key == ord(" "):
        left_speed = 0.0
        right_speed = 0.0


    set_wheel_speeds(left_speed, right_speed)