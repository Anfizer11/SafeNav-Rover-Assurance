import json
import math
import sys

from pathlib import Path

from controller import Robot

PROJECT_ROOT = (
    Path(__file__)
    .resolve()
    .parents[3]
)

if str(PROJECT_ROOT) not in sys.path:

    sys.path.insert(
        0,
        str(PROJECT_ROOT)
    )

from planning.grid_planner import (
    GRID_RESOLUTION,
    GRID_WIDTH,
    GRID_HEIGHT,

    world_to_grid,
    path_to_world_waypoints,

    build_occupancy_grid,

    is_free,

    movement_cost,

    astar,
    simplify_path,
)

# ============================================================
# CONFIGURATION
# ============================================================

robot = Robot()

TIME_STEP = int(robot.getBasicTimeStep())

# ============================================================
# ORACLE RECEIVER
# ============================================================

oracle_receiver = robot.getDevice("oracle_receiver")

if oracle_receiver is None:
    raise RuntimeError(
        'Could not find Receiver named "oracle_receiver".'
    )

oracle_receiver.enable(TIME_STEP)

# ============================================================
# OCCUPANCY GRID
# ============================================================

def print_grid(
    grid,
    start_cell,
    goal_cell,
    path=None
):

    if path is None:
        path = []

    path_cells = set(path)


    start_x, start_y = start_cell
    goal_x, goal_y = goal_cell

    print()
    print("========== OCCUPANCY GRID ==========")
    print()

    # Print high Y first so the console map resembles
    # a normal top-down Cartesian map.
    for grid_y in range(
        GRID_HEIGHT - 1,
        -1,
        -1
    ):

        row = ""

        for grid_x in range(GRID_WIDTH):

            if (
                grid_x == start_x
                and grid_y == start_y
            ):
                row += "S"

            elif (
                grid_x == goal_x
                and grid_y == goal_y
            ):
                row += "G"

            elif (grid_x, grid_y) in path_cells:
                row += "*"

            elif grid[grid_y][grid_x] == 1:
                row += "#"

            else:
                row += "."

        print(row)


map_initialized = False

waypoints = []
current_waypoint_index = 0

navigation_initialized = False
navigation_complete = False

# ============================================================
# WAYPOINT FOLLOWING CONFIGURATION
# ============================================================

DRIVE_SPEED = 0.35
TURN_SPEED = 0.25

WAYPOINT_TOLERANCE_M = 0.30
FINAL_GOAL_TOLERANCE_M = 0.75

# Turn in place only when the rover is substantially
# misaligned with the target.
TURN_START_THRESHOLD = math.radians(35)

# Once pivoting has started, continue until the rover
# is reasonably aligned. The gap between 90 and 60
# prevents rapid DRIVE/TURN switching.
TURN_STOP_THRESHOLD = math.radians(15)

STEERING_GAIN = 0.8
MAX_STEERING_CORRECTION = 0.20

# Set this to whichever direction your orientation test showed
# is Sojourner's forward local axis.
# Can be +X, -X, +Y, -Y
ROVER_FORWARD_AXIS = "+X"

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
        raise RuntimeError(
            f'Could not find motor "{name}".'
        )

    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)

    left_wheels.append(motor)


for name in right_wheel_names:

    motor = robot.getDevice(name)

    if motor is None:
        raise RuntimeError(
            f'Could not find motor "{name}".'
        )

    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)

    right_wheels.append(motor)


def set_wheel_speeds(left_speed, right_speed):

    for motor in left_wheels:
        motor.setVelocity(left_speed)

    for motor in right_wheels:
        motor.setVelocity(right_speed)


def get_rover_heading(orientation):
    """
        Gets the orientation of the rover(how it is turned).

        Return:
            Heading(in radians)
    """

    if ROVER_FORWARD_AXIS == "+X":
        forward_x = orientation[0]
        forward_y = orientation[3]

    elif ROVER_FORWARD_AXIS == "-X":
        forward_x = -orientation[0]
        forward_y = -orientation[3]

    elif ROVER_FORWARD_AXIS == "+Y":
        forward_x = orientation[1]
        forward_y = orientation[4]

    elif ROVER_FORWARD_AXIS == "-Y":
        forward_x = -orientation[1]
        forward_y = -orientation[4]

    else:
        raise ValueError(
            f"Unknown forward axis: {ROVER_FORWARD_AXIS}"
        )

    return math.atan2(
        forward_y,
        forward_x
    )

def normalize_angle(angle):
    """
        Normalizes angles to ensure that +179° - (-179°) 
        doesn't equate to -358° instead of the correct 
        +2°
    """

    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )

def clamp(value, minimum, maximum):

    return max(
        minimum,
        min(value, maximum)
    )

# ------------------------------------------------------------
# PROGRESS / STALL MONITORING
# ------------------------------------------------------------

PROGRESS_EPSILON_M = 0.02
HEADING_PROGRESS_EPSILON_RAD = math.radians(2.0)

STUCK_WARNING_TIME_S = 10.0

tracked_waypoint_index = None

best_waypoint_distance = float("inf")
best_heading_error = float("inf")

last_progress_time = robot.getTime()
last_control_mode = None

turning_in_place = False

# ============================================================
# MAIN LOOP
# ============================================================

next_navigation_debug_time = 0.0
last_oracle_receive_time = robot.getTime()

ORACLE_TIMEOUT_S = 0.5

while robot.step(TIME_STEP) != -1:

    latest_world_state = None

    # --------------------------------------------------------
    # RECEIVE LATEST WORLD STATE
    # --------------------------------------------------------

    while oracle_receiver.getQueueLength() > 0:

        raw_message = oracle_receiver.getString()

        latest_world_state = json.loads(raw_message)

        oracle_receiver.nextPacket()

    # --------------------------------------------------------
    # IF NO DATA HAS ARRIVED YET record last no data then WAIT
    # --------------------------------------------------------

    if latest_world_state is None:

        time_since_oracle = (
            robot.getTime()
            - last_oracle_receive_time
        )

        if time_since_oracle >= ORACLE_TIMEOUT_S:

            set_wheel_speeds(
                0.0,
                0.0
            )

        continue

    last_oracle_receive_time = robot.getTime()
    # --------------------------------------------------------
    # EXTRACT WORLD STATE
    # --------------------------------------------------------

    rover_position = latest_world_state[
        "rover_position"
    ]

    rover_orientation = latest_world_state[
        "rover_orientation"
    ]

    goal_position = latest_world_state[
        "goal_position"
    ]

    rocks = latest_world_state[
        "rocks"
    ]

    rock_scales = latest_world_state[
        "rock_scales"
    ]

    current_time = latest_world_state[
        "time"
    ]

    # --------------------------------------------------------
    # INITIALIZE MAP
    # --------------------------------------------------------
    if not map_initialized:

        print()
        print("========== ORACLE ROCK DATA ==========")

        for rock_name, rock_position in rocks.items():

            print(
                f"{rock_name}: "
                f"position={rock_position}, "
                f"scale={rock_scales[rock_name]}"
            )

        print()
        
        occupancy_grid = build_occupancy_grid(
            rocks,
            rock_scales,
            debug=True
        )

        start_cell = world_to_grid(
            rover_position[0],
            rover_position[1]
        )

        goal_cell = world_to_grid(
            goal_position[0],
            goal_position[1]
        )

        print(
            f"Start occupied: "
            f"{not is_free(occupancy_grid, start_cell)}"
        )

        print(
            f"Goal occupied: "
            f"{not is_free(occupancy_grid, goal_cell)}"
        )

        path = astar(
            occupancy_grid,
            start_cell,
            goal_cell
        )

        if path is None:

            print()
            print("ERROR: No path found to goal.")

        else:

            simplified_path = simplify_path(path)

            waypoints = path_to_world_waypoints(
                simplified_path
            )

            if waypoints:

                waypoints[-1] = (
                    goal_position[0],
                    goal_position[1]
                )

            # ----------------------------------------------------
            # INITIALIZE WAYPOINT FOLLOWING
            # ----------------------------------------------------

            if len(waypoints) > 1:

                # Waypoint 0 corresponds approximately to the
                # rover's current/start position, so skip it.
                current_waypoint_index = 1

                navigation_initialized = True

            else:

                print(
                    "ERROR: Not enough waypoints "
                    "to begin navigation."
                )


            print()
            print("========== WORLD WAYPOINTS ==========")

            for i, waypoint in enumerate(waypoints):
                print(
                    f"{i}: "
                    f"({waypoint[0]:.2f}, "
                    f"{waypoint[1]:.2f})"
                )
            
            print()
            print("========== A* PATH ==========")

            print(
                f"Path found with "
                f"{len(path)} cells."
            )


            path_length_grid = 0.0

            for i in range(
                len(path) - 1
            ):

                path_length_grid += movement_cost(
                    path[i],
                    path[i + 1]
                )


            path_length_m = (
                path_length_grid
                *
                GRID_RESOLUTION
            )


            print(
                f"Approximate path length: "
                f"{path_length_m:.2f} m"
            )

        print_grid(
            occupancy_grid,
            start_cell,
            goal_cell,
            path
        )

        map_initialized = True


    # ========================================================
    # WAYPOINT FOLLOWING
    # ========================================================

    if (
        navigation_initialized
        and not navigation_complete
    ):

        # ----------------------------------------------------
        # CURRENT TARGET WAYPOINT
        # ----------------------------------------------------

        target_x, target_y = waypoints[
            current_waypoint_index
        ]

        rover_x = rover_position[0]
        rover_y = rover_position[1]


        # ----------------------------------------------------
        # VECTOR FROM ROVER TO WAYPOINT
        # ----------------------------------------------------

        dx = target_x - rover_x
        dy = target_y - rover_y


        # ----------------------------------------------------
        # DISTANCE TO WAYPOINT
        # ----------------------------------------------------

        waypoint_distance = math.hypot(
            dx,
            dy
        )

        # ----------------------------------------------------
        # IS THIS THE FINAL WAYPOINT?
        # ----------------------------------------------------

        is_final_waypoint = (
            current_waypoint_index
            ==
            len(waypoints) - 1
        )


        # Use a slightly different tolerance for
        # the actual goal.
        if is_final_waypoint:
            tolerance = FINAL_GOAL_TOLERANCE_M
        else:
            tolerance = WAYPOINT_TOLERANCE_M


        # ----------------------------------------------------
        # CHECK WHETHER WAYPOINT WAS REACHED
        # ----------------------------------------------------

        if waypoint_distance <= tolerance:

            if is_final_waypoint:

                set_wheel_speeds(
                    0.0,
                    0.0
                )

                navigation_complete = True

                print()
                print(
                    "========== NAVIGATION COMPLETE =========="
                )

                print(
                    f"Final goal distance: "
                    f"{waypoint_distance:.2f} m"
                )

                continue


            else:

                print(
                    f"Reached waypoint "
                    f"{current_waypoint_index}: "
                    f"({target_x:.2f}, "
                    f"{target_y:.2f})"
                    f"at t={current_time:.2f}s"
                )

                current_waypoint_index += 1


                # Update target to the next waypoint.
                target_x, target_y = waypoints[
                    current_waypoint_index
                ]

                dx = target_x - rover_x
                dy = target_y - rover_y

                # Recalculate distance for the NEW waypoint.
                waypoint_distance = math.hypot(
                    dx,
                    dy
                )


        # ----------------------------------------------------
        # TARGET HEADING
        # ----------------------------------------------------

        target_heading = math.atan2(
            dy,
            dx
        )


        # ----------------------------------------------------
        # CURRENT ROVER HEADING
        # ----------------------------------------------------

        rover_heading = get_rover_heading(
            rover_orientation
        )


        # ----------------------------------------------------
        # HEADING ERROR
        # ----------------------------------------------------

        heading_error = normalize_angle(
            target_heading - rover_heading
        )


        # ----------------------------------------------------
        # TURN-IN-PLACE MODE SELECTION
        # ----------------------------------------------------

        if turning_in_place:

            # Once pivoting has started, keep pivoting until
            # the rover is substantially better aligned.
            if abs(heading_error) <= TURN_STOP_THRESHOLD:
                turning_in_place = False

        else:

            # Do not begin a pivot unless the rover is very
            # badly misaligned with the target.
            if abs(heading_error) >= TURN_START_THRESHOLD:
                turning_in_place = True


        # ----------------------------------------------------
        # TURN IN PLACE
        # ----------------------------------------------------

        if turning_in_place:

            if heading_error > 0:

                control_mode = "TURN_LEFT"

                set_wheel_speeds(
                    TURN_SPEED,
                    -TURN_SPEED
                )

            else:

                control_mode = "TURN_RIGHT"

                set_wheel_speeds(
                    -TURN_SPEED,
                    TURN_SPEED
                )


        # ----------------------------------------------------
        # DRIVE FORWARD WHILE STEERING
        # ----------------------------------------------------

        else:

            control_mode = "DRIVE"

            steering_correction = (
                STEERING_GAIN
                *
                heading_error
            )

            steering_correction = clamp(
                steering_correction,
                -MAX_STEERING_CORRECTION,
                MAX_STEERING_CORRECTION
            )

            left_speed = (
                DRIVE_SPEED
                +
                steering_correction
            )

            right_speed = (
                DRIVE_SPEED
                -
                steering_correction
            )

            left_speed = clamp(
                left_speed,
                -0.55,
                0.55
            )

            right_speed = clamp(
                right_speed,
                -0.55,
                0.55
            )

            set_wheel_speeds(
                left_speed,
                right_speed
            )

        # ----------------------------------------------------
        # WAYPOINT PROGRESS / STALL MONITOR
        # ----------------------------------------------------

        absolute_heading_error = abs(heading_error)

        # ----------------------------------------------------
        # NEW WAYPOINT
        # ----------------------------------------------------

        if tracked_waypoint_index != current_waypoint_index:

            tracked_waypoint_index = current_waypoint_index

            best_waypoint_distance = waypoint_distance
            best_heading_error = absolute_heading_error

            last_progress_time = current_time
            last_control_mode = control_mode


        # ----------------------------------------------------
        # CONTROL MODE CHANGED
        # ----------------------------------------------------

        elif control_mode != last_control_mode:

            # Reset the appropriate progress reference whenever
            # the controller changes between DRIVE and TURN.
            best_waypoint_distance = waypoint_distance
            best_heading_error = absolute_heading_error

            last_progress_time = current_time
            last_control_mode = control_mode


        # ----------------------------------------------------
        # DRIVE PROGRESS
        # ----------------------------------------------------

        elif control_mode == "DRIVE":

            if (
                waypoint_distance
                <
                best_waypoint_distance - PROGRESS_EPSILON_M
            ):

                best_waypoint_distance = waypoint_distance
                last_progress_time = current_time


        # ----------------------------------------------------
        # TURNING PROGRESS
        # ----------------------------------------------------

        elif (
            control_mode == "TURN_LEFT"
            or
            control_mode == "TURN_RIGHT"
        ):

            if (
                absolute_heading_error
                <
                best_heading_error
                - HEADING_PROGRESS_EPSILON_RAD
            ):

                best_heading_error = absolute_heading_error
                last_progress_time = current_time


        # ----------------------------------------------------
        # STALL DETECTION
        # ----------------------------------------------------

        if (
            current_time - last_progress_time
            >=
            STUCK_WARNING_TIME_S
        ):

            print()
            print("WARNING: POSSIBLE NAVIGATION STALL")

            print(
                f"t = {current_time:.2f} s"
            )

            print(
                f"Waypoint = "
                f"{current_waypoint_index}/"
                f"{len(waypoints) - 1}"
            )

            print(
                f"Waypoint distance = "
                f"{waypoint_distance:.2f} m"
            )

            print(
                f"Rover position = "
                f"({rover_position[0]:.2f}, "
                f"{rover_position[1]:.2f}, "
                f"{rover_position[2]:.2f})"
            )

            print(
                f"Control mode = {control_mode}"
            )

            print(
                f"Rover heading = "
                f"{math.degrees(rover_heading):.1f} deg"
            )

            print(
                f"Target heading = "
                f"{math.degrees(target_heading):.1f} deg"
            )

            print(
                f"Heading error = "
                f"{math.degrees(heading_error):.1f} deg"
            )

            print()

            # Prevent repeated warnings every timestep.
            last_progress_time = current_time
       
        

        if current_time >= next_navigation_debug_time:

            print(
                f"NAV | "
                f"t={current_time:.2f}s | "
                f"Pos=("
                f"{rover_position[0]:.2f}, "
                f"{rover_position[1]:.2f}, "
                f"{rover_position[2]:.2f}) | "
                f"Mode={control_mode} | "
                f"WP={current_waypoint_index}/"
                f"{len(waypoints) - 1} | "
                f"Dist={waypoint_distance:.2f}m | "
                f"Heading="
                f"{math.degrees(rover_heading):.1f}deg | "
                f"Target="
                f"{math.degrees(target_heading):.1f}deg | "
                f"Error="
                f"{math.degrees(heading_error):.1f}deg"
            )

            next_navigation_debug_time += 0.5
