import json
import math
import heapq

from controller import Robot

# ============================================================
# CONFIGURATION
# ============================================================

robot = Robot()

TIME_STEP = int(robot.getBasicTimeStep())

# ============================================================
# NAVIGATION GRID CONFIGURATION
# ============================================================

GRID_MIN_X = -12.0
GRID_MAX_X = 12.0

GRID_MIN_Y = -12.0
GRID_MAX_Y = 12.0

GRID_RESOLUTION = 0.5

GRID_WIDTH = (
    int((GRID_MAX_X - GRID_MIN_X) / GRID_RESOLUTION) + 1
)

GRID_HEIGHT = (
    int((GRID_MAX_Y - GRID_MIN_Y) / GRID_RESOLUTION) + 1
)


# Supervisor R2 requirement.
MIN_CENTER_DISTANCE_M = 1.0

# Extra allowance for physical path-following error.
TRACKING_MARGIN_M = 0.40

# Required clearance for the planned path.
PLANNING_CLEARANCE_M = (
    MIN_CENTER_DISTANCE_M
    +
    TRACKING_MARGIN_M
)

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
# COORDINATE CONVERSION
# ============================================================

def world_to_grid(x, y):
    """
    Convert Webots world coordinates into integer grid
    coordinates.

    Returns:
        (grid_x, grid_y)
    """

    grid_x = round(
        (x - GRID_MIN_X) / GRID_RESOLUTION
    )

    grid_y = round(
        (y - GRID_MIN_Y) / GRID_RESOLUTION
    )

    return grid_x, grid_y


def grid_to_world(grid_x, grid_y):
    """
    Convert integer grid coordinates back into Webots
    world coordinates.

    Returns:
        (x, y)
    """

    x = GRID_MIN_X + grid_x * GRID_RESOLUTION
    y = GRID_MIN_Y + grid_y * GRID_RESOLUTION

    return x, y


def path_to_world_waypoints(path):
    """
    Converts coordinates on A* controlled path to 
    Weebots world coordinates.

    Returns:
        (x,y)
    """

    waypoints = []

    for grid_x, grid_y in path:
        world_x, world_y = grid_to_world(grid_x, grid_y)

        waypoints.append( (world_x, world_y) )

    return waypoints

# ============================================================
# OCCUPANCY GRID
# ============================================================

def create_empty_grid():

    return [
        [0 for _ in range(GRID_WIDTH)]
        for _ in range(GRID_HEIGHT)
    ]

def add_obstacle_to_grid(
    grid,
    rock_x,
    rock_y,
    radius_m
):

    rock_grid_x, rock_grid_y = world_to_grid(
        rock_x,
        rock_y
    )

    radius_cells = math.ceil(
        radius_m / GRID_RESOLUTION
    )

    for dy in range(
        -radius_cells,
        radius_cells + 1
    ):

        for dx in range(
            -radius_cells,
            radius_cells + 1
        ):

            grid_x = rock_grid_x + dx
            grid_y = rock_grid_y + dy


            # Make sure we're still inside the map.
            if not (
                0 <= grid_x < GRID_WIDTH
                and
                0 <= grid_y < GRID_HEIGHT
            ):
                continue


            # Convert the candidate cell back into
            # world coordinates.
            cell_x, cell_y = grid_to_world(
                grid_x,
                grid_y
            )


            distance_from_rock = math.hypot(
                cell_x - rock_x,
                cell_y - rock_y
            )


            if distance_from_rock <= radius_m:
                grid[grid_y][grid_x] = 1

def build_occupancy_grid(rocks):

    grid = create_empty_grid()

    for rock_name, rock_position in rocks.items():

        rock_x = rock_position[0]
        rock_y = rock_position[1]

        add_obstacle_to_grid(
            grid,
            rock_x,
            rock_y,
            PLANNING_CLEARANCE_M
        )

    return grid

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
# GRID HELPERS
# ============================================================

def is_in_bounds(cell):
    x, y = cell

    return (
        0 <= x < GRID_WIDTH 
        and 
        0 <= y < GRID_HEIGHT
    )

def is_free(grid, cell):
    x, y = cell

    return grid[y][x] == 0


def get_neighbors(grid, cell):

    x, y = cell

    directions = [
        (-1,  0),
        ( 1,  0),
        ( 0, -1),
        ( 0,  1),

        (-1, -1),
        (-1,  1),
        ( 1, -1),
        ( 1,  1),
    ]

    neighbors = []

    for dx, dy in directions:

        next_cell = (
            x + dx,
            y + dy
        )

        if not is_in_bounds(next_cell):
            continue

        if not is_free(grid, next_cell):
            continue

        # Prevents diagonal "corner cutting"
        if dx != 0 and dy != 0:

            horizontal_cell = (
                x + dx,
                y
            )

            vertical_cell = (
                x,
                y + dy
            )

            if (
                not is_free(grid, horizontal_cell)
                or
                not is_free(grid, vertical_cell)
            ):
                continue
        
        neighbors.append(next_cell)

    return neighbors

def movement_cost(current, neighbor):

    current_x, current_y = current
    neighbor_x, neighbor_y = neighbor

    dx = abs(neighbor_x - current_x)
    dy = abs(neighbor_y - current_y)

    if dx == 1 and dy == 1:
        return math.sqrt(2)

    return 1.0

def heuristic(cell, goal):

    x1, y1 = cell
    x2, y2 = goal

    return math.hypot(x2-x1, y2 - y1)

# ============================================================
# A* PATH PLANNER
# ============================================================

def astar(grid, start, goal):

    # --------------------------------------------------------
    # Validate start and goal
    # --------------------------------------------------------
    if not is_in_bounds(start):
        raise ValueError(
            f"Start cell {start} is outside the grid."
        )

    if not is_in_bounds(goal):
        raise ValueError(
            f"Goal cell {goal} is outside the grid."
        )

    if not is_free(grid, start):
        raise ValueError(
            f"Start cell {start} is occupied."
        )

    if not is_free(grid, goal):
        raise ValueError(
            f"Goal cell {goal} is occupied."
        )

    # --------------------------------------------------------
    # A* data structures
    # --------------------------------------------------------

    # list of (f_score, cell)
    open_set = []

    heapq.heappush(open_set, (heuristic(start, goal), start))

    # dictionary history of visited cells
    came_from = {}

    g_score = {
        start: 0.0
    }

    # --------------------------------------------------------
    # Search
    # --------------------------------------------------------
    while open_set:

        _, current = heapq.heappop( # disregard f_score (needs to be recalculated)
            open_set
        )

        # Check if we reached the goal
        if current == goal:

            path = [current]

            while current in came_from:

                current = came_from[current]
                path.append(current)

            path.reverse()

            return path


        for neighbor in get_neighbors(grid, current):
            
            tentative_g_score = (g_score[current] + movement_cost(current, neighbor))

            if tentative_g_score < g_score.get(neighbor, float("inf")):

                came_from[neighbor] = current
                g_score[neighbor] = (tentative_g_score)
                f_score = (tentative_g_score + heuristic(neighbor, goal))
                heapq.heappush(open_set, (f_score, neighbor))


    # If no route exists
    return None


def simplify_path(path):

    if len(path) <= 2:
        return path

    simplified = [path[0]]

    previous_dx = None
    previous_dy = None

    for i in range(1, len(path)):

        current = path[i - 1]
        next_cell = path[i]

        dx = next_cell[0] - current[0]
        dy = next_cell[1] - current[1]

        if (
            previous_dx is not None
            and
            (dx != previous_dx or dy != previous_dy)
        ):
            simplified.append(current)

        previous_dx = dx
        previous_dy = dy

    simplified.append(path[-1])

    return simplified

# ============================================================
# WAYPOINT FOLLOWING CONFIGURATION
# ============================================================

DRIVE_SPEED = 0.35
TURN_SPEED = 0.25

WAYPOINT_TOLERANCE_M = 0.30
FINAL_GOAL_TOLERANCE_M = 0.75

# Turn in place only when the rover is substantially
# misaligned with the target.
TURN_START_THRESHOLD = math.radians(90)

# Once pivoting has started, continue until the rover
# is reasonably aligned. The gap between 90 and 60
# prevents rapid DRIVE/TURN switching.
TURN_STOP_THRESHOLD = math.radians(60)

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
STUCK_WARNING_TIME_S = 10.0

tracked_waypoint_index = None
best_waypoint_distance = float("inf")
last_progress_time = robot.getTime()

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

    current_time = latest_world_state[
        "time"
    ]

    # --------------------------------------------------------
    # INITIALIZE MAP
    # --------------------------------------------------------
    if not map_initialized:

        occupancy_grid = build_occupancy_grid(
            rocks
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

        if tracked_waypoint_index != current_waypoint_index:

            tracked_waypoint_index = current_waypoint_index
            best_waypoint_distance = waypoint_distance
            last_progress_time = current_time

        elif (
            waypoint_distance
            <
            best_waypoint_distance - PROGRESS_EPSILON_M
        ):

            best_waypoint_distance = waypoint_distance
            last_progress_time = current_time

        elif (
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

            # Prevent warning spam.
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
