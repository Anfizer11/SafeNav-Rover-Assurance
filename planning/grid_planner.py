import math
import heapq


# ============================================================
# GRID CONFIGURATION
# ============================================================

GRID_MIN_X = -12.0
GRID_MAX_X = 12.0

GRID_MIN_Y = -12.0
GRID_MAX_Y = 12.0

GRID_RESOLUTION = 0.5


GRID_WIDTH = (
    int(
        (GRID_MAX_X - GRID_MIN_X)
        / GRID_RESOLUTION
    )
    + 1
)

GRID_HEIGHT = (
    int(
        (GRID_MAX_Y - GRID_MIN_Y)
        / GRID_RESOLUTION
    )
    + 1
)


# ============================================================
# SAFETY / OBSTACLE GEOMETRY
# ============================================================

# Supervisor R2 requirement:
# rover center must remain at least this far from
# each rock center.
MIN_CENTER_DISTANCE_M = 1.0


# Additional allowance for path-following error.
TRACKING_MARGIN_M = 0.40


# Center-distance-based planning envelope.
CENTER_CLEARANCE_PLANNING_RADIUS_M = (
    MIN_CENTER_DISTANCE_M
    +
    TRACKING_MARGIN_M
)


# Approximate maximum horizontal radius of the
# Webots regular Rock PROTO at scale = 1.
ROCK_BASE_RADIUS_M = 0.084


# Conservative circular approximation of the
# Webots Sojourner collision footprint.
ROVER_EFFECTIVE_RADIUS_M = 0.40


# Small extra physical collision buffer.
PHYSICAL_SAFETY_MARGIN_M = 0.05


# ============================================================
# COORDINATE CONVERSION
# ============================================================

def world_to_grid(x, y):

    grid_x = round(
        (x - GRID_MIN_X)
        /
        GRID_RESOLUTION
    )

    grid_y = round(
        (y - GRID_MIN_Y)
        /
        GRID_RESOLUTION
    )

    return grid_x, grid_y


def grid_to_world(grid_x, grid_y):

    x = (
        GRID_MIN_X
        +
        grid_x * GRID_RESOLUTION
    )

    y = (
        GRID_MIN_Y
        +
        grid_y * GRID_RESOLUTION
    )

    return x, y


def path_to_world_waypoints(path):

    waypoints = []

    for grid_x, grid_y in path:

        world_x, world_y = grid_to_world(
            grid_x,
            grid_y
        )

        waypoints.append(
            (
                world_x,
                world_y
            )
        )

    return waypoints


# ============================================================
# OCCUPANCY GRID
# ============================================================

def create_empty_grid():

    return [
        [
            0
            for _ in range(GRID_WIDTH)
        ]
        for _ in range(GRID_HEIGHT)
    ]


def calculate_rock_planning_radius(
    rock_scale
):

    # Physical radius of this particular rock.
    rock_radius_m = (
        ROCK_BASE_RADIUS_M
        *
        rock_scale
    )

    # Existing R2 + tracking constraint.
    center_clearance_radius_m = (
        CENTER_CLEARANCE_PLANNING_RADIUS_M
    )

    # Actual physical non-collision constraint.
    physical_collision_radius_m = (
        rock_radius_m
        +
        ROVER_EFFECTIVE_RADIUS_M
        +
        PHYSICAL_SAFETY_MARGIN_M
    )

    # Both constraints must be satisfied.
    planning_radius_m = max(
        center_clearance_radius_m,
        physical_collision_radius_m
    )

    return {
        "rock_radius_m": rock_radius_m,

        "center_clearance_radius_m":
            center_clearance_radius_m,

        "physical_collision_radius_m":
            physical_collision_radius_m,

        "planning_radius_m":
            planning_radius_m,
    }


def add_obstacle_to_grid(
    grid,
    rock_x,
    rock_y,
    radius_m
):

    rock_grid_x, rock_grid_y = (
        world_to_grid(
            rock_x,
            rock_y
        )
    )

    radius_cells = math.ceil(
        radius_m
        /
        GRID_RESOLUTION
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


            if not (
                0 <= grid_x < GRID_WIDTH
                and
                0 <= grid_y < GRID_HEIGHT
            ):
                continue


            cell_x, cell_y = (
                grid_to_world(
                    grid_x,
                    grid_y
                )
            )


            distance_from_rock = math.hypot(
                cell_x - rock_x,
                cell_y - rock_y
            )


            if distance_from_rock <= radius_m:

                grid[grid_y][grid_x] = 1


def build_occupancy_grid(
    rocks,
    rock_scales,
    debug=False
):

    grid = create_empty_grid()


    for rock_name, rock_position in rocks.items():

        rock_x = rock_position[0]
        rock_y = rock_position[1]

        rock_scale = (
            rock_scales[rock_name]
        )


        envelope = (
            calculate_rock_planning_radius(
                rock_scale
            )
        )


        if debug:

            print(
                f"{rock_name}: "
                f"scale={rock_scale:.1f}, "
                f"rock_radius="
                f"{envelope['rock_radius_m']:.3f} m, "
                f"center_envelope="
                f"{envelope['center_clearance_radius_m']:.3f} m, "
                f"physical_envelope="
                f"{envelope['physical_collision_radius_m']:.3f} m, "
                f"planning_radius="
                f"{envelope['planning_radius_m']:.3f} m"
            )


        add_obstacle_to_grid(
            grid,
            rock_x,
            rock_y,
            envelope[
                "planning_radius_m"
            ]
        )


    return grid


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


        if not is_in_bounds(
            next_cell
        ):
            continue


        if not is_free(
            grid,
            next_cell
        ):
            continue


        # Prevent diagonal corner cutting.
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
                not is_free(
                    grid,
                    horizontal_cell
                )
                or
                not is_free(
                    grid,
                    vertical_cell
                )
            ):
                continue


        neighbors.append(
            next_cell
        )


    return neighbors


# ============================================================
# A* HELPERS
# ============================================================

def movement_cost(
    current,
    neighbor
):

    current_x, current_y = current
    neighbor_x, neighbor_y = neighbor

    dx = abs(
        neighbor_x - current_x
    )

    dy = abs(
        neighbor_y - current_y
    )


    if dx == 1 and dy == 1:

        return math.sqrt(2)


    return 1.0


def heuristic(
    cell,
    goal
):

    x1, y1 = cell
    x2, y2 = goal

    return math.hypot(
        x2 - x1,
        y2 - y1
    )


# ============================================================
# A* PATH PLANNER
# ============================================================

def astar(
    grid,
    start,
    goal
):

    if not is_in_bounds(start):

        raise ValueError(
            f"Start cell {start} "
            f"is outside the grid."
        )


    if not is_in_bounds(goal):

        raise ValueError(
            f"Goal cell {goal} "
            f"is outside the grid."
        )


    if not is_free(grid, start):

        raise ValueError(
            f"Start cell {start} "
            f"is occupied."
        )


    if not is_free(grid, goal):

        raise ValueError(
            f"Goal cell {goal} "
            f"is occupied."
        )


    open_set = []

    heapq.heappush(
        open_set,
        (
            heuristic(
                start,
                goal
            ),
            start
        )
    )


    came_from = {}


    g_score = {
        start: 0.0
    }


    while open_set:

        _, current = (
            heapq.heappop(
                open_set
            )
        )


        if current == goal:

            path = [
                current
            ]


            while current in came_from:

                current = (
                    came_from[
                        current
                    ]
                )

                path.append(
                    current
                )


            path.reverse()

            return path


        for neighbor in get_neighbors(
            grid,
            current
        ):

            tentative_g_score = (
                g_score[current]
                +
                movement_cost(
                    current,
                    neighbor
                )
            )


            if (
                tentative_g_score
                <
                g_score.get(
                    neighbor,
                    float("inf")
                )
            ):

                came_from[
                    neighbor
                ] = current

                g_score[
                    neighbor
                ] = tentative_g_score


                f_score = (
                    tentative_g_score
                    +
                    heuristic(
                        neighbor,
                        goal
                    )
                )


                heapq.heappush(
                    open_set,
                    (
                        f_score,
                        neighbor
                    )
                )


    return None


# ============================================================
# PATH SIMPLIFICATION
# ============================================================

def simplify_path(path):

    if len(path) <= 2:
        return path


    simplified = [
        path[0]
    ]

    previous_dx = None
    previous_dy = None


    for i in range(
        1,
        len(path)
    ):

        current = path[i - 1]
        next_cell = path[i]

        dx = (
            next_cell[0]
            -
            current[0]
        )

        dy = (
            next_cell[1]
            -
            current[1]
        )


        if (
            previous_dx is not None
            and
            (
                dx != previous_dx
                or
                dy != previous_dy
            )
        ):

            simplified.append(
                current
            )


        previous_dx = dx
        previous_dy = dy


    simplified.append(
        path[-1]
    )

    return simplified