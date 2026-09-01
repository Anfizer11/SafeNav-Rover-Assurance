import csv
import math
from datetime import datetime
from pathlib import Path
import json

from controller import Supervisor


# ============================================================
# CONFIGURATION
# ============================================================

supervisor = Supervisor()

oracle_emitter = supervisor.getDevice("oracle_emitter")

if oracle_emitter is None:
    raise RuntimeError(
        'Could not find Emitter named "oracle_emitter".'
    )

TIME_STEP = int(supervisor.getBasicTimeStep())

LOG_INTERVAL = 0.5

# ============================================================
# GET WORLD OBJECTS
# ============================================================

rover = supervisor.getFromDef("ROVER")
goal = supervisor.getFromDef("GOAL")


rock_names = [
    "ROCK_01",
    "ROCK_02",
    "ROCK_03",
    "ROCK_04",
]

rocks = {}

for rock_name in rock_names:
    rock = supervisor.getFromDef(rock_name)

    if rock is None:
        raise RuntimeError(
            f'Could not find rock with DEF name "{rock_name}".'
        )

    rocks[rock_name] = rock

# ============================================================
# VALIDATE IMPORTANT OBJECTS
# ============================================================

if rover is None:
    raise RuntimeError(
        'Could not find rover. Make sure it has DEF name "ROVER".'
    )

if goal is None:
    raise RuntimeError(
        'Could not find goal. Make sure it has DEF name "GOAL".'
    )

# ============================================================
# COLLISION DETECTION
# ============================================================

CONTACT_TOLERANCE_M = 0.001  # 1 millimeter


def detect_rover_rock_collision():

    # All physical contact points involving the rover,
    # including its wheels and other descendant solids.
    rover_contacts = rover.getContactPoints(True)

    for rock_name, rock in rocks.items():

        # Physical contact points involving this rock.
        rock_contacts = rock.getContactPoints(True)

        for rover_contact in rover_contacts:

            rover_contact_position = rover_contact.getPoint()

            for rock_contact in rock_contacts:

                rock_contact_position = rock_contact.getPoint()

                contact_point_distance = math.dist(
                    rover_contact_position,
                    rock_contact_position
                )

                # If the rover and rock report essentially
                # the same world-space contact point,
                # they are physically touching.
                if contact_point_distance <= CONTACT_TOLERANCE_M:
                    return True, rock_name

    return False, None

# ============================================================
# LOGGING SETUP
# ============================================================

PROJECT_ROOT = Path(__file__).resolve().parents[3]

LOG_DIRECTORY = PROJECT_ROOT / "data" / "logs"

LOG_DIRECTORY.mkdir(
    parents=True,
    exist_ok=True
)

timestamp = datetime.now().strftime(
    "%Y-%m-%d_%H-%M-%S"
)

LOG_FILE = LOG_DIRECTORY / f"mission_{timestamp}.csv"

# ============================================================
# SCENARIO CONFIGURATION
# ============================================================ 

ACTIVE_SCENARIO = "scenario_001"

SCENARIO_FILE = (
    PROJECT_ROOT
    / "scenarios"
    / f"{ACTIVE_SCENARIO}.json"
)

# ============================================================
# LOAD AND APPLY SCENARIO
# ============================================================

if not SCENARIO_FILE.exists():
    raise FileNotFoundError(
        f"Could not find scenario file:\n"
        f"{SCENARIO_FILE}"
    )


with SCENARIO_FILE.open(
    "r",
    encoding="utf-8"
) as scenario_file:

    scenario = json.load(
        scenario_file
    )


scenario_id = scenario.get(
    "scenario_id",
    SCENARIO_FILE.stem
)


print()
print("========== LOADING SCENARIO ==========")
print(f"Scenario: {scenario_id}")
print(f"File: {SCENARIO_FILE}")


# ------------------------------------------------------------
# ROVER
# ------------------------------------------------------------

rover_translation_field = rover.getField(
    "translation"
)

rover_rotation_field = rover.getField(
    "rotation"
)


if rover_translation_field is None:
    raise RuntimeError(
        'ROVER does not have a "translation" field.'
    )

if rover_rotation_field is None:
    raise RuntimeError(
        'ROVER does not have a "rotation" field.'
    )


rover_translation_field.setSFVec3f(
    scenario["rover"]["position"]
)

rover_rotation_field.setSFRotation(
    scenario["rover"]["rotation"]
)


# Clear any velocity/angular velocity the rover
# may have had before being repositioned.
rover.resetPhysics()


# ------------------------------------------------------------
# GOAL
# ------------------------------------------------------------

goal_translation_field = goal.getField(
    "translation"
)


if goal_translation_field is None:
    raise RuntimeError(
        'GOAL does not have a "translation" field.'
    )


goal_translation_field.setSFVec3f(
    scenario["goal"]["position"]
)


# ------------------------------------------------------------
# ROCKS
# ------------------------------------------------------------

for rock_name, rock_config in scenario[
    "rocks"
].items():

    if rock_name not in rocks:
        raise RuntimeError(
            f'Scenario references "{rock_name}", '
            f"but that DEF does not exist in the world."
        )


    rock = rocks[rock_name]


    rock_translation_field = rock.getField(
        "translation"
    )


    if rock_translation_field is None:
        raise RuntimeError(
            f'{rock_name} does not have a '
            f'"translation" field.'
        )


    rock_translation_field.setSFVec3f(
        rock_config["position"]
    )


    # Rock.proto uses a scalar scale value.
    if "scale" in rock_config:

        rock_scale_field = rock.getField(
            "scale"
        )

        if rock_scale_field is None:
            raise RuntimeError(
                f'{rock_name} does not have a '
                f'"scale" field.'
            )


        rock_scale_field.setSFFloat(
            float(
                rock_config["scale"]
            )
        )


    rock.resetPhysics()


print("Scenario successfully applied.")
print()

# ============================================================
# OPEN CSV
# ============================================================

with LOG_FILE.open(
    "w",
    newline="",
    encoding="utf-8"
) as csv_file:

    writer = csv.writer(csv_file)

    # --------------------------------------------------------
    # CSV HEADER
    # --------------------------------------------------------

    writer.writerow([
        "time_s",

        "rover_x_m",
        "rover_y_m",
        "rover_z_m",

        "goal_x_m",
        "goal_y_m",
        "goal_z_m",

        "goal_distance_2d_m",

        "closest_rock",
        "closest_rock_distance_2d_m",
    ])


    print(
        f"SafeNav Supervisor logging to:\n"
        f"{LOG_FILE}"
    )


    # ============================================================
    # MISSION REQUIREMENTS
    # ============================================================

    GOAL_RADIUS_M = 0.75
    MIN_CENTER_DISTANCE_M = 1.0

    # R4 requirement:
    # The rover should complete the mission within this time
    MISSION_TIMEOUT_S = 700.0

    # Diagnostic limit:
    # Even if R4 is violated, allow the simulation to continue
    # so we can determine whether the rover eventually reaches
    # the goal
    SIMULATION_HARD_STOP_S = 1200.0

    # ============================================================
    # MISSION STATE
    # ============================================================

    minimum_rock_distance = float("inf")

    collision_occurred = False
    collision_rock = None

    clearance_violated = False

    goal_reached = False
    goal_reached_time = None

    timeout_violated = False

    mission_complete = False
    mission_end_reason = None

    # ========================================================
    # MAIN LOOP
    # ========================================================

    next_log_time = 0.0

    while supervisor.step(TIME_STEP) != -1:

        current_time = supervisor.getTime()

        # ----------------------------------------------------
        # GROUND-TRUTH POSITIONS
        # ----------------------------------------------------

        rover_position = rover.getPosition()
        goal_position = goal.getPosition()
        rover_orientation = rover.getOrientation()


        # ----------------------------------------------------
        # GOAL DISTANCE
        # ----------------------------------------------------

        rover_distance_from_goal_2d = math.hypot(
            rover_position[0] - goal_position[0],
            rover_position[1] - goal_position[1]
        )

        # ----------------------------------------------------
        # ROCK DISTANCES
        # ----------------------------------------------------

        rock_distances = {}

        for rock_name, rock in rocks.items():

            rock_position = rock.getPosition()

            distance_2d = math.hypot(
                rover_position[0] - rock_position[0],
                rover_position[1] - rock_position[1]
            )

            rock_distances[rock_name] = distance_2d


        closest_rock = min(
            rock_distances,
            key=rock_distances.get
        )

        closest_rock_distance = (
            rock_distances[closest_rock]
        )

        minimum_rock_distance = min(
            minimum_rock_distance,
            closest_rock_distance
        )

        # ----------------------------------------------------
        # World State (sent to controller)
        # ----------------------------------------------------
        world_state = {
            "time": current_time,

            "rover_position": [
                rover_position[0],
                rover_position[1],
                rover_position[2],
            ],

            "rover_orientation": list(
                rover_orientation
            ),

            "goal_position": [
                goal_position[0],
                goal_position[1],
                goal_position[2],
            ],

            "rocks": {},

            "rock_scales": {},
        }

        for rock_name, rock in rocks.items():

            rock_position = rock.getPosition()

            # --------------------------------------------------------
            # Rock position
            # --------------------------------------------------------

            world_state["rocks"][rock_name] = [
                rock_position[0],
                rock_position[1],
                rock_position[2],
            ]


            # --------------------------------------------------------
            # Rock scale
            # --------------------------------------------------------

            scale_field = rock.getField(
                "scale"
            )

            if scale_field is None:

                # Defensive fallback. The Rock PROTO should expose
                # a scale field, but assume the default scale if it
                # cannot be accessed.
                rock_scale = 1.0

            else:

                rock_scale = scale_field.getSFFloat()


            world_state["rock_scales"][rock_name] = (
                rock_scale
            )

        message = json.dumps(world_state)

        oracle_emitter.send(
            message.encode("utf-8")
        )

        # ----------------------------------------------------
        # CLEARANCE REQUIREMENT
        # ----------------------------------------------------

        if (
            closest_rock_distance < MIN_CENTER_DISTANCE_M
            and not clearance_violated
        ):
            clearance_violated = True

            print(
                f"\nWARNING: R2 clearance requirement violated!\n"
                f"t = {current_time:.2f} s\n"
                f"Rock = {closest_rock}\n"
                f"Center distance = {closest_rock_distance:.2f} m\n"
            )

        # ----------------------------------------------------
        # Goal Logic
        # ----------------------------------------------------
        goal_reached = (rover_distance_from_goal_2d <= GOAL_RADIUS_M)

        if (
            goal_reached
            and goal_reached_time is None
        ):
            goal_reached_time = current_time

        # ----------------------------------------------------
        # Collision Logic
        # ----------------------------------------------------
        collision_detected, detected_rock = (
            detect_rover_rock_collision()
        )

        if collision_detected:

            collision_occurred = True
            collision_rock = detected_rock

        # ----------------------------------------------------
        # MISSION DEADLINE
        # ----------------------------------------------------

        if (
            current_time > MISSION_TIMEOUT_S
            and not timeout_violated
        ):

            timeout_violated = True

            print()
            print("WARNING: R4 mission deadline exceeded!")
            print(
                f"Mission has not completed by "
                f"{MISSION_TIMEOUT_S:.2f} s."
            )
            print(
                "Simulation will continue for diagnostic purposes."
            )
            print()

        # ----------------------------------------------------
        # MISSION COMPLETION
        # ----------------------------------------------------

        if goal_reached:

            mission_complete = True

            if goal_reached_time <= MISSION_TIMEOUT_S:
                mission_end_reason = "GOAL REACHED"

            else:
                mission_end_reason = (
                    "GOAL REACHED AFTER DEADLINE"
                )


        elif collision_occurred:

            mission_complete = True

            mission_end_reason = (
                f"COLLISION WITH {collision_rock}"
            )


        elif current_time >= SIMULATION_HARD_STOP_S:

            mission_complete = True

            mission_end_reason = (
                "SIMULATION HARD STOP"
            )
        
        # ----------------------------------------------------
        # WRITE TO CSV
        # ----------------------------------------------------

        if current_time >= next_log_time:

            writer.writerow([
                f"{current_time:.3f}",

                f"{rover_position[0]:.4f}",
                f"{rover_position[1]:.4f}",
                f"{rover_position[2]:.4f}",

                f"{goal_position[0]:.4f}",
                f"{goal_position[1]:.4f}",
                f"{goal_position[2]:.4f}",

                f"{rover_distance_from_goal_2d:.4f}",

                closest_rock,
                f"{closest_rock_distance:.4f}",
            ])

            # Immediately push the latest data to disk.
            csv_file.flush()

            # ----------------------------------------------------
            # CONSOLE OUTPUT
            # ----------------------------------------------------
    
            print(
                f"t={current_time:.2f}s | "
                f"Goal={rover_distance_from_goal_2d:.2f}m | "
                f"Closest={closest_rock} "
                f"({closest_rock_distance:.2f}m)"
            )
            
            next_log_time += LOG_INTERVAL

        # Exit loop if mission is complete
        if mission_complete:
            break
# ----------------------------------------------------
# Mission Results
# ----------------------------------------------------

r1_no_collision = not collision_occurred

r2_safe_clearance = (
    minimum_rock_distance >= MIN_CENTER_DISTANCE_M
)

r3_goal_reached = goal_reached

r4_within_time = (
    goal_reached_time is not None
    and
    goal_reached_time <= MISSION_TIMEOUT_S
)

mission_success = (
    r1_no_collision
    and r2_safe_clearance
    and r3_goal_reached
    and r4_within_time
)

print()
print("========== MISSION SUMMARY ==========")

print(
    f"Mission ended because: "
    f"{mission_end_reason}"
)

print()

print(
    f"R1 - No collision: "
    f"{'PASS' if r1_no_collision else 'FAIL'}"
)

print(
    f"R2 - Minimum center distance: "
    f"{'PASS' if r2_safe_clearance else 'FAIL'}"
)

print(
    f"R3 - Goal reached: "
    f"{'PASS' if r3_goal_reached else 'FAIL'}"
)

print(
    f"R4 - Mission timeout: "
    f"{'PASS' if r4_within_time else 'FAIL'}"
)

print()
print(
    f"Minimum obstacle distance: "
    f"{minimum_rock_distance:.2f} m"
)

if goal_reached_time is not None:

    print(
        f"Goal reached at: "
        f"{goal_reached_time:.2f} s"
    )

else:

    print(
        "Goal reached at: NOT REACHED"
    )

if collision_rock is not None:
    print(
        f"Collision object: {collision_rock}"
    )

print()

if mission_success:
    print("MISSION RESULT: SUCCESS")
else:
    print("MISSION RESULT: FAILURE")