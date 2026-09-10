import argparse
import json
import math
import random
import sys

from pathlib import Path


# ============================================================
# PROJECT IMPORT SETUP
# ============================================================

PROJECT_ROOT = (
    Path(__file__)
    .resolve()
    .parents[1]
)

if str(PROJECT_ROOT) not in sys.path:

    sys.path.insert(
        0,
        str(PROJECT_ROOT)
    )


from planning.grid_planner import (
    ROCK_BASE_RADIUS_M,

    calculate_rock_planning_radius,

    build_occupancy_grid,
    world_to_grid,
    is_free,
    astar,
)

from simulation.terrain_generation.terrain_model import (
    TerrainModel,
)

from simulation.terrain_generation.rock_placement import (
    calculate_rock_translation_z,
)


# ============================================================
# FIXED MISSION CONFIGURATION
# ============================================================

# For Random Scenario Generator V1, keep the rover and goal
# fixed. Only the rock configuration changes.

ROVER_POSITION = [
    -0.262875,
    1.29,
    0.390796
]

ROVER_ROTATION = [
    0.0000020399499999950273,
    0.0000008449779999979402,
    0.9999999999975624,
    2.35619
]

GOAL_POSITION = [
    -3.21,
    3.34,
    0.0
]


# ============================================================
# RANDOM ROCK CONFIGURATION
# ============================================================

ROCK_NAMES = [
    "ROCK_01",
    "ROCK_02",
    "ROCK_03",
    "ROCK_04",
]


# Restrict V1 to rock sizes already used in development.
ROCK_SCALES = [
    1.0,
    2.0,
    3.0,
    4.0,
]


# Generate rocks in a region surrounding the start-to-goal
# mission corridor rather than across the entire terrain.
SCENARIO_MARGIN_M = 3.0


# Physical rocks should not intersect each other.
ROCK_SEPARATION_MARGIN_M = 0.10


# Prevent infinite generation loops if constraints
# become impossible to satisfy.
MAX_ROCK_PLACEMENT_ATTEMPTS = 500
MAX_SCENARIO_ATTEMPTS = 100


# ============================================================
# GEOMETRY HELPERS
# ============================================================

def distance_2d(
    point_a,
    point_b
):

    return math.hypot(
        point_a[0] - point_b[0],
        point_a[1] - point_b[1]
    )


def physical_rock_radius(
    scale
):

    return (
        ROCK_BASE_RADIUS_M
        *
        scale
    )


# ============================================================
# ROCK VALIDATION
# ============================================================

def rock_overlaps_existing_rock(
    candidate_position,
    candidate_scale,
    generated_rocks
):

    candidate_radius = (
        physical_rock_radius(
            candidate_scale
        )
    )


    for existing_rock in generated_rocks.values():

        existing_position = (
            existing_rock["position"]
        )

        existing_scale = (
            existing_rock["scale"]
        )

        existing_radius = (
            physical_rock_radius(
                existing_scale
            )
        )


        minimum_allowed_distance = (
            candidate_radius
            +
            existing_radius
            +
            ROCK_SEPARATION_MARGIN_M
        )


        actual_distance = distance_2d(
            candidate_position,
            existing_position
        )


        if (
            actual_distance
            <
            minimum_allowed_distance
        ):

            return True


    return False


def rock_blocks_fixed_point(
    rock_position,
    rock_scale,
    fixed_position
):

    envelope = (
        calculate_rock_planning_radius(
            rock_scale
        )
    )


    planning_radius = (
        envelope[
            "planning_radius_m"
        ]
    )


    distance = distance_2d(
        rock_position,
        fixed_position
    )


    return (
        distance
        <=
        planning_radius
    )


# ============================================================
# GENERATE ROCK CONFIGURATION
# ============================================================

def generate_rocks(
    rng,
    terrain
):

    generated_rocks = {}


    minimum_x = (
        min(
            ROVER_POSITION[0],
            GOAL_POSITION[0]
        )
        -
        SCENARIO_MARGIN_M
    )

    maximum_x = (
        max(
            ROVER_POSITION[0],
            GOAL_POSITION[0]
        )
        +
        SCENARIO_MARGIN_M
    )

    minimum_y = (
        min(
            ROVER_POSITION[1],
            GOAL_POSITION[1]
        )
        -
        SCENARIO_MARGIN_M
    )

    maximum_y = (
        max(
            ROVER_POSITION[1],
            GOAL_POSITION[1]
        )
        +
        SCENARIO_MARGIN_M
    )


    for rock_name in ROCK_NAMES:

        rock_placed = False


        for _ in range(
            MAX_ROCK_PLACEMENT_ATTEMPTS
        ):

            scale = rng.choice(
                ROCK_SCALES
            )


            x = rng.uniform(
                minimum_x,
                maximum_x
            )

            y = rng.uniform(
                minimum_y,
                maximum_y
            )


            # ----------------------------------------------
            # AUTOMATIC TERRAIN-AWARE Z PLACEMENT
            # ----------------------------------------------

            try:

                z = (
                    calculate_rock_translation_z(
                        terrain,
                        x,
                        y,
                        scale
                    )
                )

            except ValueError:

                # Part of the scaled rock would extend outside
                # the terrain.
                continue


            candidate_position = [
                x,
                y,
                z
            ]


            # ----------------------------------------------
            # DO NOT BLOCK ROVER START
            # ----------------------------------------------

            if rock_blocks_fixed_point(
                candidate_position,
                scale,
                ROVER_POSITION
            ):

                continue


            # ----------------------------------------------
            # DO NOT BLOCK GOAL
            # ----------------------------------------------

            if rock_blocks_fixed_point(
                candidate_position,
                scale,
                GOAL_POSITION
            ):

                continue


            # ----------------------------------------------
            # DO NOT PHYSICALLY OVERLAP ANOTHER ROCK
            # ----------------------------------------------

            if rock_overlaps_existing_rock(
                candidate_position,
                scale,
                generated_rocks
            ):

                continue


            # ----------------------------------------------
            # ACCEPT ROCK
            # ----------------------------------------------

            generated_rocks[
                rock_name
            ] = {
                "position": candidate_position,
                "scale": scale
            }


            rock_placed = True

            break


        if not rock_placed:

            return None


    return generated_rocks


# ============================================================
# PLANNING VALIDATION
# ============================================================

def validate_scenario_with_planner(
    generated_rocks
):

    rock_positions = {}

    rock_scales = {}


    for (
        rock_name,
        rock_config
    ) in generated_rocks.items():

        rock_positions[
            rock_name
        ] = rock_config[
            "position"
        ]

        rock_scales[
            rock_name
        ] = rock_config[
            "scale"
        ]


    occupancy_grid = (
        build_occupancy_grid(
            rock_positions,
            rock_scales,
            debug=False
        )
    )


    start_cell = world_to_grid(
        ROVER_POSITION[0],
        ROVER_POSITION[1]
    )

    goal_cell = world_to_grid(
        GOAL_POSITION[0],
        GOAL_POSITION[1]
    )


    if not is_free(
        occupancy_grid,
        start_cell
    ):

        return None


    if not is_free(
        occupancy_grid,
        goal_cell
    ):

        return None


    path = astar(
        occupancy_grid,
        start_cell,
        goal_cell
    )


    if path is None:

        return None


    return path


# ============================================================
# GENERATE COMPLETE SCENARIO
# ============================================================

def generate_scenario(
    seed,
    scenario_id
):

    rng = random.Random(
        seed
    )

    terrain = (
        TerrainModel.from_wbt()
    )


    for scenario_attempt in range(
        1,
        MAX_SCENARIO_ATTEMPTS + 1
    ):

        generated_rocks = (
            generate_rocks(
                rng,
                terrain
            )
        )


        if generated_rocks is None:

            continue


        path = (
            validate_scenario_with_planner(
                generated_rocks
            )
        )


        if path is None:

            continue


        scenario = {

            "scenario_id":
                scenario_id,

            "description":
                (
                    "Randomly generated SafeNav "
                    "perfect-information scenario."
                ),

            "seed":
                seed,

            "generator_version":
                "v1",

            "rover": {

                "position":
                    ROVER_POSITION,

                "rotation":
                    ROVER_ROTATION,
            },

            "goal": {

                "position":
                    GOAL_POSITION,
            },

            "rocks":
                generated_rocks,
        }


        return (
            scenario,
            path,
            scenario_attempt
        )


    raise RuntimeError(
        "Unable to generate a valid scenario "
        f"after {MAX_SCENARIO_ATTEMPTS} attempts."
    )


# ============================================================
# SAVE SCENARIO
# ============================================================

def save_scenario(
    scenario,
    output_file
):

    output_file = Path(
        output_file
    )


    if not output_file.is_absolute():

        output_file = (
            PROJECT_ROOT
            / "scenarios"
            / output_file
        )


    with output_file.open(
        "w",
        encoding="utf-8"
    ) as file:

        json.dump(
            scenario,
            file,
            indent=2
        )


    return output_file


# ============================================================
# COMMAND-LINE ENTRY POINT
# ============================================================

def main():

    parser = argparse.ArgumentParser(
        description=(
            "Generate one reproducible "
            "SafeNav random scenario."
        )
    )


    parser.add_argument(
        "--seed",
        type=int,
        default=1001,
        help=(
            "Random seed used to generate "
            "the scenario."
        )
    )


    parser.add_argument(
        "--output",
        default="random_000001.json",
        help=(
            "Output JSON filename."
        )
    )


    args = parser.parse_args()


    scenario_id = (
        Path(args.output).stem
    )


    scenario, path, attempts = (
        generate_scenario(
            seed=args.seed,
            scenario_id=scenario_id
        )
    )


    output_file = save_scenario(
        scenario,
        args.output
    )


    print()
    print(
        "========== RANDOM SCENARIO GENERATED =========="
    )

    print(
        f"Scenario: {scenario_id}"
    )

    print(
        f"Seed: {args.seed}"
    )

    print(
        f"Generation attempts: {attempts}"
    )

    print()


    for (
        rock_name,
        rock_config
    ) in scenario["rocks"].items():

        position = (
            rock_config["position"]
        )

        scale = (
            rock_config["scale"]
        )


        print(
            f"{rock_name}: "
            f"position=("
            f"{position[0]:.3f}, "
            f"{position[1]:.3f}, "
            f"{position[2]:.3f}), "
            f"scale={scale:.1f}"
        )


    print()

    print(
        f"A* path cells: "
        f"{len(path)}"
    )

    print(
        f"Saved to:\n"
        f"{output_file}"
    )


if __name__ == "__main__":

    main()