import json
import os
import statistics
import subprocess
import sys
import time

from pathlib import Path


# ============================================================
# PROJECT PATHS
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


WORLD_FILE = (
    PROJECT_ROOT
    / "simulation"
    / "worlds"
    / "safenav_mars_dev.wbt"
)

RESULT_DIRECTORY = (
    PROJECT_ROOT
    / "data"
    / "results"
)

SCENARIO_DIRECTORY = (
    PROJECT_ROOT
    / "scenarios"
)


# ============================================================
# IMPORT RANDOM SCENARIO GENERATOR
# ============================================================

from scenarios.generate_random_scenario import (
    generate_scenario,
    save_scenario,
)


# ============================================================
# WEBOTS EXECUTABLE
# ============================================================

WEBOTS_EXECUTABLE = Path(
    r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
)


# ============================================================
# EXPERIMENT CONFIGURATION
# ============================================================

START_SEED = 3001

NUM_SCENARIOS = 100

EXPERIMENT_NAME = (
    "perfect_information_baseline_v1"
)


EXPERIMENT_CONFIG = {

    "controller": {

        "turn_start_angle_deg":
            35.0,

        "turn_stop_angle_deg":
            15.0,
    },


    "requirements": {

        "minimum_center_distance_m":
            1.0,

        "goal_radius_m":
            0.75,

        "mission_timeout_s":
            700.0,
    },


    "planning": {

        "planning_clearance_m":
            1.40,

        "grid_resolution_m":
            0.5,
    },
}


# ============================================================
# SCENARIO NAMING
# ============================================================

def scenario_id_from_seed(
    seed
):

    return (
        f"random_seed_{seed:06d}"
    )


# ============================================================
# GENERATE ONE SCENARIO
# ============================================================

def generate_experiment_scenario(
    seed
):

    scenario_id = (
        scenario_id_from_seed(
            seed
        )
    )

    output_filename = (
        f"{scenario_id}.json"
    )


    scenario, path, attempts = (
        generate_scenario(
            seed=seed,
            scenario_id=scenario_id
        )
    )


    output_file = save_scenario(
        scenario,
        output_filename
    )


    print()
    print(
        "--------------------------------------------"
    )

    print(
        "SCENARIO GENERATED"
    )

    print(
        "--------------------------------------------"
    )

    print(
        f"Scenario: {scenario_id}"
    )

    print(
        f"Seed: {seed}"
    )

    print(
        f"Generation attempts: {attempts}"
    )

    print(
        f"A* cells: {len(path)}"
    )

    print(
        f"Saved to: {output_file}"
    )


    return (
        scenario_id,
        output_file
    )


# ============================================================
# FIND NEW RESULT FILE
# ============================================================

def find_new_result_file(
    scenario_id,
    files_before
):

    matching_files = set(
        RESULT_DIRECTORY.glob(
            f"result_{scenario_id}_*.json"
        )
    )


    new_files = (
        matching_files
        -
        files_before
    )


    if len(new_files) == 0:

        raise RuntimeError(
            f"No new result JSON was created "
            f"for {scenario_id}."
        )


    return max(
        new_files,
        key=lambda path:
            path.stat().st_mtime
    )


# ============================================================
# RUN ONE WEBOTS MISSION
# ============================================================

def run_scenario(
    scenario_id
):

    print()
    print(
        "============================================"
    )

    print(
        f"RUNNING: {scenario_id}"
    )

    print(
        "============================================"
    )


    files_before = set(
        RESULT_DIRECTORY.glob(
            f"result_{scenario_id}_*.json"
        )
    )


    environment = (
        os.environ.copy()
    )


    environment[
        "SAFENAV_SCENARIO"
    ] = scenario_id


    environment[
        "SAFENAV_BATCH_MODE"
    ] = "1"


    command = [
        str(WEBOTS_EXECUTABLE),

        "--batch",
        "--minimize",
        "--mode=fast",
        "--stdout",
        "--stderr",

        str(WORLD_FILE),
    ]


    start_wall_time = (
        time.time()
    )


    completed_process = (
        subprocess.run(
            command,
            cwd=PROJECT_ROOT,
            env=environment,
        )
    )


    wall_clock_time = (
        time.time()
        -
        start_wall_time
    )


    if (
        completed_process.returncode
        != 0
    ):

        raise RuntimeError(
            f"Webots exited with code "
            f"{completed_process.returncode} "
            f"while running "
            f"{scenario_id}."
        )


    result_file = (
        find_new_result_file(
            scenario_id,
            files_before
        )
    )


    with result_file.open(
        "r",
        encoding="utf-8"
    ) as file:

        result = json.load(
            file
        )


    result[
        "wall_clock_time_s"
    ] = wall_clock_time


    return result


# ============================================================
# RUN ONE COMPLETE EXPERIMENT CASE
# ============================================================

def run_experiment_case(
    seed
):

    scenario_id, scenario_file = (
        generate_experiment_scenario(
            seed
        )
    )


    result = run_scenario(
        scenario_id
    )


    # Add experiment metadata that isn't part
    # of the Supervisor's mission result.
    result[
        "scenario_file"
    ] = str(
        scenario_file
    )


    print()
    print(
        "--------------------------------------------"
    )

    print(
        f"COMPLETED: {scenario_id}"
    )

    print(
        "--------------------------------------------"
    )


    print(
        f"Result: "
        f"{'SUCCESS' if result['mission_success'] else 'FAILURE'}"
    )


    print(
        f"Goal time: "
        f"{result['goal_reached_time_s']} s"
    )


    print(
        f"Minimum clearance: "
        f"{result['minimum_rock_distance_m']:.3f} m"
    )


    print(
        f"Wall-clock runtime: "
        f"{result['wall_clock_time_s']:.2f} s"
    )


    return result


# ============================================================
# STATISTICS
# ============================================================

def calculate_summary(
    results
):

    scenario_count = len(
        results
    )


    successful_missions = sum(
        1
        for result in results
        if result[
            "mission_success"
        ]
    )


    r1_pass_count = sum(
        1
        for result in results
        if result[
            "requirements"
        ][
            "r1_no_collision"
        ]
    )


    r2_pass_count = sum(
        1
        for result in results
        if result[
            "requirements"
        ][
            "r2_safe_clearance"
        ]
    )


    r3_pass_count = sum(
        1
        for result in results
        if result[
            "requirements"
        ][
            "r3_goal_reached"
        ]
    )


    r4_pass_count = sum(
        1
        for result in results
        if result[
            "requirements"
        ][
            "r4_within_time"
        ]
    )


    goal_times = [
        result[
            "goal_reached_time_s"
        ]

        for result in results

        if result[
            "goal_reached_time_s"
        ] is not None
    ]


    minimum_clearances = [
        result[
            "minimum_rock_distance_m"
        ]

        for result in results
    ]


    wall_clock_times = [
        result[
            "wall_clock_time_s"
        ]

        for result in results
    ]


    summary = {

        "experiment":
            EXPERIMENT_NAME,

        "configuration":
            EXPERIMENT_CONFIG,

        "start_seed":
            START_SEED,

        "scenario_count":
            scenario_count,

        "successful_missions":
            successful_missions,

        "success_rate":
            (
                successful_missions
                /
                scenario_count
            ),


        "requirements": {

            "r1_no_collision_pass_count":
                r1_pass_count,

            "r1_no_collision_pass_rate":
                r1_pass_count
                /
                scenario_count,


            "r2_safe_clearance_pass_count":
                r2_pass_count,

            "r2_safe_clearance_pass_rate":
                r2_pass_count
                /
                scenario_count,


            "r3_goal_reached_pass_count":
                r3_pass_count,

            "r3_goal_reached_pass_rate":
                r3_pass_count
                /
                scenario_count,


            "r4_within_time_pass_count":
                r4_pass_count,

            "r4_within_time_pass_rate":
                r4_pass_count
                /
                scenario_count,
        },


        "mission_time_s": {

            "mean":
                (
                    statistics.mean(
                        goal_times
                    )
                    if goal_times
                    else None
                ),

            "median":
                (
                    statistics.median(
                        goal_times
                    )
                    if goal_times
                    else None
                ),

            "minimum":
                (
                    min(
                        goal_times
                    )
                    if goal_times
                    else None
                ),

            "maximum":
                (
                    max(
                        goal_times
                    )
                    if goal_times
                    else None
                ),
        },


        "minimum_clearance_m": {

            "mean":
                statistics.mean(
                    minimum_clearances
                ),

            "minimum":
                min(
                    minimum_clearances
                ),

            "maximum":
                max(
                    minimum_clearances
                ),
        },


        "wall_clock_time_s": {

            "total":
                sum(
                    wall_clock_times
                ),

            "mean_per_scenario":
                statistics.mean(
                    wall_clock_times
                ),
        },


        "results":
            results,
    }


    return summary


# ============================================================
# SAVE EXPERIMENT SUMMARY
# ============================================================

def save_summary(
    summary
):

    summary_file = (
        RESULT_DIRECTORY
        /
        (
            f"{EXPERIMENT_NAME}"
            "_summary.json"
        )
    )


    with summary_file.open(
        "w",
        encoding="utf-8"
    ) as file:

        json.dump(
            summary,
            file,
            indent=2
        )


    return summary_file


# ============================================================
# MAIN
# ============================================================

def main():

    # --------------------------------------------------------
    # VALIDATE PATHS
    # --------------------------------------------------------

    if not WEBOTS_EXECUTABLE.exists():

        raise FileNotFoundError(
            "Could not find Webots executable:\n"
            f"{WEBOTS_EXECUTABLE}"
        )


    if not WORLD_FILE.exists():

        raise FileNotFoundError(
            "Could not find Webots world:\n"
            f"{WORLD_FILE}"
        )


    RESULT_DIRECTORY.mkdir(
        parents=True,
        exist_ok=True
    )


    # --------------------------------------------------------
    # EXPERIMENT HEADER
    # --------------------------------------------------------

    print()
    print(
        "============================================"
    )

    print(
        "SAFENAV PERFECT-INFORMATION PILOT"
    )

    print(
        "============================================"
    )


    print(
        f"Start seed: "
        f"{START_SEED}"
    )

    print(
        f"Number of scenarios: "
        f"{NUM_SCENARIOS}"
    )


    print(
        f"Final seed: "
        f"{START_SEED + NUM_SCENARIOS - 1}"
    )


    # --------------------------------------------------------
    # RUN EXPERIMENT
    # --------------------------------------------------------

    results = []


    for experiment_index in range(
        NUM_SCENARIOS
    ):

        seed = (
            START_SEED
            +
            experiment_index
        )


        print()
        print(
            "############################################"
        )

        print(
            f"EXPERIMENT "
            f"{experiment_index + 1}"
            f"/{NUM_SCENARIOS}"
        )

        print(
            "############################################"
        )


        result = (
            run_experiment_case(
                seed
            )
        )


        results.append(
            result
        )


    # --------------------------------------------------------
    # AGGREGATE RESULTS
    # --------------------------------------------------------

    summary = (
        calculate_summary(
            results
        )
    )


    summary_file = (
        save_summary(
            summary
        )
    )


    # --------------------------------------------------------
    # FINAL CONSOLE SUMMARY
    # --------------------------------------------------------

    print()
    print(
        "============================================"
    )

    print(
        "PERFECT-INFORMATION PILOT COMPLETE"
    )

    print(
        "============================================"
    )


    print(
        f"Scenarios: "
        f"{summary['scenario_count']}"
    )


    print(
        f"Successful: "
        f"{summary['successful_missions']}"
    )


    print(
        f"Success rate: "
        f"{summary['success_rate'] * 100:.1f}%"
    )


    print()


    print(
        "Requirement pass rates:"
    )

    print(
        f"R1: "
        f"{summary['requirements']['r1_no_collision_pass_rate'] * 100:.1f}%"
    )

    print(
        f"R2: "
        f"{summary['requirements']['r2_safe_clearance_pass_rate'] * 100:.1f}%"
    )

    print(
        f"R3: "
        f"{summary['requirements']['r3_goal_reached_pass_rate'] * 100:.1f}%"
    )

    print(
        f"R4: "
        f"{summary['requirements']['r4_within_time_pass_rate'] * 100:.1f}%"
    )


    print()


    print(
        f"Mean goal time: "
        f"{summary['mission_time_s']['mean']:.2f} s"
    )

    print(
        f"Median goal time: "
        f"{summary['mission_time_s']['median']:.2f} s"
    )


    print()


    print(
        f"Mean minimum clearance: "
        f"{summary['minimum_clearance_m']['mean']:.3f} m"
    )

    print(
        f"Worst minimum clearance: "
        f"{summary['minimum_clearance_m']['minimum']:.3f} m"
    )


    print()


    print(
        f"Total wall-clock time: "
        f"{summary['wall_clock_time_s']['total']:.2f} s"
    )


    print()


    print(
        f"Summary saved to:\n"
        f"{summary_file}"
    )


if __name__ == "__main__":

    main()