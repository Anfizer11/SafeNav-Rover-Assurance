import os
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]

WORLD_FILE = (
    PROJECT_ROOT
    / "simulation"
    / "worlds"
    / "safenav_mars_dev.wbt"
)

WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

START_SEED = 4101
NUM_SCENARIOS = 20


# ============================================================
# IMPORT RANDOM SCENARIO GENERATOR
# ============================================================

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from scenarios.generate_random_scenario import generate_scenario, save_scenario


# ============================================================
# COLLECTION
# ============================================================

for seed in range(START_SEED, START_SEED + NUM_SCENARIOS):

    scenario_id = f"random_seed_{seed:06d}"

    print()
    print("============================================")
    print(f"Collecting perception data for {scenario_id}")
    print("============================================")

    scenario, planned_path, attempts = generate_scenario(
        seed,
        scenario_id
    )

    scenario_file = PROJECT_ROOT / "scenarios" / f"{scenario_id}.json"

    save_scenario(
        scenario,
        scenario_file
    )

    env = os.environ.copy()

    env["SAFENAV_SCENARIO"] = scenario_id
    env["SAFENAV_BATCH_MODE"] = "1"
    env["SAFENAV_CAPTURE_PERCEPTION"] = "1"

    result = subprocess.run(
        [
            WEBOTS_EXE,
            "--batch",
            "--minimize",
            "--mode=fast",
            "--stdout",
            "--stderr",
            str(WORLD_FILE)
        ],
        env=env
    )

    if result.returncode != 0:
        print(f"ERROR: Webots failed for {scenario_id}")
        continue

    capture_directory = (
        PROJECT_ROOT
        / "data"
        / "frames"
        / "perception_raw"
        / scenario_id
    )

    frame_count = len(list(capture_directory.glob("*.png")))

    print(
        f"{scenario_id} complete | "
        f"frames captured={frame_count}"
    )

    print(
        f"Generated {scenario_id} | "
        f"attempts={attempts} | "
        f"path cells={len(planned_path)}"
    )


print()
print("============================================")
print("PERCEPTION COLLECTION COMPLETE")
print("============================================")