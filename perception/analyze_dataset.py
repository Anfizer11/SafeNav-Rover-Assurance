import json
from pathlib import Path


# ============================================================
# PATHS
# ============================================================

PROJECT_ROOT = Path(__file__).resolve().parents[1]

METADATA_PATH = (
    PROJECT_ROOT
    / "data"
    / "datasets"
    / "perception_v1"
    / "metadata"
    / "frames.jsonl"
)


# ============================================================
# LOAD METADATA
# ============================================================

if not METADATA_PATH.exists():
    raise RuntimeError(f"Metadata file not found: {METADATA_PATH}")

scenario_stats = {}

with open(METADATA_PATH, "r") as file:

    for line in file:

        record = json.loads(line)

        scenario_id = record["scenario_id"]
        visible_rocks = record["visible_rocks"]

        if scenario_id not in scenario_stats:
            scenario_stats[scenario_id] = {
                "frames": 0,
                "positive_frames": 0,
                "negative_frames": 0,
                "rock_instances": 0,
                "rock_names": set()
            }

        stats = scenario_stats[scenario_id]

        stats["frames"] += 1
        stats["rock_instances"] += len(visible_rocks)

        if visible_rocks:
            stats["positive_frames"] += 1
        else:
            stats["negative_frames"] += 1

        stats["rock_names"].update(visible_rocks)


# ============================================================
# PRINT PER-SCENARIO SUMMARY
# ============================================================

print()
print("========== PER-SCENARIO DATASET SUMMARY ==========")
print()

for scenario_id in sorted(scenario_stats):

    stats = scenario_stats[scenario_id]

    positive_rate = (
        stats["positive_frames"]
        / stats["frames"]
        * 100.0
    )

    rock_names = sorted(stats["rock_names"])

    print(
        f"{scenario_id} | "
        f"frames={stats['frames']} | "
        f"positive={stats['positive_frames']} | "
        f"negative={stats['negative_frames']} | "
        f"positive rate={positive_rate:.1f}% | "
        f"instances={stats['rock_instances']} | "
        f"rocks={rock_names}"
    )


# ============================================================
# OVERALL SUMMARY
# ============================================================

total_frames = sum(
    stats["frames"]
    for stats in scenario_stats.values()
)

total_positive = sum(
    stats["positive_frames"]
    for stats in scenario_stats.values()
)

total_negative = sum(
    stats["negative_frames"]
    for stats in scenario_stats.values()
)

total_instances = sum(
    stats["rock_instances"]
    for stats in scenario_stats.values()
)

print()
print("========== OVERALL SUMMARY ==========")
print(f"Scenarios: {len(scenario_stats)}")
print(f"Frames: {total_frames}")
print(f"Positive frames: {total_positive}")
print(f"Negative frames: {total_negative}")
print(f"Rock instances: {total_instances}")
print(f"Positive rate: {100.0 * total_positive / total_frames:.1f}%")