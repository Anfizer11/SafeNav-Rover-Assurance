import json
import shutil
from pathlib import Path

# ============================================================
# DIRECTORIES
# ============================================================

PROJECT_ROOT = Path(__file__).resolve().parents[1]

SOURCE_DATASET = PROJECT_ROOT / "data" / "datasets" / "perception_v1"

SOURCE_IMAGES = SOURCE_DATASET / "images"
SOURCE_LABELS = SOURCE_DATASET / "labels"
METADATA_PATH = SOURCE_DATASET / "metadata" / "frames.jsonl"

OUTPUT_DATASET = PROJECT_ROOT / "data" / "datasets" / "perception_v1_split"


# ============================================================
# SCENARIO SPLITS
# ============================================================

VALIDATION_SCENARIOS = {
    "random_seed_004105",
    "random_seed_004108",
    "random_seed_004110",
    "random_seed_004111",
}

TEST_SCENARIOS = {
    "random_seed_004004",
    "random_seed_004102",
    "random_seed_004114",
    "random_seed_004117",
}


# ============================================================
# CREATE DIRECTORIES
# ============================================================

for split_name in ["train", "val", "test"]:

    image_directory = OUTPUT_DATASET / "images" / split_name
    label_directory = OUTPUT_DATASET / "labels" / split_name

    image_directory.mkdir(parents=True, exist_ok=True)
    label_directory.mkdir(parents=True, exist_ok=True)


# ============================================================
# LOAD METADATA AND SPLIT
# ============================================================

split_counts = {
    "train": 0,
    "val": 0,
    "test": 0
}

positive_counts = {
    "train": 0,
    "val": 0,
    "test": 0
}

with open(METADATA_PATH, "r") as file:

    for line in file:

        record = json.loads(line)

        scenario_id = record["scenario_id"]
        image_name = record["image"]
        label_name = record["label"]

        if scenario_id in VALIDATION_SCENARIOS:
            split_name = "val"

        elif scenario_id in TEST_SCENARIOS:
            split_name = "test"

        else:
            split_name = "train"

        source_image = SOURCE_IMAGES / image_name
        source_label = SOURCE_LABELS / label_name

        output_image = OUTPUT_DATASET / "images" / split_name / image_name
        output_label = OUTPUT_DATASET / "labels" / split_name / label_name

        if not source_image.exists():
            raise RuntimeError(f"Missing image: {source_image}")

        if not source_label.exists():
            raise RuntimeError(f"Missing label: {source_label}")

        shutil.copy2(source_image, output_image)
        shutil.copy2(source_label, output_label)

        split_counts[split_name] += 1

        if record["visible_rocks"]:
            positive_counts[split_name] += 1


# ============================================================
# SUMMARY
# ============================================================

print()
print("========== DATASET SPLIT COMPLETE ==========")

for split_name in ["train", "val", "test"]:

    total = split_counts[split_name]
    positive = positive_counts[split_name]
    negative = total - positive

    positive_rate = 100.0 * positive / total

    print()
    print(f"{split_name.upper()}")
    print(f"Frames: {total}")
    print(f"Positive: {positive}")
    print(f"Negative: {negative}")
    print(f"Positive rate: {positive_rate:.1f}%")




# TRAIN — 17 scenarios
# random_seed_004001
# random_seed_004002
# random_seed_004003
# random_seed_004005
# random_seed_004101
# random_seed_004103
# random_seed_004104
# random_seed_004106
# random_seed_004107
# random_seed_004109
# random_seed_004112
# random_seed_004113
# random_seed_004115
# random_seed_004116
# random_seed_004118
# random_seed_004119
# random_seed_004120

# VALIDATION — 4 scenarios
# random_seed_004105
# random_seed_004108
# random_seed_004110
# random_seed_004111

# TEST — 4 scenarios
# random_seed_004004
# random_seed_004102
# random_seed_004114
# random_seed_004117

# TRAIN
# 1437 frames
# 500 positive
# 937 negative
# 34.8% positive
# 532 rock instances

# VALIDATION
# 308 frames
# 115 positive
# 193 negative
# 37.3% positive
# 115 rock instances

# TEST
# 323 frames
# 117 positive
# 206 negative
# 36.2% positive
# 120 rock instances