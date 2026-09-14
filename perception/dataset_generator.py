import json
import shutil
from pathlib import Path

from rock_geometry import REGULAR_ROCK_LOCAL_VERTICES
from projection import get_camera_world_pose, project_rock_bounding_box
from labels import bounding_box_to_yolo, format_yolo_label


# ============================================================
# DIRECTORIES
# ============================================================

PROJECT_ROOT = Path(__file__).resolve().parents[1]

SOURCE_ROOT = PROJECT_ROOT / "data" / "frames" / "perception_raw"

DATASET_DIRECTORY = PROJECT_ROOT / "data" / "datasets" / "perception_v1"
IMAGES_DIRECTORY = DATASET_DIRECTORY / "images"
LABELS_DIRECTORY = DATASET_DIRECTORY / "labels"
METADATA_DIRECTORY = DATASET_DIRECTORY / "metadata"

IMAGES_DIRECTORY.mkdir(parents=True, exist_ok=True)
LABELS_DIRECTORY.mkdir(parents=True, exist_ok=True)
METADATA_DIRECTORY.mkdir(parents=True, exist_ok=True)


# ============================================================
# FIND SOURCE FRAMES
# ============================================================

state_files = sorted(SOURCE_ROOT.glob("*/frame_*_state.json"))

if not state_files:
    raise RuntimeError(f"No state files found in {SOURCE_ROOT}")


# ============================================================
# GENERATE DATASET
# ============================================================

metadata_records = []

positive_frames = 0
negative_frames = 0
total_objects = 0


for state_file in state_files:

    with open(state_file, "r") as file:
        state = json.load(file)

    scenario_id = state["scenario_id"]

    frame_name = state_file.name.replace("_state.json", "")
    source_image_name = f"{frame_name}.png"
    source_image_path = state_file.parent / source_image_name

    if not source_image_path.exists():
        raise RuntimeError(f"Missing image for {state_file}")

    # Globally unique name across scenarios.
    output_stem = f"{scenario_id}_{frame_name}"

    output_image_name = f"{output_stem}.png"
    output_label_name = f"{output_stem}.txt"

    rover_position = state["rover_position"]
    rover_orientation = state["rover_orientation"]
    rocks = state["rocks"]
    rock_scales = state["rock_scales"]

    camera_position, camera_rotation = get_camera_world_pose(
        rover_position,
        rover_orientation
    )

    yolo_labels = []
    visible_rocks = []

    # --------------------------------------------------------
    # GENERATE LABELS
    # --------------------------------------------------------

    for rock_name, rock_position in rocks.items():

        rock_scale = rock_scales[rock_name]

        bounding_box = project_rock_bounding_box(
            rock_position,
            rock_scale,
            camera_position,
            camera_rotation,
            REGULAR_ROCK_LOCAL_VERTICES
        )

        if bounding_box is None:
            continue

        yolo_box = bounding_box_to_yolo(bounding_box)
        label_line = format_yolo_label(0, yolo_box)

        yolo_labels.append(label_line)
        visible_rocks.append(rock_name)

    # --------------------------------------------------------
    # SAVE IMAGE
    # --------------------------------------------------------

    output_image_path = IMAGES_DIRECTORY / output_image_name
    shutil.copy2(source_image_path, output_image_path)

    # --------------------------------------------------------
    # SAVE YOLO LABEL
    # --------------------------------------------------------

    output_label_path = LABELS_DIRECTORY / output_label_name

    with open(output_label_path, "w") as file:
        if yolo_labels:
            file.write("\n".join(yolo_labels) + "\n")

    # --------------------------------------------------------
    # METADATA
    # --------------------------------------------------------

    metadata_record = {
        "scenario_id": scenario_id,
        "source_frame": source_image_name,
        "image": output_image_name,
        "label": output_label_name,
        "time": state["time"],
        "visible_rocks": visible_rocks,
        "rover_position": rover_position,
        "rover_orientation": rover_orientation
    }

    metadata_records.append(metadata_record)

    if visible_rocks:
        positive_frames += 1
    else:
        negative_frames += 1

    total_objects += len(visible_rocks)

    print(
        f"{output_image_name} | "
        f"visible rocks={len(visible_rocks)} | "
        f"{visible_rocks}"
    )


# ============================================================
# SAVE METADATA
# ============================================================

metadata_path = METADATA_DIRECTORY / "frames.jsonl"

with open(metadata_path, "w") as file:
    for record in metadata_records:
        file.write(json.dumps(record) + "\n")


# ============================================================
# SUMMARY
# ============================================================

print()
print("========== DATASET GENERATION COMPLETE ==========")
print(f"Frames processed: {len(state_files)}")
print(f"Positive frames: {positive_frames}")
print(f"Negative frames: {negative_frames}")
print(f"Rock instances: {total_objects}")
print(f"Images: {IMAGES_DIRECTORY}")
print(f"Labels: {LABELS_DIRECTORY}")
print(f"Metadata: {metadata_path}")