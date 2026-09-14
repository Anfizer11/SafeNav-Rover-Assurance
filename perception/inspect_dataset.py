from pathlib import Path

import cv2


# ============================================================
# DIRECTORIES
# ============================================================

PROJECT_ROOT = Path(__file__).resolve().parents[1]

DATASET_DIRECTORY = PROJECT_ROOT / "data" / "datasets" / "perception_v1"
IMAGES_DIRECTORY = DATASET_DIRECTORY / "images"
LABELS_DIRECTORY = DATASET_DIRECTORY / "labels"
INSPECTION_DIRECTORY = DATASET_DIRECTORY / "inspection"

INSPECTION_DIRECTORY.mkdir(parents=True, exist_ok=True)


# ============================================================
# FRAMES TO INSPECT
# ============================================================

SAMPLE_FRAMES = [
    "random_seed_004001_frame_0030",
    "random_seed_004002_frame_0000",
    "random_seed_004002_frame_0037",
    "random_seed_004002_frame_0073",
    "random_seed_004003_frame_0000",
    "random_seed_004003_frame_0019",
    "random_seed_004003_frame_0020",
    "random_seed_004004_frame_0050",
    "random_seed_004005_frame_0030",
]


# ============================================================
# DRAW YOLO LABELS
# ============================================================

for frame_name in SAMPLE_FRAMES:

    image_path = IMAGES_DIRECTORY / f"{frame_name}.png"
    label_path = LABELS_DIRECTORY / f"{frame_name}.txt"

    if not image_path.exists():
        print(f"Missing image: {image_path.name}")
        continue

    if not label_path.exists():
        print(f"Missing label: {label_path.name}")
        continue

    image = cv2.imread(str(image_path))

    if image is None:
        print(f"Could not load: {image_path.name}")
        continue

    image_height, image_width = image.shape[:2]

    with open(label_path, "r") as file:
        label_lines = [line.strip() for line in file if line.strip()]

    for label_line in label_lines:

        class_id, x_center, y_center, box_width, box_height = label_line.split()

        x_center = float(x_center) * image_width
        y_center = float(y_center) * image_height
        box_width = float(box_width) * image_width
        box_height = float(box_height) * image_height

        x_min = int(x_center - box_width / 2.0)
        y_min = int(y_center - box_height / 2.0)
        x_max = int(x_center + box_width / 2.0)
        y_max = int(y_center + box_height / 2.0)

        cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)

        cv2.putText(
            image,
            f"rock ({class_id})",
            (x_min, max(20, y_min - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            2
        )

    output_path = INSPECTION_DIRECTORY / f"{frame_name}_annotated.png"
    cv2.imwrite(str(output_path), image)

    print(
        f"{frame_name} | "
        f"objects={len(label_lines)} | "
        f"saved={output_path.name}"
    )


print()
print(f"Inspection images: {INSPECTION_DIRECTORY}")