from pathlib import Path

from ultralytics import YOLO


# ============================================================
# PATHS
# ============================================================

PROJECT_ROOT = Path(__file__).resolve().parents[1]

DATASET_YAML = (
    PROJECT_ROOT
    / "data"
    / "datasets"
    / "perception_v1_split"
    / "dataset.yaml"
)

RESULTS_DIRECTORY = (
    PROJECT_ROOT
    / "data"
    / "results"
    / "perception"
)


# ============================================================
# TRAINING
# ============================================================

def main():

    if not DATASET_YAML.exists():
        raise RuntimeError(f"Dataset YAML not found: {DATASET_YAML}")

    model = YOLO("yolov8n.pt")

    model.train(
        # data=str(DATASET_YAML),
        # epochs=1,
        # imgsz=640,
        # batch=16,
        # workers=0,
        # project=str(RESULTS_DIRECTORY),
        # name="yolov8n_smoke_test"

        data=str(DATASET_YAML),
        epochs=30,
        imgsz=640,
        batch=16,
        workers=0,
        seed=0,
        deterministic=True,
        patience=10,
        project=str(RESULTS_DIRECTORY),
        name="yolov8n_perception_v1"
    )


if __name__ == "__main__":
    main()