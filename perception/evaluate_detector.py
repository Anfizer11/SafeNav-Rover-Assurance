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

MODEL_PATH = (
    PROJECT_ROOT
    / "data"
    / "results"
    / "perception"
    / "yolov8n_perception_v1"
    / "weights"
    / "best.pt"
)

RESULTS_DIRECTORY = (
    PROJECT_ROOT
    / "data"
    / "results"
    / "perception"
)


# ============================================================
# EVALUATION
# ============================================================

def main():

    if not DATASET_YAML.exists():
        raise RuntimeError(f"Dataset YAML not found: {DATASET_YAML}")

    if not MODEL_PATH.exists():
        raise RuntimeError(f"Model weights not found: {MODEL_PATH}")

    model = YOLO(str(MODEL_PATH))

    model.val(
        data=str(DATASET_YAML),
        split="test",
        imgsz=640,
        batch=16,
        workers=0,
        plots=True,
        project=str(RESULTS_DIRECTORY),
        name="yolov8n_perception_v1_test"
    )


if __name__ == "__main__":
    main()