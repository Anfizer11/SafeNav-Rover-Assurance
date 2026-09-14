import json
from pathlib import Path
import cv2

from rock_geometry import REGULAR_ROCK_LOCAL_VERTICES
from projection import (
    world_to_pixel,
    get_camera_world_pose,
    project_rock_bounding_box,
    CAMERA_WIDTH,
    CAMERA_HEIGHT
)
from labels import bounding_box_to_yolo, format_yolo_label


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEBUG_DIRECTORY = PROJECT_ROOT / "data" / "frames" / "projection_debug"

state_files = sorted(DEBUG_DIRECTORY.glob("frame_*_state.json"))

if not state_files:
    raise RuntimeError(f"No debug state files found in {DEBUG_DIRECTORY}")


print("========== PROJECTION DEBUG ==========")
print()

visible_frame_count = 0
closest_candidates = []


for state_file in state_files:

    with open(state_file, "r") as file:
        state = json.load(file)

    rover_position = state["rover_position"]
    rover_orientation = state["rover_orientation"]
    rocks = state["rocks"]
    rock_scales = state["rock_scales"]

    yolo_labels = []

    camera_position, camera_rotation = get_camera_world_pose(
        rover_position,
        rover_orientation
    )

    image_name = state_file.name.replace("_state.json", ".png")
    image_path = DEBUG_DIRECTORY / image_name

    image = cv2.imread(str(image_path))

    if image is None:
        raise RuntimeError(f"Could not read image: {image_path}")

    frame_has_visible_rock = False

    for rock_name, rock_position in rocks.items():

        rock_scale = rock_scales[rock_name]

        bounding_box = project_rock_bounding_box(
            rock_position,
            rock_scale,
            camera_position,
            camera_rotation,
            REGULAR_ROCK_LOCAL_VERTICES
        )

        # No visible portion of this rock projects into the image.
        if bounding_box is None:
            continue

        frame_has_visible_rock = True
        visible_frame_count += 1

        x_min, y_min, x_max, y_max = bounding_box

        print(f"{state_file.name}")
        print(f"  Time: {state['time']:.3f} s")
        print(f"  {rock_name}")
        print(
            f"  Bounding box: "
            f"({x_min:.1f}, {y_min:.1f}) -> "
            f"({x_max:.1f}, {y_max:.1f})"
        )

        top_left = (int(round(x_min)), int(round(y_min)))
        bottom_right = (int(round(x_max)), int(round(y_max)))

        cv2.rectangle(image, top_left, bottom_right, (0, 0, 255), 2)

        # Draw the rock origin only if its center is actually visible.
        pixel = world_to_pixel(rock_position, rover_position, rover_orientation)

        if pixel is not None:

            pixel_x, pixel_y = pixel

            if 0 <= pixel_x < CAMERA_WIDTH and 0 <= pixel_y < CAMERA_HEIGHT:

                draw_x = int(round(pixel_x))
                draw_y = int(round(pixel_y))

                cv2.circle(image, (draw_x, draw_y), 8, (0, 0, 255), 2)

        # Put the name near the top-left of the bounding box.
        label_x = int(round(x_min))
        label_y = max(15, int(round(y_min)) - 5)

        cv2.putText(
            image,
            rock_name,
            (label_x, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1
        )

        yolo_box = bounding_box_to_yolo(bounding_box)

        x_center, y_center, box_width, box_height = yolo_box

        label_line = format_yolo_label(0, yolo_box)

        yolo_labels.append(label_line)

        print()

    if yolo_labels:

        label_name = state_file.name.replace("_state.json", ".txt")
        label_path = DEBUG_DIRECTORY / label_name

        with open(label_path, "w") as file:
            file.write("\n".join(yolo_labels) + "\n")

        print(f"  YOLO label: {label_path.name}")

    if frame_has_visible_rock:

        annotated_path = DEBUG_DIRECTORY / image_name.replace(".png", "_annotated.png")
        cv2.imwrite(str(annotated_path), image)

        print(f"  Annotated: {annotated_path.name}")
        print()

print()
print("========== SUMMARY ==========")
print(f"Frames checked: {len(state_files)}")
print(f"Visible rock projections: {visible_frame_count}")