from projection import CAMERA_WIDTH, CAMERA_HEIGHT


# ============================================================
# BOUNDING BOX → YOLO
# ============================================================

def bounding_box_to_yolo(
    bounding_box,
    image_width=CAMERA_WIDTH,
    image_height=CAMERA_HEIGHT
):
    """
    Convert a pixel bounding box from
    (x_min, y_min, x_max, y_max)
    into normalized YOLO format:

    (x_center, y_center, width, height)
    """

    x_min, y_min, x_max, y_max = bounding_box

    box_width = x_max - x_min
    box_height = y_max - y_min

    x_center = x_min + box_width / 2.0
    y_center = y_min + box_height / 2.0

    yolo_bounding_box = (
        x_center / image_width,
        y_center / image_height,
        box_width / image_width,
        box_height / image_height
    )

    return yolo_bounding_box


def format_yolo_label(class_id, yolo_bounding_box):
    """
    Convert a YOLO bounding box into one label-file line.
    """

    x_center, y_center, box_width, box_height = yolo_bounding_box

    return (
        f"{class_id} "
        f"{x_center:.6f} "
        f"{y_center:.6f} "
        f"{box_width:.6f} "
        f"{box_height:.6f}"
    )