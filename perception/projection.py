import math
import numpy as np

# ============================================================
# CAMERA CONFIGURATION
# ============================================================

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FOV_RAD = 0.785


CAMERA_LOCAL_POSITION = np.array([0.28, 0.0, 0.09], dtype=float)

# ============================================================
# ROVER → CAMERA WORLD POSE
# ============================================================

def get_camera_world_pose(
    rover_position,
    rover_orientation
):
    """
    Calculates camera position and roation 
    given the rovers position and orientation.

    Returns
    -------
    camera_world position, camera_world_rotation
    """

    rover_position = np.asarray(rover_position, dtype=float)
    rover_rotation = np.asarray(rover_orientation, dtype=float).reshape(3, 3)

    camera_world_position = rover_position + rover_rotation @ CAMERA_LOCAL_POSITION
    camera_world_rotation = rover_rotation

    return camera_world_position, camera_world_rotation


# ============================================================
# WORLD → CAMERA
# ============================================================

def world_to_camera(
    world_point,
    camera_position,
    camera_rotation
):
    """
    Convert a 3D point from world coordinates
    into camera coordinates.

    Parameters
    ----------
    world_point:
        [x, y, z] position in world coordinates.

    camera_position:
        [x, y, z] camera position in world coordinates.

    camera_rotation:
        3x3 rotation matrix describing the camera's
        orientation in world coordinates.

    Returns
    -------
    numpy.ndarray:
        [x, y, z] point expressed in the camera frame.
    """

    # Convert inputs to NumPy arrays.
    world_point = np.asarray(world_point, dtype=float)
    camera_position = np.asarray(camera_position, dtype=float)
    camera_rotation = np.asarray(camera_rotation, dtype=float).reshape(3, 3)

    # TRANSLATION
    # Express the world point relative to the camera origin.
    relative_point = world_point - camera_position

    # ROTATION
    camera_point = camera_rotation.T @ relative_point


    return camera_point


# ============================================================
# CAMERA → IMAGE
# ============================================================

def camera_to_pixel(
    camera_point,
    image_width=CAMERA_WIDTH,
    image_height=CAMERA_HEIGHT,
    horizontal_fov_rad=CAMERA_FOV_RAD
):
    """
    Project a 3D camera-space point onto
    the 2D image plane.

    Returns None if the point is behind the camera.
    """

    camera_point = np.asarray(camera_point, dtype=float)

    x_camera, y_camera, z_camera = camera_point

    # X is the forward/depth axis in the Webots camera frame.
    if x_camera <= 0.0:
        return None


    # Convert horizontal field of view into focal length in pixels.
    focal_length_pixels = image_width / (2.0 * math.tan(horizontal_fov_rad / 2.0))

    center_x = image_width / 2.0
    center_y = image_height / 2.0

    # +Y points left in camera coordinates, while image x increases right.
    pixel_x = center_x - focal_length_pixels * (y_camera / x_camera)

    # +Z points up in camera coordinates, while image y increases downward.
    pixel_y = center_y - focal_length_pixels * (z_camera / x_camera)
    
    return float(pixel_x), float(pixel_y)


def world_to_pixel(
    world_point,
    rover_position,
    rover_orientation
):
    """
    Project a world-space point into the nav_camera image.
    """

    camera_position, camera_rotation = get_camera_world_pose(rover_position, rover_orientation)
    camera_point = world_to_camera(world_point, camera_position, camera_rotation)
    pixel = camera_to_pixel(camera_point)

    return pixel

# ============================================================
# ROCK → BOUNDING BOX
# ============================================================

def project_rock_bounding_box(
    rock_position,
    rock_scale,
    camera_position,
    camera_rotation,
    rock_local_vertices
):
    """
    Project the 3D geometry of one rock onto the
    camera image and return a 2D bounding box.

    Returns
    -------
    tuple or None:
        (x_min, y_min, x_max, y_max), or None if the
        rock cannot be projected into the image.
    """

    rock_position = np.asarray(rock_position, dtype=float)
    rock_local_vertices = np.asarray(rock_local_vertices, dtype=float)

    projected_pixels = []

    for local_vertex in rock_local_vertices:

        scaled_vertex = local_vertex * rock_scale
        world_vertex = rock_position + scaled_vertex

        camera_vertex = world_to_camera(world_vertex, camera_position, camera_rotation)
        pixel = camera_to_pixel(camera_vertex)

        if pixel is not None:
            projected_pixels.append(pixel)

    if not projected_pixels:
        return None

    pixel_x_values = [pixel[0] for pixel in projected_pixels]
    pixel_y_values = [pixel[1] for pixel in projected_pixels]

    x_min = min(pixel_x_values)
    y_min = min(pixel_y_values)
    x_max = max(pixel_x_values)
    y_max = max(pixel_y_values)

    x_min = max(0.0, min(x_min, CAMERA_WIDTH - 1))
    y_min = max(0.0, min(y_min, CAMERA_HEIGHT - 1))
    x_max = max(0.0, min(x_max, CAMERA_WIDTH - 1))
    y_max = max(0.0, min(y_max, CAMERA_HEIGHT - 1))

    if x_min >= x_max or y_min >= y_max:
        return None

    return x_min, y_min, x_max, y_max