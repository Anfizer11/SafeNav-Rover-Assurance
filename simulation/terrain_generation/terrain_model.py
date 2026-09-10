import math
import re

from pathlib import Path


# ============================================================
# DEFAULT WORLD FILE
# ============================================================

PROJECT_ROOT = (
    Path(__file__)
    .resolve()
    .parents[2]
)

DEFAULT_WORLD_FILE = (
    PROJECT_ROOT
    / "simulation"
    / "worlds"
    / "safenav_mars_dev.wbt"
)


# ============================================================
# REGEX HELPERS
# ============================================================

FLOAT_PATTERN = (
    r"[-+]?"
    r"(?:"
    r"\d+(?:\.\d*)?"
    r"|"
    r"\.\d+"
    r")"
    r"(?:[eE][-+]?\d+)?"
)


# ============================================================
# TERRAIN MODEL
# ============================================================

class TerrainModel:

    def __init__(
        self,
        heights,
        x_dimension,
        y_dimension,
        x_spacing,
        y_spacing,
        origin_x,
        origin_y,
        origin_z
    ):

        self.heights = heights

        self.x_dimension = x_dimension
        self.y_dimension = y_dimension

        self.x_spacing = x_spacing
        self.y_spacing = y_spacing

        self.origin_x = origin_x
        self.origin_y = origin_y
        self.origin_z = origin_z


        expected_height_count = (
            self.x_dimension
            *
            self.y_dimension
        )


        if len(self.heights) != expected_height_count:

            raise ValueError(
                "ElevationGrid height count does not "
                "match xDimension * yDimension. "
                f"Expected {expected_height_count}, "
                f"got {len(self.heights)}."
            )


    # ========================================================
    # LOAD FROM WEBOTS WORLD
    # ========================================================

    @classmethod
    def from_wbt(
        cls,
        world_file=DEFAULT_WORLD_FILE
    ):

        world_file = Path(
            world_file
        )


        if not world_file.exists():

            raise FileNotFoundError(
                f"Could not find Webots world file:\n"
                f"{world_file}"
            )


        world_text = world_file.read_text(
            encoding="utf-8"
        )


        # ----------------------------------------------------
        # FIND TERRAIN TRANSLATION
        # ----------------------------------------------------

        terrain_translation_pattern = (
            r"DEF\s+terrain\s+Solid\s*\{"
            r"\s*translation\s+"
            f"({FLOAT_PATTERN})\\s+"
            f"({FLOAT_PATTERN})\\s+"
            f"({FLOAT_PATTERN})"
        )


        translation_match = re.search(
            terrain_translation_pattern,
            world_text,
            re.DOTALL
        )


        if translation_match is None:

            raise ValueError(
                "Could not locate DEF terrain Solid "
                "translation in the world file."
            )


        origin_x = float(
            translation_match.group(1)
        )

        origin_y = float(
            translation_match.group(2)
        )

        origin_z = float(
            translation_match.group(3)
        )


        # ----------------------------------------------------
        # FIND ELEVATION GRID
        # ----------------------------------------------------

        elevation_grid_pattern = (
            r"geometry\s+ElevationGrid\s*\{"
            r"\s*height\s*\[(.*?)\]"
            r"\s*xDimension\s+(\d+)"
            r"\s*xSpacing\s+"
            f"({FLOAT_PATTERN})"
            r"\s*yDimension\s+(\d+)"
            r"\s*ySpacing\s+"
            f"({FLOAT_PATTERN})"
        )


        grid_match = re.search(
            elevation_grid_pattern,
            world_text,
            re.DOTALL
        )


        if grid_match is None:

            raise ValueError(
                "Could not locate ElevationGrid "
                "in the world file."
            )


        height_text = grid_match.group(1)


        heights = [
            float(value)

            for value in re.findall(
                FLOAT_PATTERN,
                height_text
            )
        ]


        x_dimension = int(
            grid_match.group(2)
        )

        x_spacing = float(
            grid_match.group(3)
        )

        y_dimension = int(
            grid_match.group(4)
        )

        y_spacing = float(
            grid_match.group(5)
        )


        return cls(
            heights=heights,

            x_dimension=x_dimension,
            y_dimension=y_dimension,

            x_spacing=x_spacing,
            y_spacing=y_spacing,

            origin_x=origin_x,
            origin_y=origin_y,
            origin_z=origin_z
        )


    # ========================================================
    # TERRAIN BOUNDS
    # ========================================================

    def contains(
        self,
        x,
        y
    ):

        maximum_x = (
            self.origin_x
            +
            (
                self.x_dimension - 1
            )
            *
            self.x_spacing
        )

        maximum_y = (
            self.origin_y
            +
            (
                self.y_dimension - 1
            )
            *
            self.y_spacing
        )


        return (
            self.origin_x <= x <= maximum_x
            and
            self.origin_y <= y <= maximum_y
        )


    # ========================================================
    # HEIGHT ARRAY ACCESS
    # ========================================================

    def _height_at_index(
        self,
        grid_x,
        grid_y
    ):

        index = (
            grid_x
            +
            grid_y
            *
            self.x_dimension
        )


        return (
            self.heights[index]
            +
            self.origin_z
        )


    # ========================================================
    # TERRAIN HEIGHT
    # ========================================================

    def height_at(
        self,
        x,
        y
    ):

        if not self.contains(
            x,
            y
        ):

            raise ValueError(
                f"Point ({x}, {y}) lies outside "
                "the ElevationGrid."
            )


        # Convert world coordinates into fractional
        # terrain-grid coordinates.
        grid_x = (
            (x - self.origin_x)
            /
            self.x_spacing
        )

        grid_y = (
            (y - self.origin_y)
            /
            self.y_spacing
        )


        lower_x = math.floor(
            grid_x
        )

        lower_y = math.floor(
            grid_y
        )


        # If the point is exactly on the maximum edge,
        # use the final terrain cell.
        lower_x = min(
            lower_x,
            self.x_dimension - 2
        )

        lower_y = min(
            lower_y,
            self.y_dimension - 2
        )


        upper_x = lower_x + 1
        upper_y = lower_y + 1


        fraction_x = (
            grid_x - lower_x
        )

        fraction_y = (
            grid_y - lower_y
        )


        # Four surrounding terrain heights.
        h00 = self._height_at_index(
            lower_x,
            lower_y
        )

        h10 = self._height_at_index(
            upper_x,
            lower_y
        )

        h01 = self._height_at_index(
            lower_x,
            upper_y
        )

        h11 = self._height_at_index(
            upper_x,
            upper_y
        )


        # ----------------------------------------------------
        # BILINEAR INTERPOLATION
        # ----------------------------------------------------

        lower_height = (
            h00
            *
            (1.0 - fraction_x)
            +
            h10
            *
            fraction_x
        )


        upper_height = (
            h01
            *
            (1.0 - fraction_x)
            +
            h11
            *
            fraction_x
        )


        interpolated_height = (
            lower_height
            *
            (1.0 - fraction_y)
            +
            upper_height
            *
            fraction_y
        )


        return interpolated_height


# ============================================================
# CONVENIENCE FUNCTION
# ============================================================

_default_terrain_model = None


def terrain_height_at(
    x,
    y,
    world_file=DEFAULT_WORLD_FILE
):

    global _default_terrain_model


    if _default_terrain_model is None:

        _default_terrain_model = (
            TerrainModel.from_wbt(
                world_file
            )
        )


    return _default_terrain_model.height_at(
        x,
        y
    )

if __name__ == "__main__":

    terrain = TerrainModel.from_wbt()

    print()
    print("========== TERRAIN MODEL TEST ==========")

    print(
        f"Dimensions: "
        f"{terrain.x_dimension} x "
        f"{terrain.y_dimension}"
    )

    print(
        f"Spacing: "
        f"{terrain.x_spacing} m x "
        f"{terrain.y_spacing} m"
    )

    print(
        f"Origin: "
        f"({terrain.origin_x}, "
        f"{terrain.origin_y}, "
        f"{terrain.origin_z})"
    )

    print(
        f"Height samples: "
        f"{len(terrain.heights)}"
    )

    print()

    test_points = [
        ("ROCK_01", -1.92, 0.00),
        ("ROCK_02", -5.47, 2.21),
        ("ROCK_03", -2.66, 2.08),
        ("ROCK_04", -4.31, 1.30),
    ]


    for name, x, y in test_points:

        height = terrain.height_at(
            x,
            y
        )

        print(
            f"{name}: "
            f"terrain height = "
            f"{height:.4f} m"
        )