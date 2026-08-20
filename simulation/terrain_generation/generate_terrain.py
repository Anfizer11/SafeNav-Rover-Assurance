from pathlib import Path
import csv
import math
import random
import re
import shutil

# ============================================================
# SAFENAV TERRAIN CONFIGURATION
# ============================================================

SCRIPT_DIR = Path(__file__).resolve().parent

# Expected layout if you keep this file in:
# SafeNav/simulation/terrain_generation/generate_terrain.py
WORLD_FILE = SCRIPT_DIR.parent / "worlds" / "safenav_mars_dev.wbt"

# Your current Scene Tree shows: DEF terrain Solid
TERRAIN_DEF = "terrain"

# ElevationGrid configuration
X_DIMENSION = 61
Y_DIMENSION = 61
X_SPACING_M = 0.5
Y_SPACING_M = 0.5

# Center a 30 m x 30 m grid around (0, 0): -15..+15 on x/y.
CENTER_TERRAIN_AT_ORIGIN = True
TERRAIN_Z_M = 0.0
BASE_HEIGHT_M = 0.0

# Reproducible random roughness
TERRAIN_SEED = 42

# Output / safety
HEIGHT_DECIMALS = 5
MAKE_BACKUP = True
EXPORT_CSV = False
CSV_FILE = SCRIPT_DIR / "terrain_heights.csv"
MIN_ALLOWED_HEIGHT_M = -2.0
MAX_ALLOWED_HEIGHT_M = 2.0


# ============================================================
# FEATURE LISTS — EDIT THESE WHILE WORKSHOPPING
# ============================================================
# Generation order:
#   hills -> craters -> ridges -> roughness -> flat zones

HILLS = [
    {
        "center_x_m": -7.0,
        "center_y_m": 5.0,
        "height_m": 0.70,
        "sigma_x_m": 3.5,
        "sigma_y_m": 4.0,
    },
    {
        "center_x_m": 6.0,
        "center_y_m": 7.0,
        "height_m": 0.35,
        "sigma_x_m": 4.5,
        "sigma_y_m": 3.0,
    },
    {
        "center_x_m": 7.0,
        "center_y_m": -6.0,
        "height_m": 0.55,
        "sigma_x_m": 4.0,
        "sigma_y_m": 4.5,
    },
    # Negative height creates a broad depression.
    {
        "center_x_m": -3.0,
        "center_y_m": -6.0,
        "height_m": -0.18,
        "sigma_x_m": 3.0,
        "sigma_y_m": 3.0,
    },
]

CRATERS = [
    {
        "center_x_m": 3.0,
        "center_y_m": 1.0,
        "depth_m": 0.40,
        "bowl_sigma_m": 1.8,
        "rim_radius_m": 2.4,
        "rim_height_m": 0.18,
        "rim_sigma_m": 0.5,
    },
]

RIDGES = [
    {
        "start_x_m": -11.0,
        "start_y_m": -1.0,
        "end_x_m": -4.0,
        "end_y_m": 3.0,
        "height_m": 0.14,
        "width_m": 1.3,
        "end_falloff_m": 1.5,
    },
]

ROUGHNESS = {
    "enabled": True,
    "amplitude_m": 0.045,
    "cell_size_m": 3.0,
    "octaves": 3,
    "persistence": 0.50,
    "seed": TERRAIN_SEED,
}

# Useful for a rover spawn pad or goal area. target_height_m=None means
# use the terrain height already present at the center of the zone.
FLAT_ZONES = [
    {
        "center_x_m": -11.0,
        "center_y_m": -11.0,
        "radius_m": 2.0,
        "feather_m": 1.5,
        "target_height_m": None,
    },
]


# ============================================================
# GRID / COORDINATE HELPERS
# ============================================================

def terrain_width_m():
    return (X_DIMENSION - 1) * X_SPACING_M


def terrain_length_m():
    return (Y_DIMENSION - 1) * Y_SPACING_M


def terrain_origin_x_m():
    return -terrain_width_m() / 2.0 if CENTER_TERRAIN_AT_ORIGIN else 0.0


def terrain_origin_y_m():
    return -terrain_length_m() / 2.0 if CENTER_TERRAIN_AT_ORIGIN else 0.0


def grid_x_to_world_x(i):
    return terrain_origin_x_m() + i * X_SPACING_M


def grid_y_to_world_y(j):
    return terrain_origin_y_m() + j * Y_SPACING_M


def world_x_to_grid_x(x_m):
    return (x_m - terrain_origin_x_m()) / X_SPACING_M


def world_y_to_grid_y(y_m):
    return (y_m - terrain_origin_y_m()) / Y_SPACING_M


def create_flat_grid():
    return [[BASE_HEIGHT_M for _ in range(X_DIMENSION)] for _ in range(Y_DIMENSION)]


def sample_grid_nearest(grid, x_m, y_m):
    i = round(world_x_to_grid_x(x_m))
    j = round(world_y_to_grid_y(y_m))
    i = max(0, min(X_DIMENSION - 1, i))
    j = max(0, min(Y_DIMENSION - 1, j))
    return grid[j][i]


# ============================================================
# TERRAIN PRIMITIVES
# ============================================================

def add_hill(grid, center_x_m, center_y_m, height_m, sigma_x_m, sigma_y_m=None):
    """Add a smooth elliptical Gaussian hill; use negative height for a depression."""
    sigma_y_m = sigma_x_m if sigma_y_m is None else sigma_y_m
    if sigma_x_m <= 0 or sigma_y_m <= 0:
        raise ValueError("Hill sigma values must be greater than zero.")

    for j in range(Y_DIMENSION):
        y = grid_y_to_world_y(j)
        for i in range(X_DIMENSION):
            x = grid_x_to_world_x(i)
            dx = (x - center_x_m) / sigma_x_m
            dy = (y - center_y_m) / sigma_y_m
            grid[j][i] += height_m * math.exp(-0.5 * (dx * dx + dy * dy))


def add_crater(
    grid,
    center_x_m,
    center_y_m,
    depth_m,
    bowl_sigma_m,
    rim_radius_m,
    rim_height_m,
    rim_sigma_m,
):
    """Add a crater with a depressed bowl and raised ring-shaped rim."""
    if depth_m < 0:
        raise ValueError("Crater depth_m should be positive.")
    if bowl_sigma_m <= 0 or rim_radius_m <= 0 or rim_sigma_m <= 0:
        raise ValueError("Crater size parameters must be greater than zero.")

    for j in range(Y_DIMENSION):
        y = grid_y_to_world_y(j)
        for i in range(X_DIMENSION):
            x = grid_x_to_world_x(i)
            r = math.hypot(x - center_x_m, y - center_y_m)
            bowl = -depth_m * math.exp(-0.5 * (r / bowl_sigma_m) ** 2)
            rim = rim_height_m * math.exp(-0.5 * ((r - rim_radius_m) / rim_sigma_m) ** 2)
            grid[j][i] += bowl + rim


def _distance_to_segment(px, py, ax, ay, bx, by):
    """Return perpendicular distance to segment and distance beyond an endpoint."""
    vx = bx - ax
    vy = by - ay
    length_sq = vx * vx + vy * vy
    if length_sq == 0:
        return math.hypot(px - ax, py - ay), 0.0

    length = math.sqrt(length_sq)
    t = ((px - ax) * vx + (py - ay) * vy) / length_sq

    if t < 0.0:
        cx, cy = ax, ay
        beyond = -t * length
    elif t > 1.0:
        cx, cy = bx, by
        beyond = (t - 1.0) * length
    else:
        cx = ax + t * vx
        cy = ay + t * vy
        beyond = 0.0

    return math.hypot(px - cx, py - cy), beyond


def add_ridge(
    grid,
    start_x_m,
    start_y_m,
    end_x_m,
    end_y_m,
    height_m,
    width_m,
    end_falloff_m=1.0,
):
    """Add a smooth raised strip; use negative height for a trench-like strip."""
    if width_m <= 0 or end_falloff_m <= 0:
        raise ValueError("Ridge width and end falloff must be positive.")

    for j in range(Y_DIMENSION):
        y = grid_y_to_world_y(j)
        for i in range(X_DIMENSION):
            x = grid_x_to_world_x(i)
            perpendicular, beyond = _distance_to_segment(
                x, y, start_x_m, start_y_m, end_x_m, end_y_m
            )
            cross_section = math.exp(-0.5 * (perpendicular / width_m) ** 2)
            endpoint_fade = math.exp(-0.5 * (beyond / end_falloff_m) ** 2)
            grid[j][i] += height_m * cross_section * endpoint_fade


# ============================================================
# SMOOTH MULTI-SCALE ROUGHNESS (STANDARD LIBRARY ONLY)
# ============================================================

def _smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def _lerp(a, b, t):
    return a + (b - a) * t


def _make_random_lattice(x_cells, y_cells, rng):
    return [
        [rng.uniform(-1.0, 1.0) for _ in range(x_cells + 1)]
        for _ in range(y_cells + 1)
    ]


def _sample_value_noise(lattice, x_cell, y_cell):
    max_y = len(lattice) - 1
    max_x = len(lattice[0]) - 1

    floor_x = math.floor(x_cell)
    floor_y = math.floor(y_cell)
    x0 = max(0, min(max_x - 1, int(floor_x)))
    y0 = max(0, min(max_y - 1, int(floor_y)))
    x1 = x0 + 1
    y1 = y0 + 1

    tx = _smoothstep(x_cell - floor_x)
    ty = _smoothstep(y_cell - floor_y)

    top = _lerp(lattice[y0][x0], lattice[y0][x1], tx)
    bottom = _lerp(lattice[y1][x0], lattice[y1][x1], tx)
    return _lerp(top, bottom, ty)


def add_roughness(grid, amplitude_m, cell_size_m, octaves=3, persistence=0.5, seed=42):
    """Add smooth reproducible roughness without jagged per-vertex random spikes."""
    if amplitude_m < 0:
        raise ValueError("Roughness amplitude cannot be negative.")
    if cell_size_m <= 0:
        raise ValueError("Roughness cell size must be greater than zero.")
    if octaves < 1:
        raise ValueError("Roughness octaves must be at least 1.")
    if not 0 < persistence <= 1:
        raise ValueError("Roughness persistence must be in (0, 1].")

    rng = random.Random(seed)
    total_width = terrain_width_m()
    total_length = terrain_length_m()
    octave_amplitude = amplitude_m
    octave_cell_size = cell_size_m

    for _ in range(octaves):
        x_cells = max(1, math.ceil(total_width / octave_cell_size))
        y_cells = max(1, math.ceil(total_length / octave_cell_size))
        lattice = _make_random_lattice(x_cells, y_cells, rng)

        for j in range(Y_DIMENSION):
            y_cell = (j * Y_SPACING_M / total_length) * y_cells
            if j == Y_DIMENSION - 1:
                y_cell = y_cells - 1e-12

            for i in range(X_DIMENSION):
                x_cell = (i * X_SPACING_M / total_width) * x_cells
                if i == X_DIMENSION - 1:
                    x_cell = x_cells - 1e-12

                grid[j][i] += octave_amplitude * _sample_value_noise(
                    lattice, x_cell, y_cell
                )

        octave_amplitude *= persistence
        octave_cell_size /= 2.0


# ============================================================
# FLAT / SAFE ZONES
# ============================================================

def flatten_zone(grid, center_x_m, center_y_m, radius_m, feather_m, target_height_m=None):
    """Flatten a circular region and smoothly feather it into surrounding terrain."""
    if radius_m < 0 or feather_m < 0:
        raise ValueError("Flatten radius and feather cannot be negative.")

    if target_height_m is None:
        target_height_m = sample_grid_nearest(grid, center_x_m, center_y_m)

    outer_radius = radius_m + feather_m

    for j in range(Y_DIMENSION):
        y = grid_y_to_world_y(j)
        for i in range(X_DIMENSION):
            x = grid_x_to_world_x(i)
            distance = math.hypot(x - center_x_m, y - center_y_m)

            if distance <= radius_m:
                grid[j][i] = target_height_m
            elif feather_m > 0 and distance < outer_radius:
                t = _smoothstep((distance - radius_m) / feather_m)
                grid[j][i] = _lerp(target_height_m, grid[j][i], t)


# ============================================================
# BUILD PIPELINE
# ============================================================

def build_terrain():
    grid = create_flat_grid()

    for hill in HILLS:
        add_hill(grid=grid, **hill)

    for crater in CRATERS:
        add_crater(grid=grid, **crater)

    for ridge in RIDGES:
        add_ridge(grid=grid, **ridge)

    if ROUGHNESS.get("enabled", False):
        add_roughness(
            grid=grid,
            amplitude_m=ROUGHNESS["amplitude_m"],
            cell_size_m=ROUGHNESS["cell_size_m"],
            octaves=ROUGHNESS.get("octaves", 3),
            persistence=ROUGHNESS.get("persistence", 0.5),
            seed=ROUGHNESS.get("seed", TERRAIN_SEED),
        )

    for zone in FLAT_ZONES:
        flatten_zone(grid=grid, **zone)

    validate_grid(grid)
    return grid


def validate_grid(grid):
    if len(grid) != Y_DIMENSION or any(len(row) != X_DIMENSION for row in grid):
        raise ValueError("Generated grid dimensions do not match X_DIMENSION/Y_DIMENSION.")

    minimum = min(min(row) for row in grid)
    maximum = max(max(row) for row in grid)

    if minimum < MIN_ALLOWED_HEIGHT_M:
        raise ValueError(
            f"Terrain reaches {minimum:.3f} m, below MIN_ALLOWED_HEIGHT_M={MIN_ALLOWED_HEIGHT_M}."
        )
    if maximum > MAX_ALLOWED_HEIGHT_M:
        raise ValueError(
            f"Terrain reaches {maximum:.3f} m, above MAX_ALLOWED_HEIGHT_M={MAX_ALLOWED_HEIGHT_M}."
        )


# ============================================================
# OPTIONAL CSV EXPORT
# ============================================================

def export_csv(grid):
    with CSV_FILE.open("w", newline="") as file:
        writer = csv.writer(file)
        for row in grid:
            writer.writerow([f"{value:.{HEIGHT_DECIMALS}f}" for value in row])
    print(f"Height CSV written to: {CSV_FILE}")


# ============================================================
# WEBOTS .WBT UPDATE HELPERS
# ============================================================

def find_matching_character(text, opening_index, opening_char, closing_char):
    depth = 0
    in_string = False
    escaped = False
    in_comment = False

    for index in range(opening_index, len(text)):
        char = text[index]

        if in_comment:
            if char == "\n":
                in_comment = False
            continue

        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            continue

        if char == "#":
            in_comment = True
            continue
        if char == '"':
            in_string = True
            continue

        if char == opening_char:
            depth += 1
        elif char == closing_char:
            depth -= 1
            if depth == 0:
                return index

    raise ValueError(f"Could not find matching '{closing_char}'.")


def find_terrain_solid(text):
    pattern = rf"\bDEF\s+{re.escape(TERRAIN_DEF)}\s+Solid\s*\{{"
    match = re.search(pattern, text)
    if not match:
        raise ValueError(
            f'Could not find "DEF {TERRAIN_DEF} Solid" in {WORLD_FILE}. '
            "Check TERRAIN_DEF at the top of this script."
        )

    opening_brace = text.find("{", match.start())
    closing_brace = find_matching_character(text, opening_brace, "{", "}")
    return match.start(), closing_brace


def replace_or_insert_scalar_field(block, field_name, value):
    pattern = rf"(?m)^([ \t]*){re.escape(field_name)}\s+[^\r\n#]+"
    match = re.search(pattern, block)

    if match:
        indentation = match.group(1)
        replacement = f"{indentation}{field_name} {value}"
        return block[:match.start()] + replacement + block[match.end():]

    opening_brace = block.find("{")
    return block[:opening_brace + 1] + f"\n  {field_name} {value}" + block[opening_brace + 1:]


def format_height_rows(grid, indentation):
    return "\n".join(
        indentation + " ".join(f"{value:.{HEIGHT_DECIMALS}f}" for value in row)
        for row in grid
    )


def replace_height_field(elevation_grid_block, grid):
    match = re.search(r"(?m)^([ \t]*)height\s*\[", elevation_grid_block)

    if match:
        field_indent = match.group(1)
        row_indent = field_indent + "  "
        opening_bracket = elevation_grid_block.find("[", match.start())
        closing_bracket = find_matching_character(
            elevation_grid_block, opening_bracket, "[", "]"
        )
        replacement = (
            f"{field_indent}height [\n"
            f"{format_height_rows(grid, row_indent)}\n"
            f"{field_indent}]"
        )
        return (
            elevation_grid_block[:match.start()]
            + replacement
            + elevation_grid_block[closing_bracket + 1:]
        )

    opening_brace = elevation_grid_block.find("{")
    replacement = "\n  height [\n" + format_height_rows(grid, "    ") + "\n  ]"
    return (
        elevation_grid_block[:opening_brace + 1]
        + replacement
        + elevation_grid_block[opening_brace + 1:]
    )


def update_world_file(grid):
    if not WORLD_FILE.exists():
        raise FileNotFoundError(f"World file does not exist:\n{WORLD_FILE}")

    original_text = WORLD_FILE.read_text(encoding="utf-8")
    solid_start, solid_end = find_terrain_solid(original_text)
    solid_block = original_text[solid_start:solid_end + 1]

    translation = f"{terrain_origin_x_m():g} {terrain_origin_y_m():g} {TERRAIN_Z_M:g}"
    solid_block = replace_or_insert_scalar_field(solid_block, "translation", translation)

    grid_match = re.search(r"\bElevationGrid\s*\{", solid_block)
    if not grid_match:
        raise ValueError(f'No ElevationGrid found inside "DEF {TERRAIN_DEF} Solid".')

    grid_open = solid_block.find("{", grid_match.start())
    grid_close = find_matching_character(solid_block, grid_open, "{", "}")
    elevation_grid_block = solid_block[grid_match.start():grid_close + 1]

    elevation_grid_block = replace_or_insert_scalar_field(
        elevation_grid_block, "xDimension", str(X_DIMENSION)
    )
    elevation_grid_block = replace_or_insert_scalar_field(
        elevation_grid_block, "xSpacing", str(X_SPACING_M)
    )
    elevation_grid_block = replace_or_insert_scalar_field(
        elevation_grid_block, "yDimension", str(Y_DIMENSION)
    )
    elevation_grid_block = replace_or_insert_scalar_field(
        elevation_grid_block, "ySpacing", str(Y_SPACING_M)
    )
    elevation_grid_block = replace_height_field(elevation_grid_block, grid)

    solid_block = (
        solid_block[:grid_match.start()]
        + elevation_grid_block
        + solid_block[grid_close + 1:]
    )
    updated_text = original_text[:solid_start] + solid_block + original_text[solid_end + 1:]

    if MAKE_BACKUP:
        backup_file = WORLD_FILE.with_suffix(WORLD_FILE.suffix + ".bak")
        shutil.copy2(WORLD_FILE, backup_file)
        print(f"Backup created: {backup_file}")

    WORLD_FILE.write_text(updated_text, encoding="utf-8")


# ============================================================
# MAIN
# ============================================================

def main():
    print("Generating SafeNav terrain...\n")
    print(f"World: {WORLD_FILE}")
    print(f"Grid: {X_DIMENSION} x {Y_DIMENSION}")
    print(f"Terrain size: {terrain_width_m():.2f} m x {terrain_length_m():.2f} m")
    print(f"Height samples: {X_DIMENSION * Y_DIMENSION}")
    print(f"Seed: {TERRAIN_SEED}\n")

    grid = build_terrain()
    minimum = min(min(row) for row in grid)
    maximum = max(max(row) for row in grid)

    print(f"Minimum height: {minimum:.3f} m")
    print(f"Maximum height: {maximum:.3f} m")

    if EXPORT_CSV:
        export_csv(grid)

    update_world_file(grid)

    print("\nFeature summary:")
    print(f"  Hills/depressions: {len(HILLS)}")
    print(f"  Craters:           {len(CRATERS)}")
    print(f"  Ridges:            {len(RIDGES)}")
    print(f"  Roughness:         {'enabled' if ROUGHNESS.get('enabled', False) else 'disabled'}")
    print(f"  Flat zones:        {len(FLAT_ZONES)}")

    print("\nTerrain world bounds:")
    print(
        f"  X: {terrain_origin_x_m():.2f} to "
        f"{terrain_origin_x_m() + terrain_width_m():.2f} m"
    )
    print(
        f"  Y: {terrain_origin_y_m():.2f} to "
        f"{terrain_origin_y_m() + terrain_length_m():.2f} m"
    )

    print(f"\nUpdated: {WORLD_FILE}")
    print("Reload the .wbt world in Webots to see the changes.")


if __name__ == "__main__":
    main()
