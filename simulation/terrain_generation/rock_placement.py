from pathlib import Path
import sys


# ============================================================
# PROJECT IMPORT SETUP
# ============================================================

PROJECT_ROOT = (
    Path(__file__)
    .resolve()
    .parents[2]
)

if str(PROJECT_ROOT) not in sys.path:

    sys.path.insert(
        0,
        str(PROJECT_ROOT)
    )


from simulation.terrain_generation.terrain_model import (
    TerrainModel,
)


# ============================================================
# WEBOTS R2025a REGULAR ROCK GEOMETRY
# ============================================================

# Local-space vertices from the regular Rock mesh in
# Webots R2025a Rock.proto.
#
# Each vertex is:
#
#     (x, y, z)
#
# and Webots multiplies all three coordinates by the
# Rock "scale" field.

REGULAR_ROCK_VERTICES = [

    ( 0.0282555, -0.0123062,  0.0493622),
    ( 0.0514526,  0.0516891,  0.0471497),
    ( 0.0213318,  0.0449829,  0.0468445),
    ( 0.0608444,  0.0574036,  0.0324860),
    (-0.0161400,  0.0463638,  0.0209618),
    (-0.0175323,  0.0003681,  0.0294952),
    ( 0.0659485,  0.0076895, -0.0014772),
    ( 0.0296440, -0.0428085,  0.0261192),
    (-0.0213318, -0.0405960,  0.0248642),
    ( 0.0575638, -0.0246735,  0.0051146),
    ( 0.0346298,  0.0553970,  0.0064392),
    (-0.0625458, -0.0035353,  0.0255203),
    ( 0.0045910, -0.0539703, -0.0393677),
    (-0.0445328, -0.0316010,  0.0104427),
    (-0.0316048, -0.0461426, -0.0520782),
    ( 0.0381699, -0.0187950, -0.0489807),
    (-0.0555496, -0.0468445, -0.0379105),
    (-0.0018876,  0.0425644, -0.0396271),
    (-0.0668030, -0.0030928, -0.0295410),
    (-0.0359573, -0.0154018, -0.0562286),
    (-0.0384674,  0.0333023, -0.0195274),
]


# Tiny upward allowance to avoid numerical terrain
# intersection after placement.
ROCK_PLACEMENT_MARGIN_M = 0.005


# ============================================================
# ROCK Z PLACEMENT
# ============================================================

def calculate_rock_translation_z(
    terrain,
    rock_x,
    rock_y,
    rock_scale
):

    required_translation_z = float("-inf")


    for (
        local_x,
        local_y,
        local_z
    ) in REGULAR_ROCK_VERTICES:

        # Convert this rock vertex into horizontal
        # world coordinates.
        vertex_world_x = (
            rock_x
            +
            local_x * rock_scale
        )

        vertex_world_y = (
            rock_y
            +
            local_y * rock_scale
        )


        # Terrain directly underneath this rock vertex.
        terrain_z = terrain.height_at(
            vertex_world_x,
            vertex_world_y
        )


        # We require:
        #
        # translation_z + scaled_vertex_z >= terrain_z
        #
        # therefore:
        #
        # translation_z >= terrain_z - scaled_vertex_z

        candidate_translation_z = (
            terrain_z
            -
            local_z * rock_scale
        )


        required_translation_z = max(
            required_translation_z,
            candidate_translation_z
        )


    return (
        required_translation_z
        +
        ROCK_PLACEMENT_MARGIN_M
    )


# ============================================================
# TEST
# ============================================================

if __name__ == "__main__":

    terrain = TerrainModel.from_wbt()


    print()
    print(
        "========== ROCK PLACEMENT TEST =========="
    )


    test_rocks = [

        (
            "ROCK_01",
            -1.92,
            0.00,
            4.0,
            0.33
        ),

        (
            "ROCK_02",
            -5.47,
            2.21,
            2.0,
            0.70
        ),

        (
            "ROCK_03",
            -2.66,
            2.08,
            1.0,
            0.35
        ),

        (
            "ROCK_04",
            -4.31,
            1.30,
            1.0,
            0.45
        ),
    ]


    for (
        name,
        x,
        y,
        scale,
        existing_z
    ) in test_rocks:

        generated_z = (
            calculate_rock_translation_z(
                terrain,
                x,
                y,
                scale
            )
        )


        print(
            f"{name}: "
            f"scale={scale:.1f}, "
            f"generated_z={generated_z:.4f} m, "
            f"existing_z={existing_z:.4f} m, "
            f"difference="
            f"{generated_z - existing_z:+.4f} m"
        )