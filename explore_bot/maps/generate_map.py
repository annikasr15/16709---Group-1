#!/usr/bin/env python3
"""
Assignment 3, Task 4 Part B -- generates map.pgm + map.yaml from the SAME
obstacle/wall coordinates as ../worlds/arena.sdf, so the static map handed
to map_server lines up exactly with the physics world.

This is a provided/reference file (how the map was made), not something
you need to run or edit -- map.pgm and map.yaml are already generated and
sitting next to this script. Only rerun this if you deliberately change
arena.sdf's geometry (and if you do, update the OBSTACLES list below to
match, by hand -- there's no auto-sync).

Pure stdlib -- writes a binary PGM (P5) directly, no Pillow dependency.
"""

import struct

RESOLUTION = 0.05  # meters/pixel
ORIGIN_X = -3.5     # world x of the map image's bottom-left corner
ORIGIN_Y = -3.5     # world y of the map image's bottom-left corner
WIDTH_M = 7.0
HEIGHT_M = 7.0

WIDTH_PX = int(WIDTH_M / RESOLUTION)   # 140
HEIGHT_PX = int(HEIGHT_M / RESOLUTION)  # 140

FREE = 254
OCCUPIED = 0

# Axis-aligned boxes (cx, cy, size_x, size_y) -- MUST match arena.sdf's
# <pose> and <box><size> for each wall/obstacle model.
BOXES = [
    (0, 3, 6.1, 0.1),      # wall_north
    (0, -3, 6.1, 0.1),     # wall_south
    (3, 0, 0.1, 6.1),      # wall_east
    (-3, 0, 0.1, 6.1),     # wall_west
    (0, -0.5, 1.0, 0.3),   # obstacle_a
    (-1.0, 1.0, 0.4, 0.4),  # obstacle_b
    (1.2, 0.3, 0.4, 0.6),  # obstacle_c
]


def is_occupied(x, y):
    for cx, cy, sx, sy in BOXES:
        if abs(x - cx) <= sx / 2.0 and abs(y - cy) <= sy / 2.0:
            return True
    return False


def main():
    pixels = bytearray(WIDTH_PX * HEIGHT_PX)
    for row in range(HEIGHT_PX):
        # row 0 = top of image = max world y
        y = ORIGIN_Y + (HEIGHT_PX - row - 0.5) * RESOLUTION
        for col in range(WIDTH_PX):
            x = ORIGIN_X + (col + 0.5) * RESOLUTION
            pixels[row * WIDTH_PX + col] = OCCUPIED if is_occupied(x, y) else FREE

    with open("map.pgm", "wb") as f:
        header = f"P5\n{WIDTH_PX} {HEIGHT_PX}\n255\n".encode("ascii")
        f.write(header)
        f.write(bytes(pixels))

    with open("map.yaml", "w") as f:
        f.write(
            "image: map.pgm\n"
            f"resolution: {RESOLUTION}\n"
            f"origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )

    print(f"Wrote map.pgm ({WIDTH_PX}x{HEIGHT_PX}) and map.yaml")


if __name__ == "__main__":
    main()
