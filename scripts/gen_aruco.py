#!/usr/bin/env python3
"""Generate ArUco marker models for the sar_zone world.

Textures corrupt under llvmpipe mipmapping in this environment, so markers
are built from GEOMETRY: a white base plate plus one thin black box per
black cell of the marker grid (DICT_4X4_50: 6x6 cells incl. border, plus a
1-cell white quiet zone). Crisp at any distance, no texture pipeline.

Writes sim/models/aruco_<id>/{model.sdf,model.config} for IDS.
"""
import os
import cv2

IDS = [0, 1, 2, 3]
PLATE_M = 1.0            # total panel side (incl. quiet zone)
CELLS = 8                # 6 marker cells + 1 quiet zone each side
IMG_CELLS = 6            # cells in the cv2-rendered marker (borderBits=1)
OUT_DIR = os.path.join(os.path.dirname(__file__), "..", "sim", "models")

BLACK = "0.02 0.02 0.02 1"
WHITE = "0.95 0.95 0.95 1"

BOX = """      <visual name="{name}">
        <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        <material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>
      </visual>
"""


def marker_bits(marker_id):
    """6x6 boolean grid (True = black) from the cv2-rendered marker."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, 600)  # 100 px/cell
    bits = []
    for row in range(IMG_CELLS):
        line = []
        for col in range(IMG_CELLS):
            cell = img[row * 100:(row + 1) * 100, col * 100:(col + 1) * 100]
            line.append(bool(cell.mean() < 128))
        bits.append(line)
    return bits


def build_sdf(marker_id):
    cell = PLATE_M / CELLS
    parts = [BOX.format(name="plate", x=0, y=0, z=0.005,
                        sx=PLATE_M, sy=PLATE_M, sz=0.01, c=WHITE)]
    bits = marker_bits(marker_id)
    n = 0
    for row in range(IMG_CELLS):
        for col in range(IMG_CELLS):
            if not bits[row][col]:
                continue
            # cell (0,0) is the far corner in -x/-y; grid sits inside quiet zone.
            # x is mirrored: the nadir camera view mirrors one axis, so
            # pre-mirror the geometry for the pattern to read correctly.
            x = PLATE_M / 2 - (col + 1.5) * cell
            y = -PLATE_M / 2 + (row + 1.5) * cell
            parts.append(BOX.format(name=f"cell_{n}", x=x, y=y, z=0.011,
                                    sx=cell, sy=cell, sz=0.002, c=BLACK))
            n += 1
    visuals = "".join(parts)
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="aruco_{marker_id}">
    <static>true</static>
    <pose>0 0 0 0 0 0</pose>
    <link name="base">
{visuals}    </link>
  </model>
</sdf>
"""


CONFIG = """<?xml version="1.0"?>
<model>
  <name>aruco_{i}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>sar</name><email>raccoon@example.com</email></author>
  <description>ArUco marker panel (DICT_4X4_50 id {i}, geometry-based), 1.0 m</description>
</model>
"""


def main():
    for marker_id in IDS:
        out = os.path.join(OUT_DIR, f"aruco_{marker_id}")
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, "model.sdf"), "w") as f:
            f.write(build_sdf(marker_id))
        with open(os.path.join(out, "model.config"), "w") as f:
            f.write(CONFIG.format(i=marker_id))
        png = os.path.join(out, "marker.png")
        if os.path.exists(png):
            os.remove(png)
        print(f"wrote {out}/model.sdf (id={marker_id}, geometry-based)")


if __name__ == "__main__":
    main()
